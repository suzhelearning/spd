"""ROS 2 command mailbox and physics-thread-only MuJoCo executor."""
from __future__ import annotations

from dataclasses import dataclass
import math
import threading
import time
from typing import Any, Callable

import numpy as np

from spd_vr.interfaces.ros_joint_command import JointCommandError, JointCommandSnapshot, MAX_AGE_NS, TOPIC, VALID_READY_MASK, best_effort_qos, snapshot_from_ros


@dataclass(frozen=True, slots=True)
class AppliedCommand:
    snapshot: JointCommandSnapshot
    applied_sim_time_ns: int
    hold_mask: int
    position_rad: tuple[float, ...]


class JointCommandMailbox:
    """Keep one legal candidate; callbacks never modify the plant."""

    def __init__(self, validator: Callable[[JointCommandSnapshot], Any] | None = None) -> None:
        self._lock = threading.RLock()
        self._validator = validator
        self._latest: JointCommandSnapshot | None = None
        self._pending: JointCommandSnapshot | None = None
        self._enabled = False
        self._authorized_session: str | None = None
        self._last_session: str | None = None
        self._last_sequence: int | None = None
        self._last_stamp_ns: int | None = None
        self.received = 0
        self.accepted = 0
        self.rejected = 0
        self.last_reject_reason = ""

    @property
    def enabled(self) -> bool:
        with self._lock:
            return self._enabled

    @property
    def authorized_session(self) -> str | None:
        with self._lock:
            return self._authorized_session

    @property
    def latest(self) -> JointCommandSnapshot | None:
        with self._lock:
            return self._latest

    def _reject(self, reason: Any) -> bool:
        self.rejected += 1
        self.last_reject_reason = str(reason)
        return False

    def receive(self, message: Any, *, now_ns: int | None = None) -> bool:
        now = int(time.time_ns() if now_ns is None else now_ns)
        with self._lock:
            self.received += 1
            try:
                snapshot = snapshot_from_ros(message, now_ns=now)
                if self._validator is not None:
                    self._validator(snapshot)
                if self._last_session == snapshot.session_id:
                    snapshot.validate(previous_sequence=self._last_sequence,
                                      previous_stamp_ns=self._last_stamp_ns)
            except (AttributeError, JointCommandError, TypeError, ValueError, OverflowError) as exc:
                return self._reject(exc)
            if self._authorized_session is not None and snapshot.session_id != self._authorized_session:
                self._enabled = False
                self._authorized_session = None
                self._pending = None
                self.last_reject_reason = "session changed; explicit authorization required"
            self._latest = snapshot
            self._last_session = snapshot.session_id
            self._last_sequence = snapshot.sequence
            self._last_stamp_ns = snapshot.stamp_ns
            self.accepted += 1
            if self._enabled:
                self._pending = snapshot
            return True

    def authorize(self, enabled: bool, *, session_id: str | None = None,
                  now_ns: int | None = None) -> bool:
        """Low-level mailbox gate; runtime operators must use executor.authorize."""
        with self._lock:
            self._enabled = False
            self._authorized_session = None
            self._pending = None
            if not enabled:
                return True
            candidate = self._latest
            if candidate is None:
                self.last_reject_reason = "no legal command candidate"
                return False
            try:
                candidate.validate(now_ns=time.time_ns() if now_ns is None else now_ns)
                if self._validator is not None:
                    self._validator(candidate)
                if not candidate.ready_mask:
                    raise JointCommandError("candidate has no ready groups")
                if session_id is not None and candidate.session_id != session_id:
                    raise JointCommandError("candidate session does not match authorization")
            except (TypeError, ValueError) as exc:
                self.last_reject_reason = str(exc)
                return False
            self._enabled = True
            self._authorized_session = candidate.session_id
            self._pending = candidate
            return True

    def clear(self) -> None:
        with self._lock:
            self._latest = None
            self._pending = None
            self._last_session = None
            self._last_sequence = None
            self._last_stamp_ns = None
            self._authorized_session = None
            self._enabled = False
            self.last_reject_reason = ""

    def take_pending(self) -> JointCommandSnapshot | None:
        with self._lock:
            value, self._pending = self._pending, None
            return value


class RosJointCommandExecutor:
    """Authorize retained targets and latch per-group timeouts at physics ticks."""

    def __init__(self, node: Any, plant: Any, mailbox: JointCommandMailbox | None = None,
                 *, max_enable_delta_rad: float = 0.15) -> None:
        from tianji_spd_interfaces.msg import JointCommand

        if not math.isfinite(max_enable_delta_rad) or max_enable_delta_rad < 0:
            raise ValueError("max_enable_delta_rad must be finite and nonnegative")
        self.node = node
        self.plant = plant
        self.max_enable_delta_rad = float(max_enable_delta_rad)
        self.mailbox = mailbox or JointCommandMailbox()
        self.mailbox._validator = plant.validate_joint_command
        self._held_targets = plant.joint_command_targets()
        self._groups = ((1, slice(0, 14)), (4, slice(14, 34)), (2, slice(34, 54)))
        self._last_ready_ns: dict[int, int] = {}
        self._latched_hold = VALID_READY_MASK
        self._hold_mask = VALID_READY_MASK
        self.subscription = node.create_subscription(
            JointCommand, TOPIC, self._on_message, best_effort_qos(),
        )

    @property
    def hold_mask(self) -> int:
        with self.mailbox._lock:
            return self._hold_mask if self.mailbox._enabled else VALID_READY_MASK

    @property
    def state(self) -> str:
        with self.mailbox._lock:
            if not self.mailbox._enabled:
                return "candidate" if self.mailbox._latest is not None else "disabled"
            return "holding" if self._hold_mask else "enabled"

    def _on_message(self, message: Any) -> None:
        self.mailbox.receive(message)

    def authorize(self, enabled: bool) -> bool:
        """Thread-safe operator gate; never reset or modify MuJoCo state."""
        with self.mailbox._lock:
            if not enabled:
                self._hold_mask = self._latched_hold = VALID_READY_MASK
                return self.mailbox.authorize(False)
            candidate = self.mailbox._latest
            self.mailbox.authorize(False)
            self._hold_mask = self._latched_hold = VALID_READY_MASK
            if candidate is None:
                self.mailbox.last_reject_reason = "no legal command candidate"
                return False
            for bit, indices in self._groups:
                if candidate.ready_mask & bit and np.any(
                    np.abs(np.asarray(candidate.position_rad[indices]) - self._held_targets[indices])
                    > self.max_enable_delta_rad
                ):
                    self.mailbox.last_reject_reason = "candidate exceeds enable delta from retained target"
                    return False
            if not self.mailbox.authorize(True):
                return False
            now = time.monotonic_ns()
            self._last_ready_ns = {bit: now for bit, _ in self._groups}
            self._latched_hold = 0
            self._hold_mask = VALID_READY_MASK ^ candidate.ready_mask
            return True


    def clear(self) -> None:
        """Clear control only; physical positions and retained targets do not move."""
        with self.mailbox._lock:
            self.mailbox.clear()
            self._last_ready_ns.clear()
            self._hold_mask = self._latched_hold = VALID_READY_MASK

    def apply_pending(self, *, now_ns: int | None = None) -> AppliedCommand | None:
        """Called only on the physics thread immediately before integration.

        ``now_ns`` is monotonic; wire age is checked separately against UTC.
        A timed-out group stays held even if new traffic arrives, until enable.
        """
        now = int(time.monotonic_ns() if now_ns is None else now_ns)
        with self.mailbox._lock:
            if not self.mailbox._enabled:
                self._hold_mask = VALID_READY_MASK
                self.plant.set_joint_command_hold(self._hold_mask)
                return None
            for bit, _ in self._groups:
                if now - self._last_ready_ns.get(bit, now) > MAX_AGE_NS:
                    self._latched_hold |= bit
            self._hold_mask |= self._latched_hold
            self.plant.set_joint_command_hold(self._hold_mask)
            snapshot = self.mailbox.take_pending()
            if snapshot is None:
                return None
            try:
                utc_now = time.time_ns()
                snapshot.validate(now_ns=utc_now)
                self.plant.submit_joint_command(snapshot, hold_mask=self._latched_hold)
            except (TypeError, ValueError) as exc:
                self.mailbox._reject(exc)
                return None
            self._hold_mask = (VALID_READY_MASK ^ snapshot.ready_mask) | self._latched_hold
            source_monotonic = now - max(0, utc_now - snapshot.stamp_ns)
            for bit, _ in self._groups:
                if snapshot.ready_mask & bit and not self._latched_hold & bit:
                    self._last_ready_ns[bit] = source_monotonic
            self._held_targets = self.plant.joint_command_targets()
            return AppliedCommand(snapshot, int(self.plant.sim_time_ns), self._hold_mask,
                                  tuple(float(value) for value in self._held_targets))


class ControlTerminal:
    """Optional stdin authorization and recording controls."""

    def __init__(self, executor: RosJointCommandExecutor,
                 recording_control: Callable[[str], None]) -> None:
        self.executor = executor
        self._recording_control = recording_control
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="spd-ros-control", daemon=True)
        self._thread.start()

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                key = input().strip().lower()
            except EOFError:
                return
            if key == "c":
                self.executor.clear()
            elif key == "e":
                self.executor.authorize(not self.executor.mailbox.enabled)
            elif key in {"r", "s", "d"}:
                self._recording_control({"r": "start", "s": "success", "d": "discard"}[key])

    def close(self) -> None:
        self._stop.set()


__all__ = ["AppliedCommand", "ControlTerminal", "JointCommandMailbox", "RosJointCommandExecutor"]
