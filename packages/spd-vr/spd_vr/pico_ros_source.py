"""Single-owner PICO normalization, calibration and bounded ROS command source.

Receiver callbacks must only enqueue frames with their monotonic receipt time.
All methods here (including connection events) run on the solver thread.
"""
from __future__ import annotations

from collections.abc import Callable
import math
from pathlib import Path
import time
from typing import Any
import uuid

import numpy as np

from .ros_joint_command import (
    ARMS_READY, LEFT_HAND_READY, RIGHT_HAND_READY, VALID_READY_MASK,
    JOINT_NAMES, JointCommandSnapshot,
)
from .wire import TrackingFrame, decode_tracking, encode_tracking

_IDENTITY_POSE = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
_GROUPS = ((ARMS_READY, slice(0, 14)), (LEFT_HAND_READY, slice(14, 34)),
           (RIGHT_HAND_READY, slice(34, 54)))


def _pose_values(pose: Any, name: str) -> tuple[float, ...]:
    position = tuple(float(v) for v in pose.position)
    quaternion = tuple(float(v) for v in pose.quaternion_xyzw)
    if len(position) != 3 or len(quaternion) != 4:
        raise ValueError(f"{name} must contain a position and quaternion")
    if not all(math.isfinite(v) for v in position + quaternion):
        raise ValueError(f"{name} must contain finite values")
    norm = math.sqrt(sum(v * v for v in quaternion))
    if not math.isfinite(norm) or norm < 1.0e-8:
        raise ValueError(f"{name} has an invalid quaternion")
    return position + tuple(v / norm for v in quaternion)


def _canonical_hand(hand: Any, side: str) -> tuple[bool, tuple[tuple[float, ...], ...]]:
    try:
        joints = tuple(hand.joints)
        if not hand.valid or len(joints) != 26 or not all(joint.valid for joint in joints):
            raise ValueError("inactive or incomplete hand")
        return True, tuple(_pose_values(joint, f"{side}.joint[{i}]") for i, joint in enumerate(joints))
    except (AttributeError, TypeError, ValueError, OverflowError):
        return False, (_IDENTITY_POSE,) * 26


class PicoRosSourceCore:
    """Normalize raw frames without ever refreshing a duplicate/stale receipt."""

    def __init__(self, *, clock_ns: Callable[[], int] = time.monotonic_ns) -> None:
        self._clock_ns = clock_ns
        self._epoch = 0
        self._sequence = 0
        self._last_timestamp_ms: int | None = None
        self._last_bridge_ns = 0
        self.last_error = ""

    @property
    def epoch(self) -> int:
        return self._epoch

    def reset_stream(self) -> None:
        self._epoch += 1
        self._last_timestamp_ms = None
        self._last_bridge_ns = 0

    def accept_frame(self, frame: Any, *, received_ns: int | None = None) -> bytes | None:
        self.last_error = ""
        try:
            timestamp_ms = int(frame.timestamp_ms)
            if timestamp_ms < 0:
                raise ValueError("timestamp_ms must be non-negative")
            receipt = int(self._clock_ns() if received_ns is None else received_ns)
            if receipt <= 0 or receipt <= self._last_bridge_ns:
                if receipt < self._last_bridge_ns:
                    self.reset_stream()
                raise ValueError("monotonic receipt rollback")
            if self._epoch == 0:
                self.reset_stream()
            if self._last_timestamp_ms is not None:
                if timestamp_ms == self._last_timestamp_ms:
                    self.last_error = "duplicate"
                    return None
                if timestamp_ms < self._last_timestamp_ms:
                    self.reset_stream()
            head_valid = bool(frame.head.valid)
            head = _pose_values(frame.head, "head") if head_valid else _IDENTITY_POSE
            left_active, left_hand = _canonical_hand(frame.left, "left")
            right_active, right_hand = _canonical_hand(frame.right, "right")
            tracking = TrackingFrame(
                sequence=self._sequence + 1, tracking_epoch=self._epoch,
                source_timestamp_ns=max(1, timestamp_ms * 1_000_000),
                bridge_monotonic_ns=receipt, left_active=left_active,
                right_active=right_active, head_valid=head_valid,
                left_scale=1.0, right_scale=1.0, head_pose=head,
                left_hand=left_hand, right_hand=right_hand,
            )
            packet = encode_tracking(tracking)
        except (AttributeError, TypeError, ValueError, OverflowError) as exc:
            self.last_error = str(exc)
            return None
        self._sequence += 1
        self._last_timestamp_ms = timestamp_ms
        self._last_bridge_ns = receipt
        return packet


class PicoTeleopCore:
    """Real solvers with explicit Align -> receiver authorization -> Start interlock.

    The wire has one arms bit: either wrist/IK failure holds BOTH arms. Hand
    groups latch independently. Every lost group requires another Align/Start;
    fresh input alone never restores its ready bit. Calibration and all solver
    state are owned by the caller, never by the TCP or UI threads.
    """

    PERIOD_NS = 5_000_000
    FRESH_NS = 50_000_000  # Matches SideAlignment, stricter than receiver 100 ms.

    @classmethod
    def from_production(cls) -> "PicoTeleopCore":
        from .arm_ik import _production_controller, _verified_model
        from .model_builder import workspace_root
        from .retarget_pair import WujiRetargetPair

        package = Path(__file__).resolve().parents[1]
        manifest_path = package / "generated/model_manifest.yaml"
        asset_root = workspace_root() / "assets/tianji_wuji2"
        urdf = asset_root / "tianji_wuji2.urdf"
        model, verified = _verified_model(package / "generated/arm_ik.xml", manifest_path, urdf)
        pair = WujiRetargetPair.from_manifest(
            package / "config/wuji2_pico_left.yaml", package / "config/wuji2_pico_right.yaml",
            manifest_path, urdf,
        )
        return cls(_production_controller(model, verified), pair, verified.manifest)

    def __init__(self, arm: Any, hands: Any, manifest: dict[str, Any], *,
                 clock_ns: Callable[[], int] = time.monotonic_ns) -> None:
        self.arm, self.hands = arm, hands
        self._clock_ns = clock_ns
        self.normalizer = PicoRosSourceCore(clock_ns=clock_ns)
        entries = manifest["joints"]
        by_name = {entry["joint"]: entry for entry in entries}
        if len(by_name) != 54 or set(by_name) != set(JOINT_NAMES):
            raise ValueError("manifest must bind exactly the canonical 54 joints")
        solver_names = [entry["joint"] for side in ("left", "right") for entry in entries
                        if entry["group"] == "arm" and entry["side"] == side]
        hand_names = [entry["joint"] for side in ("left", "right") for entry in entries
                      if entry["group"] == "hand" and entry["side"] == side]
        self._perm = np.asarray([ (solver_names + hand_names).index(name) for name in JOINT_NAMES ])
        self.limits = np.asarray([by_name[name]["range"] for name in JOINT_NAMES], dtype=float)
        velocities = np.asarray([by_name[name]["velocity_limit"] for name in JOINT_NAMES], dtype=float)
        if (self.limits.shape != (54, 2) or not np.all(np.isfinite(self.limits))
                or np.any(self.limits[:, 0] >= self.limits[:, 1])
                or not np.all(np.isfinite(velocities)) or np.any(velocities <= 0)):
            raise ValueError("invalid manifest joint limits")
        # Conservative source speed, not a relaxation of receiver safety gates.
        self.rates = np.minimum(velocities, 0.5)
        self.position = self.limits.mean(axis=1)
        self.position[:14] = np.concatenate((arm.left_q, arm.right_q))[self._perm[:14]]
        if (not np.all(np.isfinite(self.position))
                or np.any(self.position < self.limits[:, 0])
                or np.any(self.position > self.limits[:, 1])):
            raise ValueError("held HOME target is outside manifest limits")
        self._hand_goal = self.position.copy()
        self.tracking: TrackingFrame | None = None
        self.input_mask = self.ready_mask = self.running_mask = 0
        self._aligning = False
        self._last_tick_ns: int | None = None
        self._sequence = 0
        self.session_id = uuid.uuid4().hex
        self.reason = "waiting for PICO; Align, authorize SPD, then Start"

    def _drop(self, mask: int, reason: str) -> None:
        self.ready_mask &= ~mask
        self.running_mask &= ~mask
        if mask & ARMS_READY:
            self._aligning = False
        self.reason = reason

    def connected(self) -> None:
        self.normalizer.reset_stream()
        self.tracking = None
        self.input_mask = 0
        self._drop(VALID_READY_MASK, "connected; waiting for fresh input and Align")

    def disconnected(self) -> None:
        self.tracking = None
        self.input_mask = 0
        self._drop(VALID_READY_MASK, "disconnected; Align/Start required after reconnect")

    def _fresh(self, now: int) -> bool:
        return self.tracking is not None and 0 <= now - self.tracking.bridge_monotonic_ns <= self.FRESH_NS

    def _check_age(self, now: int) -> None:
        if not self._fresh(now):
            self.input_mask = 0
            self._drop(VALID_READY_MASK, "waiting/stale input; Align/Start required")

    def accept_frame(self, frame: Any, *, received_ns: int | None = None) -> bool:
        now = int(self._clock_ns())
        epoch = self.normalizer.epoch
        packet = self.normalizer.accept_frame(frame, received_ns=received_ns)
        if packet is None:
            if self.normalizer.last_error != "duplicate":
                self.input_mask = 0
                self._drop(VALID_READY_MASK, "invalid raw frame: " + self.normalizer.last_error)
            self._check_age(now)
            return False
        tracking = decode_tracking(packet)
        if epoch and epoch != tracking.tracking_epoch:
            self._drop(VALID_READY_MASK, "device clock rollback; Align/Start required")
        self.tracking = tracking
        if not self._fresh(now):
            self._check_age(now)
            return False
        from .pico_hands import PicoHandFrame

        result = self.hands.retarget(PicoHandFrame(
            tracking.left_hand, tracking.right_hand, tracking.left_active, tracking.right_active,
            tracking.tracking_epoch, tracking.sequence, tracking.source_timestamp_ns,
            tracking.left_scale, tracking.right_scale,
        ))
        self.input_mask = ((ARMS_READY if tracking.left_active and tracking.right_active else 0)
                           | (LEFT_HAND_READY if result.left_valid else 0)
                           | (RIGHT_HAND_READY if result.right_valid else 0))
        lost = (self.ready_mask | self.running_mask) & ~self.input_mask
        if lost or (self._aligning and not self.input_mask & ARMS_READY):
            self._drop(lost | (ARMS_READY if self._aligning else 0), "tracking/solver invalid; lost groups need Align/Start")
        goal = np.concatenate((self.arm.left_q, self.arm.right_q, result.left_qpos, result.right_qpos))[self._perm]
        for bit, slots in _GROUPS[1:]:
            if self.input_mask & bit:
                if np.all(np.isfinite(goal[slots])):
                    self._hand_goal[slots] = np.clip(goal[slots], self.limits[slots, 0], self.limits[slots, 1])
                else:
                    self.input_mask &= ~bit
                    self._drop(bit, "nonfinite hand result")
        self.arm.accept_tracking(tracking)
        if self._aligning:
            outputs = [alignment.accept(hand[1], True, tracking.tracking_epoch,
                                         tracking.bridge_monotonic_ns, now_ns=now)
                       for alignment, hand in ((self.arm.left_alignment, tracking.left_hand),
                                               (self.arm.right_alignment, tracking.right_hand))]
            if all(output.valid for output in outputs):
                self.ready_mask |= ARMS_READY
                self._aligning = False
                self.reason = "aligned and holding; SPD authorization required before Start"
        return True

    def command(self, command: str, *, now_ns: int | None = None) -> bool:
        now = int(self._clock_ns() if now_ns is None else now_ns)
        self._check_age(now)
        command = command.strip().lower()
        if command == "hold":
            self._drop(VALID_READY_MASK, "operator Hold; Align and SPD authorization required")
            return True
        if command == "start":
            if not self.ready_mask:
                self.reason = "Start refused: fresh input and Align required"
                return False
            self.running_mask = self.ready_mask
            self.reason = "running; Hold stops; SPD authorization required"
            return True
        if command != "align":
            return False
        self._drop(VALID_READY_MASK, "aligning; keep wrists steady")
        if not self.input_mask:
            self.reason = "Align refused: no fresh valid input"
            return False
        # A calibration is a new authorization boundary: the receiver must
        # explicitly enable this session even if it was enabled before loss.
        self.session_id = uuid.uuid4().hex
        self._sequence = 0
        self.hands.reset_filter()
        self.ready_mask = self.input_mask & (LEFT_HAND_READY | RIGHT_HAND_READY)
        if self.input_mask & ARMS_READY:
            import mujoco

            for solver, alignment, q in ((self.arm.left_solver, self.arm.left_alignment, self.arm.left_q),
                                         (self.arm.right_solver, self.arm.right_alignment, self.arm.right_q)):
                solver.data.qpos[solver.qpos_indices] = q
                mujoco.mj_forward(solver.model, solver.data)
                neutral = np.eye(4)
                neutral[:3, 3] = solver.data.site_xpos[solver.site_id]
                neutral[:3, :3] = solver.data.site_xmat[solver.site_id].reshape(3, 3)
                alignment.reset()
                alignment.neutral_robot = neutral
                solver.reset()
            self._aligning = True
        return True

    def tick(self, now_ns: int | None = None) -> None:
        now = int(self._clock_ns() if now_ns is None else now_ns)
        self._check_age(now)
        if self._last_tick_ns is not None and now < self._last_tick_ns:
            self._drop(VALID_READY_MASK, "host monotonic rollback; Align/Start required")
        dt = min(self.PERIOD_NS, max(0, now - self._last_tick_ns)) * 1e-9 if self._last_tick_ns is not None else 0.0
        self._last_tick_ns = now
        if self.running_mask & ARMS_READY:
            old_left, old_right = self.arm.left_q.copy(), self.arm.right_q.copy()
            target = self.arm.tick(now)
            goal = np.concatenate((target.left_q, target.right_q))[self._perm[:14]]
            if int(target.valid_mask) != 3 or not np.all(np.isfinite(goal)):
                self.arm.left_q, self.arm.right_q = old_left, old_right
                reasons = (f"left={target.left_hold_reason.name.lower()}, "
                           f"right={target.right_hold_reason.name.lower()}")
                self._drop(ARMS_READY, "either wrist/IK invalid (" + reasons
                           + "); both arms held; Align/Start required")
            else:
                delta = np.clip(goal - self.position[:14], -self.rates[:14] * dt, self.rates[:14] * dt)
                self.position[:14] = np.clip(self.position[:14] + delta, self.limits[:14, 0], self.limits[:14, 1])
                ordered = self.position[:14][np.argsort(self._perm[:14])]
                self.arm.left_q, self.arm.right_q = ordered[:7].copy(), ordered[7:].copy()
        for bit, slots in _GROUPS[1:]:
            if self.running_mask & bit:
                step = self.rates[slots] * dt
                self.position[slots] += np.clip(self._hand_goal[slots] - self.position[slots], -step, step)

    def snapshot(self, *, stamp_ns: int | None = None, now_ns: int | None = None) -> JointCommandSnapshot:
        self._check_age(int(self._clock_ns() if now_ns is None else now_ns))
        self._sequence += 1
        return JointCommandSnapshot.from_values(
            session_id=self.session_id, sequence=self._sequence, ready_mask=self.ready_mask,
            position_rad=self.position, stamp_ns=time.time_ns() if stamp_ns is None else stamp_ns,
        )

    def status(self, now_ns: int | None = None) -> dict[str, Any]:
        now = int(self._clock_ns() if now_ns is None else now_ns)
        self._check_age(now)
        return {"fresh": self._fresh(now), "input_mask": self.input_mask,
                "ready_mask": self.ready_mask, "running_mask": self.running_mask,
                "state": "running" if self.running_mask else ("aligning" if self._aligning else "hold"),
                "reason": self.reason, "epoch": self.normalizer.epoch}


__all__ = ["PicoRosSourceCore", "PicoTeleopCore"]
