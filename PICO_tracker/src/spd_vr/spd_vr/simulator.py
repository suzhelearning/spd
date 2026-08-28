"""Single-plant 480 Hz MuJoCo simulator with immutable input snapshots."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
import socket
from pathlib import Path
import queue
import threading
import time
from typing import TYPE_CHECKING, Any, Mapping

import numpy as np

from .arm_target_protocol import (
    ArmTargetFrame,
    ArmTargetHoldReason,
    ArmTargetStreamDecoder,
    LEFT_VALID,
    RIGHT_VALID,
    decode_packet,
)
from .camera import CameraError, CameraFrame
from .manifest import ManifestError, ManifestJoint, load_manifest, resolve_model_addresses

if TYPE_CHECKING:
    from .pico_hands import PicoHandFrame
PHYSICS_HZ = 480
ARM_TARGET_HZ = 200
HAND_TARGET_HZ = 60
CAMERA_HZ = 30
TIMESTEP_NS = 1_000_000_000 / PHYSICS_HZ
INPUT_STALE_NS = 50_000_000

@dataclass(frozen=True)
class ArmSnapshot:
    tracking_epoch: int
    sequence_id: int
    left_q: tuple[float, ...]
    right_q: tuple[float, ...]
    left_qdot: tuple[float, ...]
    right_qdot: tuple[float, ...]
    valid_mask: int
    left_hold_reason: ArmTargetHoldReason
    right_hold_reason: ArmTargetHoldReason
    source_timestamp_ns: int
    control_timestamp_ns: int

@dataclass(frozen=True)
class HandSnapshot:
    tracking_epoch: int
    sequence_id: int
    left_q: tuple[float, ...]
    right_q: tuple[float, ...]
    left_valid: bool
    right_valid: bool
    left_hold_reason: str
    right_hold_reason: str


@dataclass(frozen=True)
class CameraRequest:
    sim_time_ns: int
    qpos: np.ndarray
    qvel: np.ndarray


@dataclass(frozen=True)
class SimulationStep:
    tick: int
    sim_time_ns: int
    arm_valid_mask: int
    hand_left_valid: bool
    hand_right_valid: bool
    camera_enqueued: bool


class _CameraWorker:
    def __init__(self, provider: Any, model: Any, output: queue.Queue[CameraRequest]) -> None:
        self.provider = provider
        self.model = model
        self.output = output
        self.results: queue.Queue[Mapping[str, CameraFrame]] = queue.Queue(maxsize=4096)
        self.errors: queue.Queue[BaseException] = queue.Queue(maxsize=1)
        self._stop = threading.Event()
        self.thread = threading.Thread(target=self._run, name="spd-vr-camera", daemon=True)
        self.thread.start()

    def _run(self) -> None:
        try:
            while not self._stop.is_set():
                try:
                    request = self.output.get(timeout=0.05)
                except queue.Empty:
                    continue
                if hasattr(self.provider, "capture_snapshot"):
                    frames = self.provider.capture_snapshot(
                        request.sim_time_ns, request.qpos, request.qvel
                    )
                else:
                    frames = self.provider.capture(request.sim_time_ns)
                try:
                    self.results.put_nowait(frames)
                except queue.Full:
                    # Keep the latest completed render; physics never waits for it.
                    try:
                        self.results.get_nowait()
                    except queue.Empty:
                        pass
                    self.results.put_nowait(frames)
        except BaseException as exc:  # surfaced by poll_error/close
            try:
                self.errors.put_nowait(exc)
            except queue.Full:
                pass

    def poll_error(self) -> BaseException | None:
        try:
            return self.errors.get_nowait()
        except queue.Empty:
            return None

    def stop(self) -> None:
        self._stop.set()
        self.thread.join(timeout=2.0)


class UnifiedSimulator:
    """Own exactly one ``MjModel``/``MjData`` and apply snapshots at tick edges."""

    physics_hz = PHYSICS_HZ
    arm_target_hz = ARM_TARGET_HZ
    hand_target_hz = HAND_TARGET_HZ
    camera_hz = CAMERA_HZ

    def __init__(
        self,
        model_path: str | Path | None = None,
        manifest_path: str | Path | None = None,
        *,
        camera_provider: Any | None = None,
        recorder: Any | None = None,
        hand_retargeter: Any | None = None,
        model: Any | None = None,
        data: Any | None = None,
    ) -> None:
        try:
            import mujoco
        except ImportError as exc:  # pragma: no cover
            raise ImportError("mujoco is required for UnifiedSimulator") from exc
        self._mujoco = mujoco
        module_dir = Path(__file__).resolve().parents[1]
        model_path = Path(model_path) if model_path is not None else module_dir / "generated/tianji_wuji2_spd.xml"
        manifest_path = Path(manifest_path) if manifest_path is not None else module_dir / "generated/joint_manifest.yaml"
        self.manifest_document = load_manifest(manifest_path)
        if model is None:
            model = mujoco.MjModel.from_xml_path(str(model_path))
        self.model = model
        self.joints: list[ManifestJoint] = resolve_model_addresses(
            model,
            self.manifest_document,
            allow_scene_dofs=(model.nq > 54 or model.nv > 54),
        )
        if data is None:
            data = mujoco.MjData(model)
        self.data = data
        self._actuator_ids = {
            joint.actuator: int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, joint.actuator))
            for joint in self.joints
        }
        if any(value < 0 for value in self._actuator_ids.values()):
            raise ManifestError("manifest actuator address resolution failed")
        self._home = np.asarray([(entry.range[0] + entry.range[1]) * 0.5 for entry in self.joints], dtype=np.float64)
        self._qpos_by_index = {entry.index: entry.qpos_address for entry in self.joints}
        self._set_home_state()

        self._arm_lock = threading.Lock()
        self._hand_lock = threading.Lock()
        self._arm_snapshot = self._initial_arm_snapshot()
        self._hand_snapshot = self._initial_hand_snapshot()
        self._applied_arm = self._arm_snapshot
        self._applied_hand = self._hand_snapshot
        self._last_arm_epoch: int | None = None
        self._last_arm_sequence: int | None = None
        self._last_hand_epoch: int | None = None
        self._last_hand_sequence: int | None = None
        self._arm_last_arrival_ns: dict[str, int | None] = {"left": None, "right": None}
        self._hand_last_arrival_ns: dict[str, int | None] = {"left": None, "right": None}
        self._arm_callback_count = 0
        self._hand_callback_count = 0
        self.paused = False
        self._resume_gate_mask = 0
        self._arm_packet_decoder = ArmTargetStreamDecoder()
        self._hand_retargeter = hand_retargeter
        self._camera_provider = camera_provider
        self._recorder = recorder
        self._camera_queue: queue.Queue[CameraRequest] | None = None
        self._camera_worker: _CameraWorker | None = None
        self._camera_drop_count = 0
        if camera_provider is not None:
            self._camera_queue = queue.Queue(maxsize=4096)
            self._camera_worker = _CameraWorker(camera_provider, model, self._camera_queue)
        self._recorder_queue: queue.Queue[dict[str, Any]] | None = None
        self._recorder_thread: threading.Thread | None = None
        self._recorder_stop: threading.Event | None = None
        if recorder is not None:
            self._start_recorder_worker(recorder)
        self.tick = 0
        self._next_arm_time_ns = 0
        self._next_hand_time_ns = 0
        self._next_camera_time_ns = 0
        self._last_camera_frames: Mapping[str, CameraFrame] | None = None
        self._closed = False
        self._arm_udp_socket: socket.socket | None = None
        self._arm_udp_thread: threading.Thread | None = None
        self._arm_udp_stop: threading.Event | None = None
        self._task_object_body_ids: set[int] = set()

    def _set_home_state(self) -> None:
        self.data.qpos[:] = 0.0
        self.data.qvel[:] = 0.0
        for entry in self.joints:
            self.data.qpos[entry.qpos_address] = (entry.range[0] + entry.range[1]) * 0.5
        self._mujoco.mj_forward(self.model, self.data)

    def reset_scene(self, scene_result: Any) -> None:
        """Reset robot and movable task objects to one deterministic scene."""
        self._set_home_state()
        for obj in scene_result.objects:
            joint_id = int(self._mujoco.mj_name2id(
                self.model,
                self._mujoco.mjtObj.mjOBJ_JOINT,
                f"{obj.name}_free",
            ))
            if joint_id < 0:
                raise ManifestError(f"scene object has no free joint: {obj.name}")
            qpos_address = int(self.model.jnt_qposadr[joint_id])
            yaw = float(obj.yaw_rad) * 0.5
            self.data.qpos[qpos_address:qpos_address + 7] = (
                *np.asarray(obj.position, dtype=np.float64),
                math.cos(yaw),
                0.0,
                0.0,
                math.sin(yaw),
            )
        self.data.qvel[:] = 0.0
        if getattr(self.data, "act", None) is not None:
            self.data.act[:] = 0.0
        self.data.ctrl[:] = 0.0
        self.data.time = 0.0
        self.tick = 0
        self._mujoco.mj_forward(self.model, self.data)
        self.invalidate_snapshots()

    def _reset_camera_stream(self) -> None:
        self._last_camera_frames = None
        if self._camera_queue is not None:
            while True:
                try:
                    self._camera_queue.get_nowait()
                except queue.Empty:
                    break
        if self._camera_worker is not None:
            while True:
                try:
                    self._camera_worker.results.get_nowait()
                except queue.Empty:
                    break
        reset_camera = getattr(self._camera_provider, "reset", None)
        if reset_camera is not None:
            reset_camera()

    def _initial_arm_snapshot(self) -> ArmSnapshot:
        left_q = tuple(float(value) for value in self._home[:7])
        left_qdot = (0.0,) * 7
        right_q = tuple(float(value) for value in self._home[27:34])
        right_qdot = (0.0,) * 7
        return ArmSnapshot(
            tracking_epoch=0,
            sequence_id=0,
            left_q=left_q,
            right_q=right_q,
            left_qdot=left_qdot,
            right_qdot=right_qdot,
            valid_mask=0,
            left_hold_reason=ArmTargetHoldReason.INPUT_STALE,
            right_hold_reason=ArmTargetHoldReason.INPUT_STALE,
            source_timestamp_ns=0,
            control_timestamp_ns=0,
        )

    def _initial_hand_snapshot(self) -> HandSnapshot:
        return HandSnapshot(
            tracking_epoch=0,
            sequence_id=0,
            left_q=tuple(float(value) for value in self._home[7:27]),
            right_q=tuple(float(value) for value in self._home[34:54]),
            left_valid=False,
            right_valid=False,
            left_hold_reason="inactive",
            right_hold_reason="inactive",
        )

    @staticmethod
    def _vector(values: Any, expected: int, name: str) -> tuple[float, ...]:
        value = tuple(float(item) for item in values)
        if len(value) != expected or not all(math.isfinite(item) for item in value):
            raise ValueError(f"{name} must be finite and have length {expected}")
        return value

    def _side_range_ok(self, side: str, group: str, values: tuple[float, ...], velocities: tuple[float, ...] | None = None) -> bool:
        entries = [entry for entry in self.joints if entry.side == side and entry.group == group]
        if len(entries) != len(values):
            return False
        for entry, value in zip(entries, values):
            if not (entry.range[0] <= value <= entry.range[1]):
                return False
            if velocities is not None and entry.velocity_limit is not None:
                if abs(velocities[entry.index - (0 if side == "left" and group == "arm" else 27)]) > entry.velocity_limit:
                    return False
        return True

    def _hold_arm_snapshot(
        self,
        left_reason: ArmTargetHoldReason,
        right_reason: ArmTargetHoldReason | None = None,
        epoch: int | None = None,
        sequence: int | None = None,
    ) -> None:
        if right_reason is None:
            right_reason = left_reason
        with self._arm_lock:
            previous = self._arm_snapshot
            self._arm_snapshot = ArmSnapshot(
                tracking_epoch=previous.tracking_epoch if epoch is None else epoch,
                sequence_id=previous.sequence_id if sequence is None else sequence,
                left_q=previous.left_q,
                right_q=previous.right_q,
                left_qdot=previous.left_qdot,
                right_qdot=previous.right_qdot,
                valid_mask=0,
                left_hold_reason=left_reason,
                right_hold_reason=right_reason,
                source_timestamp_ns=previous.source_timestamp_ns,
                control_timestamp_ns=previous.control_timestamp_ns,
            )
    def start_arm_udp(self, host: str = "127.0.0.1", port: int = 15100) -> int:
        if self._arm_udp_thread is not None:
            raise RuntimeError("arm UDP receiver is already running")
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((host, int(port)))
        sock.settimeout(0.05)
        self._arm_udp_socket = sock
        self._arm_udp_stop = threading.Event()
        stop = self._arm_udp_stop
        def receive() -> None:
            while not stop.is_set():
                try:
                    packet, _ = sock.recvfrom(4096)
                except socket.timeout:
                    continue
                except OSError:
                    break
                try:
                    self.on_arm_target_packet(packet, now_ns=time.monotonic_ns())
                except ValueError:
                    # Structural packet errors are a HOLD event, not a reason
                    # for the receiver thread to terminate.
                    if not self.paused:
                        self._hold_arm_snapshot(ArmTargetHoldReason.SOLVER_FAILURE)

        self._arm_udp_thread = threading.Thread(target=receive, name="spd-vr-arm-udp", daemon=True)
        self._arm_udp_thread.start()
        return int(sock.getsockname()[1])

    def stop_arm_udp(self) -> None:
        if self._arm_udp_stop is not None:
            self._arm_udp_stop.set()
        if self._arm_udp_socket is not None:
            self._arm_udp_socket.close()
        if self._arm_udp_thread is not None:
            self._arm_udp_thread.join(timeout=1.0)
        self._arm_udp_socket = None
        self._arm_udp_thread = None
        self._arm_udp_stop = None

    def on_arm_target_packet(self, packet: bytes, *, now_ns: int | None = None) -> ArmSnapshot:
        if self.paused:
            self._arm_callback_count += 1
            with self._arm_lock:
                return self._arm_snapshot
        frame = self._arm_packet_decoder.decode(packet)
        return self.on_arm_target(frame, now_ns=now_ns)

    def on_arm_target(self, frame: ArmTargetFrame | Mapping[str, Any], *, now_ns: int | None = None) -> ArmSnapshot:
        self._arm_callback_count += 1
        if self.paused:
            with self._arm_lock:
                return self._arm_snapshot
        if not isinstance(frame, ArmTargetFrame):
            frame = decode_packet(bytes(frame), now_ns=now_ns)
        with self._arm_lock:
            previous = self._arm_snapshot
            if self._last_arm_epoch is not None and (
                frame.tracking_epoch < self._last_arm_epoch
                or (
                    frame.tracking_epoch == self._last_arm_epoch
                    and self._last_arm_sequence is not None
                    and frame.sequence <= self._last_arm_sequence
                )
            ):
                return previous
            left_q, right_q = previous.left_q, previous.right_q
            left_qdot, right_qdot = previous.left_qdot, previous.right_qdot
            valid_mask = 0
            left_reason = (
                frame.left_hold_reason
                if frame.left_hold_reason is not ArmTargetHoldReason.NONE
                else ArmTargetHoldReason.INPUT_STALE
            )
            right_reason = (
                frame.right_hold_reason
                if frame.right_hold_reason is not ArmTargetHoldReason.NONE
                else ArmTargetHoldReason.INPUT_STALE
            )
            arrival_ns = int(time.monotonic_ns() if now_ns is None else now_ns)
            if frame.valid_mask & LEFT_VALID:
                try:
                    candidate = self._vector(frame.left_q, 7, "left_q")
                    candidate_dot = self._vector(frame.left_qdot, 7, "left_qdot")
                    if self._side_range_ok("left", "arm", candidate, candidate_dot):
                        left_q, left_qdot = candidate, candidate_dot
                        valid_mask |= LEFT_VALID
                        left_reason = ArmTargetHoldReason.NONE
                        self._arm_last_arrival_ns["left"] = arrival_ns
                    else:
                        left_reason = ArmTargetHoldReason.SOLVER_FAILURE
                except ValueError:
                    left_reason = ArmTargetHoldReason.SOLVER_FAILURE
            if frame.valid_mask & RIGHT_VALID:
                try:
                    candidate = self._vector(frame.right_q, 7, "right_q")
                    candidate_dot = self._vector(frame.right_qdot, 7, "right_qdot")
                    if self._side_range_ok("right", "arm", candidate, candidate_dot):
                        right_q, right_qdot = candidate, candidate_dot
                        valid_mask |= RIGHT_VALID
                        right_reason = ArmTargetHoldReason.NONE
                        self._arm_last_arrival_ns["right"] = arrival_ns
                    else:
                        right_reason = ArmTargetHoldReason.SOLVER_FAILURE
                except ValueError:
                    right_reason = ArmTargetHoldReason.SOLVER_FAILURE
            if valid_mask & LEFT_VALID:
                self._resume_gate_mask &= ~LEFT_VALID
            if valid_mask & RIGHT_VALID:
                self._resume_gate_mask &= ~RIGHT_VALID
            self._arm_snapshot = ArmSnapshot(
                tracking_epoch=frame.tracking_epoch,
                sequence_id=frame.sequence,
                left_q=left_q,
                right_q=right_q,
                left_qdot=left_qdot,
                right_qdot=right_qdot,
                valid_mask=valid_mask,
                left_hold_reason=left_reason,
                right_hold_reason=right_reason,
                source_timestamp_ns=frame.source_timestamp_ns,
                control_timestamp_ns=frame.control_timestamp_ns,
            )
            self._last_arm_epoch = frame.tracking_epoch
            self._last_arm_sequence = frame.sequence
            return self._arm_snapshot

    def _extract_hand_target(self, result: Any) -> tuple[tuple[float, ...], tuple[float, ...], bool, bool, str, str]:
        left = self._vector(result.left_qpos, 20, "left_qpos")
        right = self._vector(result.right_qpos, 20, "right_qpos")
        return (
            left,
            right,
            bool(getattr(result, "left_valid", True)),
            bool(getattr(result, "right_valid", True)),
            str(getattr(result, "left_hold_reason", "none")),
            str(getattr(result, "right_hold_reason", "none")),
        )

    def on_pico_hands(
        self,
        frame: PicoHandFrame | Mapping[str, Any] | Any,
        *,
        now_ns: int | None = None,
    ) -> HandSnapshot:
        self._hand_callback_count += 1
        if self.paused:
            with self._hand_lock:
                return self._hand_snapshot
        if isinstance(frame, Mapping):
            from .pico_hands import PicoHandFrame
            frame = PicoHandFrame(
                left_hand=np.asarray(frame["left_hand"]),
                right_hand=np.asarray(frame["right_hand"]),
                left_active=bool(frame.get("left_active", True)),
                right_active=bool(frame.get("right_active", True)),
                tracking_epoch=int(frame.get("tracking_epoch", 0)),
                sequence_id=int(frame.get("sequence_id", 0)),
                timestamp_ns=int(frame.get("timestamp_ns", 0)),
                left_scale=float(frame.get("left_scale", 1.0)),
                right_scale=float(frame.get("right_scale", 1.0)),
            )
        elif hasattr(frame, "left_joints") and hasattr(frame, "right_joints"):
            from .pico_hands import PicoHandsInput
            frame = PicoHandsInput(frame).frame
        elif not all(hasattr(frame, name) for name in (
            "left_hand", "right_hand", "left_active", "right_active",
        )):
            raise TypeError("PicoHands input must expose left/right hand fields")
        with self._hand_lock:
            previous = self._hand_snapshot
            if self._last_hand_epoch is not None and (
                frame.tracking_epoch < self._last_hand_epoch
                or (
                    frame.tracking_epoch == self._last_hand_epoch
                    and self._last_hand_sequence is not None
                    and frame.sequence_id <= self._last_hand_sequence
                )
            ):
                return previous
            if self._hand_retargeter is None:
                raise RuntimeError("hand_retargeter is required for PicoHands input")
            if frame.tracking_epoch != previous.tracking_epoch:
                reset = getattr(self._hand_retargeter, "reset_filter", None)
                if reset is not None:
                    try:
                        reset(frame.tracking_epoch)
                    except TypeError:
                        reset()
            try:
                result = self._hand_retargeter.retarget(frame)
                left, right, left_valid, right_valid, left_reason, right_reason = self._extract_hand_target(result)
            except Exception:
                self._hand_snapshot = HandSnapshot(
                    frame.tracking_epoch, frame.sequence_id,
                    previous.left_q, previous.right_q, False, False,
                    "solver_failure", "solver_failure"
                )
                return self._hand_snapshot
            arrival_ns = int(time.monotonic_ns() if now_ns is None else now_ns)
            if left_valid and not self._side_range_ok("left", "hand", left):
                left_valid, left_reason = False, "invalid"
            if right_valid and not self._side_range_ok("right", "hand", right):
                right_valid, right_reason = False, "invalid"
            if self._resume_gate_mask & LEFT_VALID:
                left_valid, left_reason = False, "input_stale"
            elif left_valid:
                self._hand_last_arrival_ns["left"] = arrival_ns
            if self._resume_gate_mask & RIGHT_VALID:
                right_valid, right_reason = False, "input_stale"
            elif right_valid:
                self._hand_last_arrival_ns["right"] = arrival_ns
            self._hand_snapshot = HandSnapshot(
                frame.tracking_epoch, frame.sequence_id,
                left if left_valid else previous.left_q,
                right if right_valid else previous.right_q,
                left_valid, right_valid, left_reason, right_reason,
            )
            self._last_hand_epoch = frame.tracking_epoch
            self._last_hand_sequence = frame.sequence_id
            return self._hand_snapshot

    def _refresh_input_validity(self, now_ns: int) -> None:
        if self.paused:
            return
        with self._arm_lock:
            arm = self._arm_snapshot
            mask = arm.valid_mask
            left_stale = bool(mask & LEFT_VALID) and (
                self._arm_last_arrival_ns["left"] is None
                or now_ns - int(self._arm_last_arrival_ns["left"]) > INPUT_STALE_NS
            )
            right_stale = bool(mask & RIGHT_VALID) and (
                self._arm_last_arrival_ns["right"] is None
                or now_ns - int(self._arm_last_arrival_ns["right"]) > INPUT_STALE_NS
            )
            if left_stale or right_stale:
                if left_stale:
                    mask &= ~LEFT_VALID
                if right_stale:
                    mask &= ~RIGHT_VALID
                self._arm_snapshot = ArmSnapshot(
                    **{
                        **arm.__dict__,
                        "valid_mask": mask,
                        "left_hold_reason": (
                            ArmTargetHoldReason.INPUT_STALE
                            if left_stale else arm.left_hold_reason
                        ),
                        "right_hold_reason": (
                            ArmTargetHoldReason.INPUT_STALE
                            if right_stale else arm.right_hold_reason
                        ),
                    }
                )
        with self._hand_lock:
            hand = self._hand_snapshot
            left_stale = hand.left_valid and (
                self._hand_last_arrival_ns["left"] is None
                or now_ns - int(self._hand_last_arrival_ns["left"]) > INPUT_STALE_NS
            )
            right_stale = hand.right_valid and (
                self._hand_last_arrival_ns["right"] is None
                or now_ns - int(self._hand_last_arrival_ns["right"]) > INPUT_STALE_NS
            )
            if left_stale or right_stale:
                self._hand_snapshot = HandSnapshot(
                    hand.tracking_epoch,
                    hand.sequence_id,
                    hand.left_q,
                    hand.right_q,
                    False if left_stale else hand.left_valid,
                    False if right_stale else hand.right_valid,
                    "input_stale" if left_stale else hand.left_hold_reason,
                    "input_stale" if right_stale else hand.right_hold_reason,
                )

    def set_paused(self, paused: bool) -> None:
        paused = bool(paused)
        if paused == self.paused:
            return
        if paused:
            self.paused = True
            self._resume_gate_mask = LEFT_VALID | RIGHT_VALID
            self._arm_packet_decoder.reset()
            self._last_arm_epoch = self._last_arm_sequence = None
            self._last_hand_epoch = self._last_hand_sequence = None
            self._arm_last_arrival_ns = {"left": None, "right": None}
            self._hand_last_arrival_ns = {"left": None, "right": None}
            reset = getattr(self._hand_retargeter, "reset_filter", None)
            if reset is not None:
                try:
                    reset()
                except TypeError:
                    reset(0)
            with self._arm_lock:
                arm = self._arm_snapshot
                self._arm_snapshot = ArmSnapshot(
                    **{
                        **arm.__dict__,
                        "valid_mask": 0,
                        "left_hold_reason": ArmTargetHoldReason.PAUSED,
                        "right_hold_reason": ArmTargetHoldReason.PAUSED,
                    }
                )
                self._applied_arm = self._arm_snapshot
            with self._hand_lock:
                hand = self._hand_snapshot
                self._hand_snapshot = HandSnapshot(
                    hand.tracking_epoch, hand.sequence_id, hand.left_q, hand.right_q,
                    False, False, "paused", "paused"
                )
                self._applied_hand = self._hand_snapshot
        else:
            self.paused = False
            self._resume_gate_mask = LEFT_VALID | RIGHT_VALID
            with self._arm_lock:
                arm = self._arm_snapshot
                self._arm_snapshot = ArmSnapshot(
                    **{
                        **arm.__dict__,
                        "valid_mask": 0,
                        "left_hold_reason": ArmTargetHoldReason.INPUT_STALE,
                        "right_hold_reason": ArmTargetHoldReason.INPUT_STALE,
                    }
                )
                self._applied_arm = self._arm_snapshot
            with self._hand_lock:
                hand = self._hand_snapshot
                self._hand_snapshot = HandSnapshot(
                    hand.tracking_epoch, hand.sequence_id, hand.left_q, hand.right_q,
                    False, False, "input_stale", "input_stale"
                )
                self._applied_hand = self._hand_snapshot

    def _schedule_due(self, sim_time_ns: int, next_time_ns: int, period_ns: int) -> tuple[bool, int]:
        if sim_time_ns < next_time_ns:
            return False, next_time_ns
        while next_time_ns <= sim_time_ns:
            next_time_ns += period_ns
        return True, next_time_ns

    def _apply_targets(self) -> None:
        arm = self._applied_arm
        hand = self._applied_hand
        for entry in self.joints:
            if entry.group == "arm":
                side_index = entry.index if entry.side == "left" else entry.index - 27
                value = (arm.left_q if entry.side == "left" else arm.right_q)[side_index]
            else:
                side_index = entry.index - (7 if entry.side == "left" else 34)
                value = (hand.left_q if entry.side == "left" else hand.right_q)[side_index]
            self.data.ctrl[self._actuator_ids[entry.actuator]] = value

    def _enqueue_camera(self, sim_time_ns: int) -> bool:
        if self._camera_queue is None:
            return False
        request = CameraRequest(
            sim_time_ns=sim_time_ns,
            qpos=np.asarray(self.data.qpos, dtype=np.float64).copy(),
            qvel=np.asarray(self.data.qvel, dtype=np.float64).copy(),
        )
        try:
            self._camera_queue.put_nowait(request)
            return True
        except queue.Full:
            self._camera_drop_count += 1
            return False

    def _start_recorder_worker(self, recorder: Any) -> None:
        self._recorder_queue = queue.Queue(maxsize=8)
        self._recorder_stop = threading.Event()
        queue_ref = self._recorder_queue
        stop_ref = self._recorder_stop

        def worker() -> None:
            while not stop_ref.is_set() or not queue_ref.empty():
                try:
                    item = queue_ref.get(timeout=0.05)
                except queue.Empty:
                    continue
                recorder.submit(**item)

        self._recorder_thread = threading.Thread(target=worker, name="spd-vr-recorder", daemon=True)
        self._recorder_thread.start()
    def _manifest_qpos(self) -> np.ndarray:
        return np.asarray([self.data.qpos[entry.qpos_address] for entry in self.joints], dtype=np.float64)

    def _manifest_qvel(self) -> np.ndarray:
        return np.asarray([self.data.qvel[entry.dof_address] for entry in self.joints], dtype=np.float64)

    def _manifest_ctrl(self) -> np.ndarray:
        return np.asarray([self.data.ctrl[self._actuator_ids[entry.actuator]] for entry in self.joints], dtype=np.float64)


    def step(self) -> SimulationStep:
        if self._closed:
            raise RuntimeError("simulator is closed")
        if self.paused:
            return SimulationStep(
                tick=self.tick,
                sim_time_ns=self.sim_time_ns,
                arm_valid_mask=self._applied_arm.valid_mask,
                hand_left_valid=self._applied_hand.left_valid,
                hand_right_valid=self._applied_hand.right_valid,
                camera_enqueued=False,
            )
        self.tick += 1
        sim_time_ns = int(round(self.tick * TIMESTEP_NS))
        self._refresh_input_validity(time.monotonic_ns())

        with self._arm_lock:
            arm_snapshot = self._arm_snapshot
        with self._hand_lock:
            hand_snapshot = self._hand_snapshot
        arm_due, self._next_arm_time_ns = self._schedule_due(sim_time_ns, self._next_arm_time_ns, int(1e9 / ARM_TARGET_HZ))
        hand_due, self._next_hand_time_ns = self._schedule_due(sim_time_ns, self._next_hand_time_ns, int(1e9 / HAND_TARGET_HZ))
        camera_due, self._next_camera_time_ns = self._schedule_due(sim_time_ns, self._next_camera_time_ns, int(1e9 / CAMERA_HZ))
        if arm_due:
            self._applied_arm = arm_snapshot
        if hand_due:
            self._applied_hand = hand_snapshot
        self._apply_targets()
        step_start = time.perf_counter_ns()
        self._mujoco.mj_step(self.model, self.data)
        step_duration_ns = time.perf_counter_ns() - step_start
        camera_enqueued = self._enqueue_camera(sim_time_ns) if camera_due else False
        if self._recorder_queue is not None and (self.tick % 8 == 0):
            item = {
                "sim_time_ns": sim_time_ns,
                "qpos": self._manifest_qpos(),
                "qvel": self._manifest_qvel(),
                "qpos_target": self._manifest_ctrl(),
                "step_duration_ns": step_duration_ns,
                "arm_valid_mask": self._applied_arm.valid_mask,
                "hand_valid_mask": (
                    (1 if self._applied_hand.left_valid else 0)
                    | (2 if self._applied_hand.right_valid else 0)
                ),
            }
            try:
                self._recorder_queue.put_nowait(item)
            except queue.Full:
                pass
        if self._camera_worker is not None:
            error = self._camera_worker.poll_error()
            if error is not None:
                raise CameraError(str(error)) from error
        return SimulationStep(
            tick=self.tick,
            sim_time_ns=sim_time_ns,
            arm_valid_mask=self._applied_arm.valid_mask,
            hand_left_valid=self._applied_hand.left_valid,
            hand_right_valid=self._applied_hand.right_valid,
            camera_enqueued=camera_enqueued,
        )

    def drain_camera_results(self) -> list[Mapping[str, CameraFrame]]:
        if self._camera_worker is None:
            return []
        results: list[Mapping[str, CameraFrame]] = []
        while True:
            try:
                frames = self._camera_worker.results.get_nowait()
            except queue.Empty:
                break
            self._last_camera_frames = frames
            results.append(frames)
        return results

    def drain_camera_frames(self) -> Mapping[str, CameraFrame] | None:
        results = self.drain_camera_results()
        return results[-1] if results else self._last_camera_frames

    @property
    def arm_callback_count(self) -> int:
        return self._arm_callback_count

    @property
    def hand_callback_count(self) -> int:
        return self._hand_callback_count

    @property
    def camera_drop_count(self) -> int:
        return self._camera_drop_count

    @property
    def sim_time_ns(self) -> int:
        return int(round(self.tick * TIMESTEP_NS))

    def set_task_object_body_names(self, names: set[str]) -> None:
        self._task_object_body_ids = {
            int(self._mujoco.mj_name2id(self.model, self._mujoco.mjtObj.mjOBJ_BODY, name))
            for name in names
        }
        self._task_object_body_ids.discard(-1)

    def has_task_object_contact(self) -> bool:
        for index in range(int(self.data.ncon)):
            contact = self.data.contact[index]
            first = int(self.model.geom_bodyid[contact.geom1])
            second = int(self.model.geom_bodyid[contact.geom2])
            if first not in self._task_object_body_ids and second not in self._task_object_body_ids:
                continue
            other = second if first in self._task_object_body_ids else first
            name = self._mujoco.mj_id2name(
                self.model, self._mujoco.mjtObj.mjOBJ_BODY, other
            ) or ""
            if name.startswith(("l_", "r_")):
                return True
        return False

    def invalidate_snapshots(self) -> None:
        with self._arm_lock:
            self._arm_snapshot = self._initial_arm_snapshot()
            self._applied_arm = self._arm_snapshot
        with self._hand_lock:
            self._hand_snapshot = self._initial_hand_snapshot()
            self._applied_hand = self._hand_snapshot
        self._last_arm_epoch = None
        self._last_arm_sequence = None
        self._last_hand_epoch = None
        self._last_hand_sequence = None
        self._arm_last_arrival_ns = {"left": None, "right": None}
        self._hand_last_arrival_ns = {"left": None, "right": None}
        self._resume_gate_mask = 0
        self._arm_packet_decoder.reset()
        self._reset_camera_stream()
        self._next_arm_time_ns = self._next_hand_time_ns = self._next_camera_time_ns = self.sim_time_ns


    def benchmark(self, duration_s: float) -> dict[str, float | int]:
        if not math.isfinite(duration_s) or duration_s <= 0.0:
            raise ValueError("duration_s must be positive")
        durations: list[int] = []
        ticks = int(round(duration_s * PHYSICS_HZ))
        for _ in range(ticks):
            before = time.perf_counter_ns()
            self.step()
            durations.append(time.perf_counter_ns() - before)
        values = np.asarray(durations, dtype=np.float64) / 1e6
        return {
            "duration_s": duration_s,
            "ticks": ticks,
            "physics_hz": PHYSICS_HZ,
            "step_p50_ms": float(np.percentile(values, 50)),
            "step_p95_ms": float(np.percentile(values, 95)),
            "step_p99_ms": float(np.percentile(values, 99)),
            "step_max_ms": float(np.max(values)),
            "camera_drops": self._camera_drop_count,
        }

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self.stop_arm_udp()
        if self._camera_worker is not None:
            self._camera_worker.stop()
        if self._recorder_stop is not None:
            self._recorder_stop.set()
        if self._recorder_thread is not None:
            self._recorder_thread.join(timeout=2.0)

    def __enter__(self) -> "UnifiedSimulator":
        return self

    def __exit__(self, *_: Any) -> None:
        self.close()


def benchmark_main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--model", type=Path, default=None)
    parser.add_argument("--manifest", type=Path, default=None)
    args = parser.parse_args(argv)
    with UnifiedSimulator(args.model, args.manifest) as simulator:
        print(json.dumps(simulator.benchmark(args.duration), sort_keys=True))
    return 0


__all__ = [
    "ArmSnapshot",
    "HandSnapshot",
    "SimulationStep",
    "UnifiedSimulator",
    "benchmark_main",
]
