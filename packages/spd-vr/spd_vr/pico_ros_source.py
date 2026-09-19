"""Single-owner PICO normalization, calibration and bounded ROS command source.

Receiver callbacks must only enqueue frames with their monotonic receipt time.
All methods here (including connection events) run on the solver thread.
"""
from __future__ import annotations

from collections.abc import Callable
from dataclasses import replace
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
from .pico_hands import PICO_TO_MEDIAPIPE
from .alignment import _pose_matrix, _rotation_distance

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
        if not hand.valid or len(joints) != 26:
            raise ValueError("inactive or incomplete hand")
        poses = []
        for i, joint in enumerate(joints):
            try:
                if not joint.valid:
                    raise ValueError("invalid joint")
                poses.append(_pose_values(joint, f"{side}.joint[{i}]"))
            except (AttributeError, TypeError, ValueError, OverflowError):
                if i in PICO_TO_MEDIAPIPE:
                    raise
                poses.append(_IDENTITY_POSE)
        return True, tuple(poses)
    except (AttributeError, TypeError, ValueError, OverflowError):
        return False, (_IDENTITY_POSE,) * 26


def _arm_hand(hand: Any, canonical: Any, hand_active: bool) -> tuple[bool, Any]:
    """A lost fingertip must not invalidate a still-tracked wrist."""
    if hand_active:
        return True, canonical
    try:
        if not hand.valid or len(hand.joints) != 26 or not hand.joints[1].valid:
            return False, canonical
        wrist = _pose_values(hand.joints[1], "wrist")
    except (AttributeError, TypeError, ValueError, OverflowError):
        return False, canonical
    points = np.array(canonical, copy=True)
    points[1] = wrist
    return True, points


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
    FEEDBACK_FRESH_NS = 250_000_000

    @classmethod
    def from_production(cls) -> "PicoTeleopCore":
        from .arm_ik import _production_controller, _verified_model
        from .model_builder import workspace_root
        from .retarget_pair import WujiRetargetPair
        from .palm_mapping import PalmMapping

        package = Path(__file__).resolve().parents[1]
        manifest_path = package / "generated/model_manifest.yaml"
        asset_root = workspace_root() / "assets/tianji_wuji2"
        urdf = asset_root / "tianji_wuji2.urdf"
        model, verified = _verified_model(package / "generated/arm_ik.xml", manifest_path, urdf)
        pair = WujiRetargetPair.from_manifest(
            package / "config/wuji2_pico_left.yaml", package / "config/wuji2_pico_right.yaml",
            manifest_path, urdf,
        )
        return cls(_production_controller(model, verified), pair, verified.manifest,
                   palm_mapping=PalmMapping.from_urdf(urdf), require_collision_scene=True)

    def __init__(self, arm: Any, hands: Any, manifest: dict[str, Any], *,
                 clock_ns: Callable[[], int] = time.monotonic_ns,
                 palm_mapping: Any | None = None, require_collision_scene: bool = False) -> None:
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
        # Keep robot speed bounded, with source limits inside the arm QP rather
        # than distorting its Cartesian solution by clipping joints afterwards.
        self.rates = np.minimum(velocities, 1.5)
        self.rates[14:] = np.minimum(velocities[14:], 6.0)
        arm_rates = self.rates[:14][np.argsort(self._perm[:14])]
        for solver, rates in ((arm.left_solver, arm_rates[:7]), (arm.right_solver, arm_rates[7:])):
            solver.velocity_limits = np.minimum(solver.velocity_limits, rates)
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
        self.palm_mapping = palm_mapping
        self._require_collision_scene = require_collision_scene
        self._feedback: dict[str, Any] | None = None
        self._feedback_scene: str | None = None
        self._feedback_error = "waiting for actual robot state"
        self._calibrating = False
        self._calibration_count = 0
        self._calibration_last_ns = 0
        self._calibration_start_ns = 0
        self._calibration_previous: tuple[np.ndarray, ...] | None = None
        self._mapping_reason = "K: face forward, fingers forward, palms down; keep steady"
        self._preview_data = None

    def _feedback_fresh(self, now: int) -> bool:
        return (not self._feedback_error and self._feedback is not None
                and 0 <= now - self._feedback["monotonic_ns"] <= self.FEEDBACK_FRESH_NS)

    def update_feedback(self, feedback: Any) -> bool:
        """Consume physics-thread state without treating retained commands as actual."""
        try:
            if not isinstance(feedback, dict) or tuple(feedback["joint_names"]) != JOINT_NAMES[:14]:
                raise ValueError("actual feedback joint order mismatch")
            stamp = feedback["monotonic_ns"]
            now = int(self._clock_ns())
            if type(stamp) is not int or not 0 <= now - stamp <= self.FEEDBACK_FRESH_NS:
                raise ValueError("actual feedback is stale or from the future")
            if self._feedback is not None and stamp <= self._feedback["monotonic_ns"]:
                return False
            values = {}
            for key in ("position_rad", "velocity_rad_s", "retained_position_rad"):
                values[key] = np.asarray(feedback[key], dtype=float)
                if values[key].shape != (14,) or not np.all(np.isfinite(values[key])):
                    raise ValueError("invalid actual feedback " + key)
            q = values["position_rad"]
            if np.any(q < self.limits[:14, 0] - 0.02) or np.any(q > self.limits[:14, 1] + 0.02):
                raise ValueError("actual robot outside joint limits")
            scene = feedback.get("scene_xml")
            if self._require_collision_scene and (not isinstance(scene, str) or not scene):
                raise ValueError("actual scene required for collision constraints")
            if scene is not None:
                if not isinstance(scene, str) or not Path(scene).is_file():
                    raise ValueError("actual scene XML is unavailable")
                scene = str(Path(scene).resolve())
                if scene != self._feedback_scene:
                    self.arm.configure_collision_scene(scene)
                    self._feedback_scene = scene
                    self._drop(ARMS_READY, "scene changed; position Align required")
            self.arm.update_actual_state(values["position_rad"], values["velocity_rad_s"])
            self._feedback = {"monotonic_ns": stamp, **values, "scene_xml": scene}
            self._feedback_error = ""
            return True
        except (KeyError, TypeError, ValueError, RuntimeError, OSError) as exc:
            self._feedback_error = str(exc)
            if self.palm_mapping is not None:
                self._drop(ARMS_READY, "actual feedback invalid: " + str(exc))
            return False

    def _settled_feedback(self, now: int) -> bool:
        if not self._feedback_fresh(now):
            self.reason = "fresh actual robot feedback required: " + self._feedback_error
            return False
        assert self._feedback is not None
        if (np.max(np.abs(self._feedback["velocity_rad_s"])) > 0.1
                or np.max(np.abs(self._feedback["position_rad"] - self._feedback["retained_position_rad"])) > 0.1):
            self.reason = "robot has not settled at retained target; wait before calibration/Align"
            return False
        return True

    def _reset_mapping(self) -> None:
        if self.palm_mapping is not None:
            self.palm_mapping.reset()
        self._calibrating = False
        self._calibration_previous = None
        self._calibration_count = 0
        self._mapping_reason = "tracking origin changed; K palm/direction calibration required"

    def _accept_calibration(self, frame: Any, tracking: TrackingFrame, now: int) -> None:
        if not self._calibrating or self.palm_mapping is None:
            return
        try:
            if not self._settled_feedback(now):
                raise ValueError(self.reason)
            if not frame.head.valid or not self.tracking.left_active or not self.tracking.right_active:
                raise ValueError("head and both wrists must be tracked")
            head = _pose_matrix(_pose_values(frame.head, "head"))
            forward = head[:3, 0].copy()
            forward[2] = 0.0
            if np.linalg.norm(forward) < 0.2:
                raise ValueError("face horizontally forward")
            forward /= np.linalg.norm(forward)
            wrists, palms = {}, {}
            for side in ("left", "right"):
                hand = getattr(frame, side)
                if not all(hand.joints[i].valid for i in (0, 1, 7, 12, 22)):
                    raise ValueError("palm and knuckle points must be tracked for calibration")
                wrists[side] = _pose_matrix(_pose_values(hand.joints[1], side + ".wrist"))
                palms[side] = np.asarray(_pose_values(hand.joints[0], side + ".palm")[:3])
                points = {i: np.asarray(_pose_values(hand.joints[i], side + ".knuckle")[:3]) for i in (7, 12, 22)}
                wrist = wrists[side][:3, 3]
                finger_forward = points[12] - wrist
                dorsal = np.cross(points[7] - wrist, points[22] - wrist) * (1.0 if side == "left" else -1.0)
                if (np.linalg.norm(finger_forward) < 1e-4 or np.linalg.norm(dorsal) < 1e-6
                        or float(finger_forward @ forward) / np.linalg.norm(finger_forward) < 0.7
                        or dorsal[2] / np.linalg.norm(dorsal) < 0.7):
                    raise ValueError("standard pose required: fingers forward, both palms down")
            samples = (head, wrists["left"], wrists["right"])
            if tracking.source_timestamp_ns - self._calibration_last_ns > self.FRESH_NS:
                self._calibration_previous = None
            self._calibration_last_ns = tracking.source_timestamp_ns
            if self._calibration_previous is None or any(
                    np.linalg.norm(a[:3, 3] - b[:3, 3]) > 0.01
                    or _rotation_distance(a[:3, :3], b[:3, :3]) > 0.08
                    for a, b in zip(samples, self._calibration_previous)):
                self._calibration_count = 0
                self._calibration_start_ns = tracking.source_timestamp_ns
                self._calibration_previous = samples
            self._calibration_count += 1
            self._mapping_reason = f"standard pose stable samples: {self._calibration_count}/10"
            if self._calibration_count >= 10 and tracking.source_timestamp_ns - self._calibration_start_ns >= 150_000_000:
                self.palm_mapping.calibrate(head, wrists, palms, tracking.tracking_epoch)
                for side in ("left", "right"):
                    self.palm_mapping.configure_alignment(side, getattr(self.arm, side + "_alignment"))
                self._calibrating = False
                self._mapping_reason = "palm/direction calibrated; C aligns position and previews without motion"
                self.reason = self._mapping_reason
        except (AttributeError, TypeError, ValueError, OverflowError) as exc:
            self._calibration_previous = None
            self._calibration_count = 0
            self._mapping_reason = str(exc)

    def _palm_preview(self, now: int) -> dict[str, Any]:
        if self.palm_mapping is None or not self.palm_mapping.calibrated:
            return {}
        import mujoco
        from scipy.spatial.transform import Rotation

        model = self.arm.left_solver.model
        if self._preview_data is None:
            self._preview_data = mujoco.MjData(model)
        data = self._preview_data
        diagnostics = self.arm.diagnostics()
        preview = {}
        for side, slots in (("left", slice(0, 7)), ("right", slice(7, 14))):
            solver = getattr(self.arm, side + "_solver")
            alignment = getattr(self.arm, side + "_alignment")
            palm = self.palm_mapping.robot_wrist_to_palm[side]
            target_wrist = alignment.last_target
            target = target_wrist @ palm if target_wrist is not None and alignment.aligned else None
            data.qpos[solver.qpos_indices] = getattr(self.arm, side + "_q")
            mujoco.mj_forward(model, data)
            pose = np.eye(4)
            pose[:3, 3] = data.site_xpos[solver.site_id]
            pose[:3, :3] = data.site_xmat[solver.site_id].reshape(3, 3)
            solved = pose @ palm
            actual = None
            if self._feedback_fresh(now):
                actual_q = self._feedback["position_rad"][np.argsort(self._perm[:14])]
                data.qpos[solver.qpos_indices] = actual_q[slots]
                mujoco.mj_forward(model, data)
                pose[:3, 3] = data.site_xpos[solver.site_id]
                pose[:3, :3] = data.site_xmat[solver.site_id].reshape(3, 3)
                actual = pose @ palm
            info = diagnostics.get(side, {})
            preview[side] = {
                "target_palm": None if target is None else target.tolist(),
                "actual_palm": None if actual is None else actual.tolist(),
                "solver_palm": solved.tolist(),
                "position_error_m": None if actual is None or target is None else float(np.linalg.norm(target[:3, 3] - actual[:3, 3])),
                "orientation_error_rad": None if actual is None or target is None else float(np.linalg.norm(
                    Rotation.from_matrix(target[:3, :3] @ actual[:3, :3].T).as_rotvec())),
                "state": info.get("state", "preview" if target is not None else "unaligned"),
                "detail": info.get("detail", ""),
            }
        return preview

    def _drop(self, mask: int, reason: str) -> None:
        was_active = bool((self.running_mask | self.ready_mask) & ARMS_READY)
        self.ready_mask &= ~mask
        self.running_mask &= ~mask
        if mask & ARMS_READY:
            self._aligning = False
            if was_active:
                self.arm.reset_motion()
        self.reason = reason

    def connected(self) -> None:
        self.normalizer.reset_stream()
        self.tracking = None
        self.input_mask = 0
        self._reset_mapping()
        self._drop(VALID_READY_MASK, "connected; waiting for fresh input and Align")

    def disconnected(self) -> None:
        self.tracking = None
        self.input_mask = 0
        self._reset_mapping()
        self._drop(VALID_READY_MASK, "disconnected; Align/Start required after reconnect")

    def _fresh(self, now: int) -> bool:
        return self.tracking is not None and 0 <= now - self.tracking.bridge_monotonic_ns <= self.FRESH_NS

    def _check_age(self, now: int) -> None:
        if not self._fresh(now):
            self.input_mask = 0
            self._drop(VALID_READY_MASK, "waiting/stale input; Align/Start required")
            if self._calibrating:
                self._calibration_previous = None
                self._calibration_count = 0
        if self.palm_mapping is not None and (self.ready_mask | self.running_mask) & ARMS_READY and not self._feedback_fresh(now):
            self._drop(ARMS_READY, "actual robot feedback stale; Align required")

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
        left_wrist_valid, left_arm_hand = _arm_hand(frame.left, tracking.left_hand, tracking.left_active)
        right_wrist_valid, right_arm_hand = _arm_hand(frame.right, tracking.right_hand, tracking.right_active)
        self.tracking = replace(tracking, left_active=left_wrist_valid, right_active=right_wrist_valid,
                                left_hand=left_arm_hand, right_hand=right_arm_hand)
        if epoch and epoch != tracking.tracking_epoch:
            self._reset_mapping()
            self._drop(VALID_READY_MASK, "device clock rollback; Align/Start required")
        if not self._fresh(now):
            self._check_age(now)
            return False
        from .pico_hands import PicoHandFrame

        result = self.hands.retarget(PicoHandFrame(
            tracking.left_hand, tracking.right_hand, tracking.left_active, tracking.right_active,
            tracking.tracking_epoch, tracking.sequence, tracking.source_timestamp_ns,
            tracking.left_scale, tracking.right_scale,
        ))
        self.input_mask = ((ARMS_READY if left_wrist_valid and right_wrist_valid else 0)
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
        self.arm.accept_tracking(self.tracking)
        self._accept_calibration(frame, tracking, now)
        # Keep the wrist reference current while awaiting authorization, without
        # advancing robot targets. Otherwise normal accumulated motion is
        # mistaken for a single-frame jump on the first running tick.
        if self._aligning or (self.ready_mask & ARMS_READY and not self.running_mask & ARMS_READY):
            outputs = [alignment.accept(hand[1], True, tracking.tracking_epoch,
                                         tracking.source_timestamp_ns,
                                         now_ns=tracking.source_timestamp_ns + now - tracking.bridge_monotonic_ns)
                       for alignment, hand in ((self.arm.left_alignment, self.tracking.left_hand),
                                               (self.arm.right_alignment, self.tracking.right_hand))]
            if self._aligning and all(output.valid for output in outputs):
                self.ready_mask |= ARMS_READY
                self._aligning = False
                self.reason = "aligned and holding; SPD authorization required before Start"
            elif not self._aligning and not all(output.valid for output in outputs):
                self._drop(ARMS_READY, "wrist alignment invalid; Align/Start required")
        return True

    def command(self, command: str, *, now_ns: int | None = None) -> bool:
        now = int(self._clock_ns() if now_ns is None else now_ns)
        self._check_age(now)
        command = command.strip().lower()
        if command == "hold":
            self._calibrating = False
            self._drop(VALID_READY_MASK, "operator Hold; Align and SPD authorization required")
            return True
        if command == "calibrate":
            self._drop(VALID_READY_MASK, "standard palm/direction calibration; robot held")
            if self.palm_mapping is None:
                self.reason = "palm mapping is unavailable in this controller"
                return False
            if not self.input_mask & ARMS_READY or not self._settled_feedback(now):
                self._mapping_reason = self.reason
                return False
            self._reset_mapping()
            self._calibrating = True
            self.session_id = uuid.uuid4().hex
            self._sequence = 0
            self._mapping_reason = "face forward; fingers forward and palms down; keep steady"
            self.reason = self._mapping_reason
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
        if self.palm_mapping is not None:
            if not self.palm_mapping.calibrated or self._calibrating:
                self.reason = "K palm/direction calibration required before position Align"
                return False
            if not self._settled_feedback(now):
                return False
            actual = self._feedback["position_rad"]
            if np.any(actual < self.limits[:14, 0]) or np.any(actual > self.limits[:14, 1]):
                self.reason = "actual robot must be inside joint limits before Align"
                return False
            self.position[:14] = actual
            ordered = actual[np.argsort(self._perm[:14])]
            self.arm.left_q, self.arm.right_q = ordered[:7].copy(), ordered[7:].copy()
            self.arm.reset_motion()
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
        dt = min(self.FRESH_NS, max(0, now - self._last_tick_ns)) * 1e-9 if self._last_tick_ns is not None else 0.0
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
                "reason": self.reason, "epoch": self.normalizer.epoch,
                "mapping": {"calibrated": self.palm_mapping is not None and self.palm_mapping.calibrated,
                            "calibrating": self._calibrating, "reason": self._mapping_reason},
                "arm_preview": self._palm_preview(now),
                "arm_solver": self.arm.diagnostics() if self.palm_mapping is not None else {},
                "actual_feedback_fresh": self._feedback_fresh(now)}


__all__ = ["PicoRosSourceCore", "PicoTeleopCore"]
