"""Python MuJoCo plant owner and 480 Hz operator runtime."""

from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path
import queue
import threading
import time
from typing import Any, Callable, Mapping, Sequence

import numpy as np

from .arm_target_protocol import ArmTargetFrame, ArmTargetHoldReason, LEFT_VALID, RIGHT_VALID, decode_packet
from .manifest import ManifestError, ManifestJoint, load_manifest, resolve_model_addresses
from .model_compiler.artifacts import ArtifactError, verify_artifacts
from .session_state import SessionController, SessionState
from .wire import (
    ARM_TARGETS_KEY,
    CONTROL_KEY,
    STATUS_BRIDGE_KEY,
    STATUS_IK_KEY,
    TRACKING_KEY,
    ControlCommand,
    ControlFrame,
    decode_arm_target,
    decode_control,
    decode_tracking,
)
from .viewer_window import ViewerWindow
from .zenoh_transport import CONTROL_CONGESTION_CONTROL, LatestSample, ZenohNode, peer_config


PHYSICS_HZ = 480
RENDER_HZ = 60
def _decode_status(payload: bytes) -> Mapping[str, Any]:
    try:
        value = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError):
        return {"status": "invalid"}
    return value if isinstance(value, Mapping) else {"status": "invalid"}
INPUT_STALE_NS = 50_000_000
TIMESTEP_NS = 1_000_000_000 // PHYSICS_HZ


@dataclass(frozen=True, slots=True)
class PlantStep:
    tick: int
    sim_time_ns: int
    arm_valid_mask: int
    hand_valid_mask: int
    finite: bool

    @property
    def hand_left_valid(self) -> bool:
        return bool(self.hand_valid_mask & LEFT_VALID)

    @property
    def hand_right_valid(self) -> bool:
        return bool(self.hand_valid_mask & RIGHT_VALID)


@dataclass(frozen=True, slots=True)
class _ArmMailbox:
    generation: int
    frame: ArmTargetFrame
    arrival_ns: int


@dataclass(frozen=True, slots=True)
class _TrackingMailbox:
    generation: int
    frame: Any
    arrival_ns: int


class _ControlFIFO:
    def __init__(self) -> None:
        self._queue: queue.SimpleQueue[ControlFrame] = queue.SimpleQueue()

    def put(self, frame: ControlFrame) -> None:
        self._queue.put(frame)

    def drain(self) -> list[ControlFrame]:
        values: list[ControlFrame] = []
        while True:
            try:
                values.append(self._queue.get_nowait())
            except queue.Empty:
                return values


def _synthetic_xml() -> str:
    def chain(prefix: str, x: float) -> str:
        bodies = ""
        for index in reversed(range(27)):
            joint = f'{prefix}_j{index}'
            actuator = f'{prefix}_a{index}'
            pos = f"{x} 0 0" if index == 0 else "0.035 0 0"
            site = (
                f'<site name="{prefix}_wrist_target" pos="0.035 0 0" size="0.008"/>'
                if index == 6
                else ""
            )
            bodies = (
                f'<body name="{prefix}_b{index}" pos="{pos}">'
                f'<joint name="{joint}" type="hinge" axis="0 0 1" range="-2 2"/>'
                f'<geom type="capsule" fromto="0 0 0 0.035 0 0" size="0.01"/>'
                f"{site}{bodies}</body>"
            )
        return bodies

    actuators = "".join(
        f'<motor name="{side}_a{i}" joint="{side}_j{i}" ctrlrange="-2 2" ctrllimited="true"/>'
        for side in ("l", "r")
        for i in range(27)
    )
    return (
        '<mujoco model="spd-vr-synthetic">'
        f'<option timestep="{1.0 / PHYSICS_HZ:.17g}" gravity="0 0 0"/>'
        f"<worldbody>{chain('l', -0.45)}{chain('r', 0.45)}</worldbody>"
        f"<actuator>{actuators}</actuator></mujoco>"
    )


def _synthetic_joints() -> list[ManifestJoint]:
    joints: list[ManifestJoint] = []
    for index in range(54):
        side = "left" if index < 27 else "right"
        side_index = index if index < 27 else index - 27
        group = "arm" if side_index < 7 else "hand"
        prefix = "l" if side == "left" else "r"
        joints.append(
            ManifestJoint(
                index=index,
                side=side,
                group=group,
                joint=f"{prefix}_j{side_index}",
                actuator=f"{prefix}_a{side_index}",
                qpos_address=index,
                dof_address=index,
                range=(-2.0, 2.0),
                velocity_limit=10.0,
            )
        )
    return joints


class PlantController:
    """Own the complete ``MjModel``/``MjData`` and apply latest targets at ticks."""

    physics_hz = PHYSICS_HZ
    render_hz = RENDER_HZ
    arm_target_hz = 200
    hand_target_hz = 60

    def __init__(
        self,
        model_path: str | Path | None = None,
        manifest_path: str | Path | None = None,
        *,
        model: Any | None = None,
        data: Any | None = None,
        joints: Sequence[ManifestJoint] | None = None,
        hand_retargeter: Any | None = None,
        strict_artifacts: bool | None = None,
    ) -> None:
        try:
            import mujoco
        except ImportError as exc:  # pragma: no cover - package dependency
            raise ImportError("mujoco is required for PlantController") from exc
        self._mujoco = mujoco
        production_model = model is None
        verified = None
        urdf_path: Path | None = None
        self.synthetic = False
        if strict_artifacts is None:
            strict_artifacts = production_model
        if production_model:
            module_root = Path(__file__).resolve().parents[1]
            generated = module_root / "generated"
            urdf_path = Path(__file__).resolve().parents[4] / "assets" / "tianji_wuji2" / "tianji_wuji2.urdf"
            if model_path is None:
                model_path = generated / "unified_plant.xml"
            if manifest_path is None:
                manifest_path = generated / "model_manifest.yaml"
            if strict_artifacts:
                verified = verify_artifacts(manifest_path, urdf_path)
                if Path(model_path).resolve() != verified.full_model.resolve():
                    raise ArtifactError("viewer must load manifest unified_plant.xml")
            model = mujoco.MjModel.from_xml_path(str(model_path))
        if data is None:
            data = mujoco.MjData(model)
        self.model = model
        self.data = data
        if joints is not None:
            self.joints = list(joints)
        elif manifest_path is not None and Path(manifest_path).is_file():
            manifest = load_manifest(manifest_path)
            self.joints = resolve_model_addresses(model, manifest, allow_scene_dofs=model.nq > 54)
        elif model.nq == 54 and model.nv == 54:
            self.joints = _synthetic_joints()
            self.synthetic = True
        else:
            raise ManifestError("full plant requires a verified 54-DoF manifest")
        if len(self.joints) != 54 or int(model.nq) != 54 or int(model.nv) != 54:
            raise ManifestError("full plant must expose exactly 54 qpos/qvel entries")
        self._actuator_ids = {
            entry.actuator: int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, entry.actuator))
            for entry in self.joints
        }
        if any(value < 0 for value in self._actuator_ids.values()):
            raise ManifestError("manifest actuator address resolution failed")
        self._home = np.asarray(
            [(entry.range[0] + entry.range[1]) * 0.5 for entry in self.joints], dtype=np.float64
        )
        if hand_retargeter is None and production_model:
            if verified is None or urdf_path is None:
                raise ArtifactError("production viewer requires verified model artifacts")
            from .retarget_pair import WujiRetargetPair

            config_dir = Path(__file__).resolve().parents[1] / "config"
            hand_retargeter = WujiRetargetPair.from_manifest(
                config_dir / "wuji2_pico_left.yaml",
                config_dir / "wuji2_pico_right.yaml",
                verified.manifest_path,
                urdf_path,
            )
        self._hand_retargeter = hand_retargeter
        self._arm_lock = threading.Lock()
        self._tracking_lock = threading.Lock()
        self._arm_mailbox: _ArmMailbox | None = None
        self.artifact_hash = (
            str(verified.manifest.get("manifest_sha256", "unknown"))
            if verified is not None
            else "synthetic"
        )
        self._tracking_mailbox: _TrackingMailbox | None = None
        self._arm_generation = 0
        self._tracking_generation = 0
        self._last_arm_sequence: tuple[int, int] | None = None
        self._last_tracking_sequence: tuple[int, int] | None = None
        self._arm_values = {"left": self._home[:7].copy(), "right": self._home[27:34].copy()}
        self._hand_values = {"left": self._home[7:27].copy(), "right": self._home[34:54].copy()}
        self._arm_valid = {"left": False, "right": False}
        self._hand_valid = {"left": False, "right": False}
        self._arm_reason = {"left": ArmTargetHoldReason.INPUT_STALE, "right": ArmTargetHoldReason.INPUT_STALE}
        self._hand_reason = {"left": "inactive", "right": "inactive"}
        self._arm_arrival = {"left": None, "right": None}
        self._hand_arrival = {"left": None, "right": None}
        self._applied_arm_generation = 0
        self._applied_tracking_generation = 0
        self._tracking_arrival_ns: list[int] = []
        self._arm_arrival_ns: list[int] = []
        self._alignment_generation = 0
        self._alignment_ready = {"left": True, "right": True}
        self._fresh_arm_valid = {"left": True, "right": True}
        self._fresh_hand_valid = {"left": True, "right": True}
        self._required_control_timestamp_ns = 0
        self.invalid_input_count = 0
        self._fresh_alignment_required = False
        self._node: Any | None = None
        self._arm_wire: LatestSample[ArmTargetFrame] | None = None
        self._tracking_wire: LatestSample[Any] | None = None
        self._control_wire: _ControlFIFO | None = None
        self._wire_arm_generation = 0
        self._wire_tracking_generation = 0
        self._wire_control_generation = 0
        self._control_callback: Callable[[ControlFrame], Any] | None = None
        self.paused = False
        self._closed = False
        self.tick = 0
        self._set_home_state()

    @classmethod
    def synthetic_fixture(cls, *, hand_retargeter: Any | None = None) -> "PlantController":
        try:
            import mujoco
        except ImportError as exc:  # pragma: no cover
            raise ImportError("mujoco is required for the synthetic fixture") from exc

        model = mujoco.MjModel.from_xml_string(_synthetic_xml())
        data = mujoco.MjData(model)
        plant = cls(
            model=model,
            data=data,
            joints=_synthetic_joints(),
            hand_retargeter=hand_retargeter,
            strict_artifacts=False,
        )
        plant.synthetic = True
        return plant
    def connect(self, node: Any, control_callback: Callable[[ControlFrame], Any] | None = None) -> None:
        """Attach canonical Zenoh inputs; callbacks only fill bounded mailboxes."""
        if self._node is not None:
            raise RuntimeError("plant is already connected")
        self._node = node
        self._control_callback = control_callback
        self._arm_wire = LatestSample()
        self._tracking_wire = LatestSample()
        self._control_wire = _ControlFIFO()
        node.declare_latest_subscriber(ARM_TARGETS_KEY, decode_arm_target, self._arm_wire)
        node.declare_latest_subscriber(TRACKING_KEY, decode_tracking, self._tracking_wire)
        node.declare_latest_subscriber(CONTROL_KEY, decode_control, self._control_wire)

    def disconnect(self) -> None:
        node, self._node = self._node, None
        if node is not None:
            node.close()
        self._control_callback = None

    def _poll_wire(self) -> None:
        if self._control_wire is not None:
            for frame in self._control_wire.drain():
                if self._control_callback is not None:
                    self._control_callback(frame)
        if self._arm_wire is not None:
            sample = self._arm_wire.take_new(self._wire_arm_generation)
            if sample is not None:
                self._wire_arm_generation, frame = sample
                self.submit_arm_target(frame)
        if self._tracking_wire is not None:
            sample = self._tracking_wire.take_new(self._wire_tracking_generation)
            if sample is not None:
                self._wire_tracking_generation, frame = sample
                self.submit_tracking(frame)

    @classmethod
    def synthetic(cls) -> "PlantController":
        return cls.synthetic_fixture()

    def _set_home_state(self) -> None:
        self.data.qpos[:] = 0.0
        self.data.qvel[:] = 0.0
        for entry in self.joints:
            self.data.qpos[entry.qpos_address] = (entry.range[0] + entry.range[1]) * 0.5
        self.data.ctrl[:] = 0.0
        if getattr(self.data, "act", None) is not None:
            self.data.act[:] = 0.0
        self.data.time = 0.0
        self._mujoco.mj_forward(self.model, self.data)

    @property
    def sim_time_ns(self) -> int:
        return int(round(float(self.data.time) * 1_000_000_000.0))

    @property
    def requires_fresh_alignment(self) -> bool:
        return self._fresh_alignment_required

    @property
    def required_control_timestamp_ns(self) -> int:
        return self._required_control_timestamp_ns

    def require_fresh_alignment(self, control_timestamp_ns: int | None = None) -> None:
        self._required_control_timestamp_ns = max(
            0,
            int(control_timestamp_ns)
            if control_timestamp_ns is not None
            else self._required_control_timestamp_ns,
        )
        self._fresh_alignment_required = True
        self._alignment_ready = {"left": False, "right": False}
        self._fresh_arm_valid = {"left": False, "right": False}
        self._fresh_hand_valid = {"left": False, "right": False}
        self._last_arm_sequence = None
        self._last_tracking_sequence = None
        with self._arm_lock:
            self._arm_mailbox = None
        with self._tracking_lock:
            self._tracking_mailbox = None
        if self._arm_wire is not None:
            self._arm_wire.invalidate()
        if self._tracking_wire is not None:
            self._tracking_wire.invalidate()
        self._arm_valid = {"left": False, "right": False}
        self._hand_valid = {"left": False, "right": False}

    def _update_alignment_ready(self) -> None:
        for side in ("left", "right"):
            self._alignment_ready[side] = self._fresh_arm_valid[side] and self._fresh_hand_valid[side]
        self._fresh_alignment_required = not all(self._alignment_ready.values())

    def mark_alignment_fresh(self) -> None:
        self._update_alignment_ready()

    def reset_home(self, control_timestamp_ns: int | None = None) -> None:
        self._set_home_state()
        self.tick = 0
        self._arm_generation = self._tracking_generation = 0
        self._applied_arm_generation = self._applied_tracking_generation = 0
        self._arm_arrival = {"left": None, "right": None}
        self._hand_arrival = {"left": None, "right": None}
        self._arm_values = {"left": self._home[:7].copy(), "right": self._home[27:34].copy()}
        self._hand_values = {"left": self._home[7:27].copy(), "right": self._home[34:54].copy()}
        self.require_fresh_alignment(control_timestamp_ns)

    def set_paused(self, paused: bool) -> None:
        self.paused = bool(paused)

    @staticmethod
    def _finite_vector(values: Any, size: int, name: str) -> np.ndarray:
        result = np.asarray(values, dtype=np.float64).reshape(-1)
        if result.shape != (size,) or not np.all(np.isfinite(result)):
            raise ValueError(f"{name} must be finite and have length {size}")
        return result

    def submit_arm_target(self, frame: ArmTargetFrame | Mapping[str, Any], *, now_ns: int | None = None) -> int:
        if isinstance(frame, Mapping):
            frame = ArmTargetFrame(
                sequence=int(frame["sequence"]),
                tracking_epoch=int(frame["tracking_epoch"]),
                source_timestamp_ns=int(frame["source_timestamp_ns"]),
                control_timestamp_ns=int(frame["control_timestamp_ns"]),
                valid_mask=int(frame["valid_mask"]),
                left_hold_reason=ArmTargetHoldReason(int(frame["left_hold_reason"])),
                right_hold_reason=ArmTargetHoldReason(int(frame["right_hold_reason"])),
                left_q=tuple(frame["left_q"]),
                right_q=tuple(frame["right_q"]),
                left_qdot=tuple(frame["left_qdot"]),
                right_qdot=tuple(frame["right_qdot"]),
            )
        if not isinstance(frame, ArmTargetFrame):
            raise TypeError("arm target must be an ArmTargetFrame")
        key = (int(frame.tracking_epoch), int(frame.sequence))
        if self._last_arm_sequence is not None and key <= self._last_arm_sequence:
            return self._arm_generation
        self._last_arm_sequence = key
        self._arm_generation += 1
        arrival_ns = int(time.monotonic_ns() if now_ns is None else now_ns)
        self._arm_arrival_ns.append(arrival_ns)
        if len(self._arm_arrival_ns) > 32:
            del self._arm_arrival_ns[:-32]
        with self._arm_lock:
            self._arm_mailbox = _ArmMailbox(self._arm_generation, frame, arrival_ns)
        return self._arm_generation

    on_arm_target = submit_arm_target
    def submit_arm_packet(self, packet: bytes, *, now_ns: int | None = None) -> int:
        return self.submit_arm_target(decode_packet(packet), now_ns=now_ns)

    on_arm_target_packet = submit_arm_packet

    def submit_tracking(self, frame: Any, *, now_ns: int | None = None) -> int:
        if isinstance(frame, Mapping):
            epoch = int(frame.get("tracking_epoch", 0))
            sequence = int(frame.get("sequence", frame.get("sequence_id", 0)))
        else:
            epoch = int(getattr(frame, "tracking_epoch", 0))
            sequence = int(getattr(frame, "sequence", getattr(frame, "sequence_id", 0)))
        key = (epoch, sequence)
        if self._last_tracking_sequence is not None and key <= self._last_tracking_sequence:
            return self._tracking_generation
        self._last_tracking_sequence = key
        self._tracking_generation += 1
        arrival_ns = int(time.monotonic_ns() if now_ns is None else now_ns)
        self._tracking_arrival_ns.append(arrival_ns)
        if len(self._tracking_arrival_ns) > 32:
            del self._tracking_arrival_ns[:-32]
        with self._tracking_lock:
            self._tracking_mailbox = _TrackingMailbox(self._tracking_generation, frame, arrival_ns)
        return self._tracking_generation

    on_pico_hands = submit_tracking

    def _arm_side(
        self,
        side: str,
        values: Any,
        qdot: Any,
        valid: bool,
        reason: ArmTargetHoldReason,
        arrival_ns: int,
    ) -> None:
        if not valid:
            if not self._explicit_hold(reason):
                self.invalid_input_count += 1
            self._arm_valid[side] = False
            self._arm_reason[side] = reason
            return
        try:
            candidate = self._finite_vector(values, 7, f"{side}_q")
            velocity = self._finite_vector(qdot, 7, f"{side}_qdot")
        except ValueError:
            self.invalid_input_count += 1
            self._arm_valid[side] = False
            self._arm_reason[side] = ArmTargetHoldReason.SOLVER_FAILURE
            return
        entries = sorted(
            (entry for entry in self.joints if entry.side == side and entry.group == "arm"),
            key=lambda entry: entry.index,
        )
        if any(not (entry.range[0] <= value <= entry.range[1]) for entry, value in zip(entries, candidate)):
            self.invalid_input_count += 1
            self._arm_valid[side] = False
            self._arm_reason[side] = ArmTargetHoldReason.SOLVER_FAILURE
            return
        if any(entry.velocity_limit is not None and abs(value) > entry.velocity_limit for entry, value in zip(entries, velocity)):
            self.invalid_input_count += 1
            self._arm_valid[side] = False
            self._arm_reason[side] = ArmTargetHoldReason.SOLVER_FAILURE
            return
        self._arm_values[side] = candidate
        self._arm_valid[side] = True
        self._arm_reason[side] = ArmTargetHoldReason.NONE
        self._arm_arrival[side] = arrival_ns
    @staticmethod
    def _result_side(result: Any, side: str) -> tuple[Any, bool, str]:
        if isinstance(result, Mapping):
            values = result.get(f"{side}_qpos")
            valid = bool(result.get(f"{side}_valid", False))
            reason = result.get(f"{side}_hold_reason", "inactive")
        else:
            values = getattr(result, f"{side}_qpos", None)
            valid = bool(getattr(result, f"{side}_valid", False))
            reason = getattr(result, f"{side}_hold_reason", "inactive")
        return values, valid, str(getattr(reason, "value", reason))


    @staticmethod
    def _pico_hand_frame(frame: Any) -> Any:
        if not hasattr(frame, "left_hand") or not hasattr(frame, "right_hand"):
            return frame
        from .pico_hands import PicoHandFrame

        return PicoHandFrame(
            left_hand=np.asarray(frame.left_hand),
            right_hand=np.asarray(frame.right_hand),
            left_active=bool(getattr(frame, "left_active", True)),
            right_active=bool(getattr(frame, "right_active", True)),
            tracking_epoch=int(getattr(frame, "tracking_epoch", 0)),
            sequence_id=int(getattr(frame, "sequence", getattr(frame, "sequence_id", 0))),
            timestamp_ns=int(getattr(frame, "source_timestamp_ns", getattr(frame, "timestamp_ns", 0))),
            left_scale=float(getattr(frame, "left_scale", 1.0)),
            right_scale=float(getattr(frame, "right_scale", 1.0)),
        )

    @staticmethod
    def _explicit_hold(reason: Any) -> bool:
        value = getattr(reason, "value", reason)
        return value in {
            ArmTargetHoldReason.PAUSED,
            ArmTargetHoldReason.INACTIVE,
            ArmTargetHoldReason.ALIGNING,
            ArmTargetHoldReason.DISCONNECTED,
            "inactive",
            "paused",
            "aligning",
            "disconnected",
        }

    def _process_tracking(self, mailbox: _TrackingMailbox) -> None:
        frame = self._pico_hand_frame(mailbox.frame)
        if hasattr(frame, "left_qpos") and hasattr(frame, "right_qpos"):
            result = frame
        elif self._hand_retargeter is None:
            result = frame
        else:
            try:
                result = self._hand_retargeter.retarget(frame)
            except Exception:
                result = None
        for side in ("left", "right"):
            values, valid, reason = self._result_side(result, side) if result is not None else (None, False, "solver_failure")
            if valid:
                try:
                    candidate = self._finite_vector(values, 20, f"{side}_qpos")
                except ValueError:
                    valid, reason = False, "solver_failure"
                else:
                    entries = sorted(
                        (entry for entry in self.joints if entry.side == side and entry.group == "hand"),
                        key=lambda entry: entry.index,
                    )
                    if any(not (entry.range[0] <= value <= entry.range[1]) for entry, value in zip(entries, candidate)):
                        valid, reason = False, "invalid"
                    else:
                        self._hand_values[side] = candidate
            if valid:
                self._hand_arrival[side] = mailbox.arrival_ns
            if not valid and not self._explicit_hold(reason):
                self.invalid_input_count += 1
            self._hand_valid[side] = bool(valid)
            self._hand_reason[side] = "none" if valid else reason
            if self._fresh_alignment_required:
                self._fresh_hand_valid[side] = bool(valid) or self._explicit_hold(reason)
        if self._fresh_alignment_required:
            self._update_alignment_ready()

    def _refresh_stale(self, now_ns: int) -> None:
        for side in ("left", "right"):
            if self._arm_valid[side] and self._arm_arrival[side] is not None and now_ns - self._arm_arrival[side] > INPUT_STALE_NS:
                self._arm_valid[side] = False
                self._arm_reason[side] = ArmTargetHoldReason.INPUT_STALE
                if self._fresh_alignment_required:
                    self._fresh_arm_valid[side] = False
            if self._hand_valid[side] and self._hand_arrival[side] is not None and now_ns - self._hand_arrival[side] > INPUT_STALE_NS:
                self._hand_valid[side] = False
                self._hand_reason[side] = "input_stale"
                if self._fresh_alignment_required:
                    self._fresh_hand_valid[side] = False
        if self._fresh_alignment_required:
            self._update_alignment_ready()

    def _apply_ctrl(self) -> None:
        for entry in self.joints:
            if self._fresh_alignment_required and not self._alignment_ready[entry.side]:
                continue
            side_index = entry.index if entry.side == "left" else entry.index - 27
            if entry.group == "arm":
                value = self._arm_values[entry.side][side_index]
            else:
                hand_index = side_index - 7
                value = self._hand_values[entry.side][hand_index]
            actuator_id = self._actuator_ids[entry.actuator]
            value = float(value) if np.isfinite(value) else 0.0
            if getattr(self.model, "actuator_ctrllimited", None) is not None and self.model.actuator_ctrllimited[actuator_id]:
                low, high = self.model.actuator_ctrlrange[actuator_id]
                value = float(np.clip(value, low, high))
            self.data.ctrl[actuator_id] = value
        if not np.all(np.isfinite(self.data.ctrl)):
            self.data.ctrl[:] = np.nan_to_num(self.data.ctrl, nan=0.0, posinf=0.0, neginf=0.0)

    def physics_tick(self, now_ns: int | None = None) -> PlantStep:
        if self._closed:
            raise RuntimeError("plant is shut down")
        now = int(time.monotonic_ns() if now_ns is None else now_ns)
        self._poll_wire()
        if self._closed:
            return self._step_snapshot()
        if self.paused:
            return self._step_snapshot()
        with self._tracking_lock:
            tracking_mailbox = self._tracking_mailbox
        if tracking_mailbox is not None and tracking_mailbox.generation != self._applied_tracking_generation:
            self._process_tracking(tracking_mailbox)
            self._applied_tracking_generation = tracking_mailbox.generation
        with self._arm_lock:
            arm_mailbox = self._arm_mailbox
        if arm_mailbox is not None and arm_mailbox.generation != self._applied_arm_generation:
            frame = arm_mailbox.frame
            frame_token = int(frame.control_timestamp_ns)
            token_ok = (
                not self._fresh_alignment_required
                or (frame_token > 1 and frame_token == self._required_control_timestamp_ns)
            )
            if token_ok:
                self._arm_side("left", frame.left_q, frame.left_qdot, bool(frame.valid_mask & LEFT_VALID), frame.left_hold_reason, arm_mailbox.arrival_ns)
                self._arm_side("right", frame.right_q, frame.right_qdot, bool(frame.valid_mask & RIGHT_VALID), frame.right_hold_reason, arm_mailbox.arrival_ns)
            else:
                self._arm_valid = {"left": False, "right": False}
                self._arm_reason = {"left": ArmTargetHoldReason.INPUT_STALE, "right": ArmTargetHoldReason.INPUT_STALE}
            if self._fresh_alignment_required:
                self._fresh_arm_valid["left"] = token_ok and (self._arm_valid["left"] or self._explicit_hold(frame.left_hold_reason))
                self._fresh_arm_valid["right"] = token_ok and (self._arm_valid["right"] or self._explicit_hold(frame.right_hold_reason))
                self._update_alignment_ready()
            self._applied_arm_generation = arm_mailbox.generation
        self._refresh_stale(now)
        self._apply_ctrl()
        self._mujoco.mj_step(self.model, self.data)
        self.tick += 1
        return self._step_snapshot()

    step = physics_tick

    def _step_snapshot(self) -> PlantStep:
        arm_mask = (LEFT_VALID if self._arm_valid["left"] and self._alignment_ready["left"] else 0) | (RIGHT_VALID if self._arm_valid["right"] and self._alignment_ready["right"] else 0)
        hand_mask = (LEFT_VALID if self._hand_valid["left"] and self._alignment_ready["left"] else 0) | (RIGHT_VALID if self._hand_valid["right"] and self._alignment_ready["right"] else 0)
        finite = bool(
            np.all(np.isfinite(self.data.qpos))
            and np.all(np.isfinite(self.data.qvel))
            and np.all(np.isfinite(self.data.ctrl))
            and np.isfinite(self.data.time)
        )
        return PlantStep(self.tick, self.sim_time_ns, arm_mask, hand_mask, finite)

    @property
    def arm_valid_mask(self) -> int:
        return (LEFT_VALID if self._arm_valid["left"] and self._alignment_ready["left"] else 0) | (RIGHT_VALID if self._arm_valid["right"] and self._alignment_ready["right"] else 0)

    @property
    def hand_valid_mask(self) -> int:
        return (LEFT_VALID if self._hand_valid["left"] and self._alignment_ready["left"] else 0) | (RIGHT_VALID if self._hand_valid["right"] and self._alignment_ready["right"] else 0)

    def shutdown(self) -> None:
        self._closed = True

    close = shutdown


class ViewerRuntime:
    """Run independent absolute-deadline physics and render loops."""

    def __init__(
        self,
        plant: Any,
        *,
        window: ViewerWindow | Any | None = None,
        headless: bool = False,
        session: SessionController | None = None,
        clock_ns: Callable[[], int] | None = None,
        sleep: Callable[[float], None] | None = None,
    ) -> None:
        self.plant = plant
        self.headless = bool(headless)
        self._clock_ns = time.monotonic_ns if clock_ns is None else clock_ns
        self._sleep = time.sleep if sleep is None else sleep
        self.session = session or SessionController(plant)
        self._publisher: Any | None = None
        self._node: Any | None = None
        self._status_mailboxes = {
            "bridge": LatestSample(),
            "ik": LatestSample(),
        }
        self._status_generations = {"bridge": 0, "ik": 0}
        self._status: dict[str, Mapping[str, Any]] = {}
        if window is None:
            window = ViewerWindow(
                getattr(plant, "model", None),
                getattr(plant, "data", None),
                headless=self.headless,
                shutdown=self._shutdown_from_window,
                control=self.send_control,
                state=lambda: self.session.state.value,
            )
        self.window = window
        self._next_sequence = 1
        self._physics_timing_ns: list[int] = []
        self._render_timing_ns: list[int] = []
    def connect(self, node: Any) -> None:
        if self._node is not None:
            raise RuntimeError("runtime is already connected")
        self._node = node
        try:
            self._publisher = node.declare_publisher(
                CONTROL_KEY,
                congestion_control=CONTROL_CONGESTION_CONTROL,
            )
            node.declare_latest_subscriber(STATUS_BRIDGE_KEY, _decode_status, self._status_mailboxes["bridge"])
            node.declare_latest_subscriber(STATUS_IK_KEY, _decode_status, self._status_mailboxes["ik"])
            connect = getattr(self.plant, "connect", None)
            if connect is None:
                raise TypeError("plant does not support Zenoh connections")
            connect(node, self.session.apply)
        except Exception:
            close = getattr(node, "close", None)
            if close is not None:
                close()
            self._node = None
            self._publisher = None
            raise

    def close(self) -> None:
        close_window = getattr(self.window, "close", None)
        if close_window is not None:
            close_window()
        if self._node is not None:
            self.plant.disconnect()
            self._node = None
            self._publisher = None

    def _shutdown_from_window(self) -> None:
        self.send_control(ControlCommand.SHUTDOWN)

    def send_control(self, command: ControlCommand | str) -> Any:
        if isinstance(command, str):
            command = ControlCommand[command.upper()]
        else:
            command = ControlCommand(command)
        timestamp_ns = max(1, int(self._clock_ns()))
        frame = ControlFrame(self._next_sequence, timestamp_ns, command)
        self._next_sequence += 1
        if self._publisher is not None:
            from .wire import encode_control

            self._publisher.put(encode_control(frame))
        return self.session.apply(frame)

    def _poll_status(self) -> None:
        for name, mailbox in self._status_mailboxes.items():
            sample = mailbox.take_new(self._status_generations[name])
            if sample is not None:
                self._status_generations[name], value = sample
                self._status[name] = value
    @staticmethod
    def _timing_stats(samples: list[int]) -> tuple[Any, Any]:
        if not samples:
            return "unknown", "unknown"
        values = np.asarray(samples, dtype=np.float64)
        return round(float(np.percentile(values, 95)) / 1.0e6, 3), round(float(np.max(values)) / 1.0e6, 3)
    def _zenoh_status(self) -> str:
        if self._publisher is None:
            return "disabled"
        if self._status:
            return "remote_status"
        return "connected_unmatched"
    @staticmethod
    def _observed_rate(arrivals: list[int]) -> Any:
        if len(arrivals) < 2 or arrivals[-1] <= arrivals[0]:
            return "unknown"
        return round((len(arrivals) - 1) * 1.0e9 / (arrivals[-1] - arrivals[0]), 3)


    def _hud_values(self, result: Any, now_ns: int) -> dict[str, Any]:
        arrivals = []
        for name in ("_arm_arrival", "_hand_arrival"):
            arrivals.extend(getattr(self.plant, name, {}).values())
        ages = [now_ns - int(arrival) for arrival in arrivals if arrival is not None]
        physics_p95, physics_max = self._timing_stats(self._physics_timing_ns)
        render_p95, render_max = self._timing_stats(self._render_timing_ns)
        arm_reason = getattr(self.plant, "_arm_reason", {})
        hand_reason = getattr(self.plant, "_hand_reason", {})
        alignment = getattr(self.plant, "_alignment_ready", {})
        drops = sum(
            int(getattr(mailbox, "dropped_count", 0))
            for mailbox in (getattr(self.plant, "_arm_wire", None), getattr(self.plant, "_tracking_wire", None))
            if mailbox is not None
        )
        tracking = getattr(getattr(self.plant, "_tracking_mailbox", None), "frame", None)
        source = getattr(tracking, "source_timestamp_ns", None)
        bridge = getattr(tracking, "bridge_monotonic_ns", None)
        source_latency = round((now_ns - int(source)) / 1.0e6, 3) if source is not None and 0 <= int(source) <= now_ns else "unknown"
        bridge_latency = round((int(bridge) - int(source)) / 1.0e6, 3) if bridge is not None and source is not None and int(bridge) >= int(source) else "unknown"
        bridge_status = self._status.get("bridge", {}).get("ready", "unknown")
        ik_status = self._status.get("ik", {}).get("running", "unknown")
        return {
            "state": self.session.state.value,
            "zenoh": self._zenoh_status(),
            "physics_finite": getattr(result, "finite", True),
            "physics_p95_ms": physics_p95,
            "physics_max_ms": physics_max,
            "render_p95_ms": render_p95,
            "render_max_ms": render_max,
            "arm_left": f"{alignment.get('left', 'unknown')}:{arm_reason.get('left', 'unknown')}",
            "arm_right": f"{alignment.get('right', 'unknown')}:{arm_reason.get('right', 'unknown')}",
            "hand_left": f"{alignment.get('left', 'unknown')}:{hand_reason.get('left', 'unknown')}",
            "hand_right": f"{alignment.get('right', 'unknown')}:{hand_reason.get('right', 'unknown')}",
            "arm_valid_mask": getattr(result, "arm_valid_mask", 0),
            "hand_valid_mask": getattr(result, "hand_valid_mask", 0),
            "input_age_ms": round(max(ages, default=0) / 1.0e6, 3) if ages else "unknown",
            "drops": drops,
            "invalid": getattr(self.plant, "invalid_input_count", "unknown"),
            "sdk_status": "unknown",
            "bridge_status": bridge_status,
            "ik_status": ik_status,
            "tracking_rate_hz": self._observed_rate(getattr(self.plant, "_tracking_arrival_ns", [])),
            "arm_target_rate_hz": self._observed_rate(getattr(self.plant, "_arm_arrival_ns", [])),
            "source_latency_ms": source_latency,
            "bridge_latency_ms": bridge_latency,
            "contact": getattr(getattr(self.plant, "data", None), "ncon", "unknown"),
            "artifact_hash": getattr(self.plant, "artifact_hash", "unknown"),
        }
    def run(self, *, ticks: int | None = None, auto_start: bool = False) -> int:
        if ticks is not None and int(ticks) < 0:
            raise ValueError("ticks must be non-negative")
        if not self.headless and hasattr(self.window, "open"):
            self.window.open()
        if auto_start:
            self.send_control(ControlCommand.START)
        period_ns = TIMESTEP_NS
        render_period_ns = 1_000_000_000 // RENDER_HZ
        physics_deadline = int(self._clock_ns())
        render_deadline = physics_deadline
        count = 0
        try:
            while ticks is None or count < int(ticks):
                now = int(self._clock_ns())
                if now < physics_deadline:
                    self._sleep((physics_deadline - now) * 1.0e-9)
                    now = int(self._clock_ns())
                if self.session.state is SessionState.SHUTDOWN:
                    break
                if not self.headless:
                    is_running = getattr(self.window, "is_running", None)
                    if callable(is_running) and not is_running():
                        self._shutdown_from_window()
                        break
                timing_start = time.perf_counter_ns()
                result = self.plant.physics_tick(now)
                self._physics_timing_ns.append(time.perf_counter_ns() - timing_start)
                if len(self._physics_timing_ns) > 1024:
                    del self._physics_timing_ns[:-1024]
                count += 1
                self._poll_status()
                if hasattr(self.plant, "requires_fresh_alignment") and not self.plant.requires_fresh_alignment:
                    self.session.mark_aligned()
                if not self.headless and now >= render_deadline:
                    update_hud = getattr(self.window, "update_hud", None)
                    if update_hud is not None:
                        update_hud(self._hud_values(result, now))
                    sync = getattr(self.window, "sync", None)
                    if sync is not None:
                        timing_start = time.perf_counter_ns()
                        sync()
                        self._render_timing_ns.append(time.perf_counter_ns() - timing_start)
                        if len(self._render_timing_ns) > 1024:
                            del self._render_timing_ns[:-1024]
                    while render_deadline <= now:
                        render_deadline += render_period_ns
                physics_deadline += period_ns
                if physics_deadline <= now:
                    physics_deadline = now + period_ns
        finally:
            if self.session.state is SessionState.SHUTDOWN:
                close = getattr(self.window, "close", None)
                if close is not None:
                    close()
        return count


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--ticks", type=int, default=None)
    parser.add_argument("--auto-start", action="store_true")
    parser.add_argument("--synthetic", action="store_true")
    parser.add_argument("--model", type=Path, default=None)
    parser.add_argument("--manifest", type=Path, default=None)
    parser.add_argument("--endpoint", default="tcp/127.0.0.1:7447")
    args = parser.parse_args(argv)
    if args.ticks is None and args.headless:
        args.ticks = PHYSICS_HZ
    if args.synthetic:
        if not args.headless:
            raise SystemExit("--synthetic requires --headless")
        plant = PlantController.synthetic_fixture()
        synthetic = True
        runtime = ViewerRuntime(plant, headless=True)
        node = None
    else:
        plant = PlantController(args.model, args.manifest, strict_artifacts=True)
        synthetic = False
        runtime = ViewerRuntime(plant, headless=args.headless)
        node = ZenohNode(peer_config(listen=False, endpoint=args.endpoint))
        try:
            runtime.connect(node)
        except Exception:
            node.close()
            plant.close()
            raise
    try:
        count = runtime.run(ticks=args.ticks, auto_start=args.auto_start)
        finite = bool(np.all(np.isfinite(plant.data.qpos)) and np.all(np.isfinite(plant.data.qvel)) and np.all(np.isfinite(plant.data.ctrl)))
        print(f"headless={args.headless} ticks={count} simulated_seconds={plant.sim_time_ns / 1e9:.6f} finite={finite} synthetic={synthetic}")
        return 0 if finite and (args.ticks is None or count == args.ticks) else 1
    finally:
        runtime.close()
        plant.close()


__all__ = ["PHYSICS_HZ", "PlantController", "PlantStep", "RENDER_HZ", "ViewerRuntime", "main"]

if __name__ == "__main__":
    raise SystemExit(main())
