"""Python MuJoCo plant owner and 480 Hz operator runtime."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import threading
import time
from typing import Any, Callable, Mapping, Sequence

import numpy as np

from .arm_target_protocol import ArmTargetFrame, ArmTargetHoldReason, LEFT_VALID, RIGHT_VALID, decode_packet
from .manifest import ManifestError, ManifestJoint, load_manifest, resolve_model_addresses
from .model_compiler.artifacts import ArtifactError, verify_artifacts
from .session_state import SessionController, SessionState
from .viewer_window import ViewerWindow
from .wire import ARM_TARGETS_KEY, CONTROL_KEY, TRACKING_KEY, ControlCommand, ControlFrame, decode_arm_target, decode_control, decode_tracking
from .zenoh_transport import LatestSample


PHYSICS_HZ = 480
RENDER_HZ = 60
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
        self.synthetic = False
        if strict_artifacts is None:
            strict_artifacts = model is None
        if model is None:
            module_root = Path(__file__).resolve().parents[1]
            generated = module_root / "generated"
            urdf = Path(__file__).resolve().parents[4] / "assets" / "tianji_wuji2" / "tianji_wuji2.urdf"
            if model_path is None:
                model_path = generated / "unified_plant.xml"
            if manifest_path is None:
                manifest_path = generated / "model_manifest.yaml"
            if strict_artifacts:
                verified = verify_artifacts(manifest_path, urdf)
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
        self._hand_retargeter = hand_retargeter
        self._arm_lock = threading.Lock()
        self._tracking_lock = threading.Lock()
        self._arm_mailbox: _ArmMailbox | None = None
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
        self._fresh_alignment_required = False
        self._node: Any | None = None
        self._arm_wire: LatestSample[ArmTargetFrame] | None = None
        self._tracking_wire: LatestSample[Any] | None = None
        self._control_wire: LatestSample[ControlFrame] | None = None
        self._wire_arm_generation = 0
        self._wire_tracking_generation = 0
        self._wire_control_generation = 0
        self._control_callback: Callable[[ControlFrame], Any] | None = None
        self.paused = False
        self._closed = False
        self.tick = 0
        self._set_home_state()

    @classmethod
    def synthetic_fixture(cls) -> "PlantController":
        try:
            import mujoco
        except ImportError as exc:  # pragma: no cover
            raise ImportError("mujoco is required for the synthetic fixture") from exc

        model = mujoco.MjModel.from_xml_string(_synthetic_xml())
        data = mujoco.MjData(model)
        plant = cls(model=model, data=data, joints=_synthetic_joints(), strict_artifacts=False)
        plant.synthetic = True
        return plant
    def connect(self, node: Any, control_callback: Callable[[ControlFrame], Any] | None = None) -> None:
        """Attach canonical Zenoh inputs; callbacks only fill one-slot mailboxes."""
        if self._node is not None:
            raise RuntimeError("plant is already connected")
        self._node = node
        self._control_callback = control_callback
        self._arm_wire = LatestSample()
        self._tracking_wire = LatestSample()
        self._control_wire = LatestSample()
        node.declare_latest_subscriber(ARM_TARGETS_KEY, decode_arm_target, self._arm_wire)
        node.declare_latest_subscriber(TRACKING_KEY, decode_tracking, self._tracking_wire)
        node.declare_latest_subscriber(CONTROL_KEY, decode_control, self._control_wire)

    def disconnect(self) -> None:
        node, self._node = self._node, None
        if node is not None:
            node.close()
        self._control_callback = None

    def _poll_wire(self) -> None:
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
        if self._control_wire is not None:
            sample = self._control_wire.take_new(self._wire_control_generation)
            if sample is not None:
                self._wire_control_generation, frame = sample
                if self._control_callback is not None:
                    self._control_callback(frame)

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

    def require_fresh_alignment(self) -> None:
        self._fresh_alignment_required = True
        self._last_arm_sequence = None
        self._last_tracking_sequence = None
        with self._arm_lock:
            self._arm_mailbox = None
        with self._tracking_lock:
            self._tracking_mailbox = None
        self._arm_valid = {"left": False, "right": False}
        self._hand_valid = {"left": False, "right": False}

    def mark_alignment_fresh(self) -> None:
        self._fresh_alignment_required = False

    def reset_home(self) -> None:
        self._set_home_state()
        self.tick = 0
        self._fresh_alignment_required = True
        self._arm_mailbox = None
        self._tracking_mailbox = None
        self._arm_generation = self._tracking_generation = 0
        self._applied_arm_generation = self._applied_tracking_generation = 0
        self._last_arm_sequence = self._last_tracking_sequence = None
        self._arm_valid = {"left": False, "right": False}
        self._hand_valid = {"left": False, "right": False}
        self._arm_arrival = {"left": None, "right": None}
        self._hand_arrival = {"left": None, "right": None}
        self._arm_values = {"left": self._home[:7].copy(), "right": self._home[27:34].copy()}
        self._hand_values = {"left": self._home[7:27].copy(), "right": self._home[34:54].copy()}

    def set_paused(self, paused: bool) -> None:
        self.paused = bool(paused)
        if not self.paused:
            self.require_fresh_alignment()

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
        with self._arm_lock:
            self._arm_mailbox = _ArmMailbox(
                self._arm_generation,
                frame,
                int(time.monotonic_ns() if now_ns is None else now_ns),
            )
        return self._arm_generation

    on_arm_target = submit_arm_target
    def submit_arm_packet(self, packet: bytes, *, now_ns: int | None = None) -> int:
        return self.submit_arm_target(decode_packet(packet), now_ns=now_ns)

    on_arm_target_packet = submit_arm_packet

    def submit_tracking(self, frame: Any, *, now_ns: int | None = None) -> int:
        epoch = int(getattr(frame, "tracking_epoch", frame.get("tracking_epoch", 0) if isinstance(frame, Mapping) else 0))
        sequence = int(getattr(frame, "sequence_id", frame.get("sequence_id", 0) if isinstance(frame, Mapping) else 0))
        key = (epoch, sequence)
        if self._last_tracking_sequence is not None and key <= self._last_tracking_sequence:
            return self._tracking_generation
        self._last_tracking_sequence = key
        self._tracking_generation += 1
        with self._tracking_lock:
            self._tracking_mailbox = _TrackingMailbox(
                self._tracking_generation,
                frame,
                int(time.monotonic_ns() if now_ns is None else now_ns),
            )
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
            self._arm_valid[side] = False
            self._arm_reason[side] = reason
            return
        try:
            candidate = self._finite_vector(values, 7, f"{side}_q")
            velocity = self._finite_vector(qdot, 7, f"{side}_qdot")
        except ValueError:
            self._arm_valid[side] = False
            self._arm_reason[side] = ArmTargetHoldReason.SOLVER_FAILURE
            return
        entries = [entry for entry in self.joints if entry.side == side and entry.group == "arm"]
        if any(not (entry.range[0] <= value <= entry.range[1]) for entry, value in zip(entries, candidate)):
            self._arm_valid[side] = False
            self._arm_reason[side] = ArmTargetHoldReason.SOLVER_FAILURE
            return
        if any(entry.velocity_limit is not None and abs(value) > entry.velocity_limit for entry, value in zip(entries, velocity)):
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
            reason = str(result.get(f"{side}_hold_reason", "inactive"))
        else:
            values = getattr(result, f"{side}_qpos", None)
            valid = bool(getattr(result, f"{side}_valid", False))
            reason = str(getattr(result, f"{side}_hold_reason", "inactive"))
        return values, valid, reason


    def _process_tracking(self, mailbox: _TrackingMailbox) -> None:
        frame = mailbox.frame
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
                    entries = [entry for entry in self.joints if entry.side == side and entry.group == "hand"]
                    if any(not (entry.range[0] <= value <= entry.range[1]) for entry, value in zip(entries, candidate)):
                        valid, reason = False, "invalid"
                    else:
                        self._hand_values[side] = candidate
            self._hand_valid[side] = bool(valid)
            self._hand_reason[side] = "none" if valid else reason
            if valid:
                self._hand_arrival[side] = mailbox.arrival_ns

    def _refresh_stale(self, now_ns: int) -> None:
        for side in ("left", "right"):
            if self._arm_valid[side] and self._arm_arrival[side] is not None and now_ns - self._arm_arrival[side] > INPUT_STALE_NS:
                self._arm_valid[side] = False
                self._arm_reason[side] = ArmTargetHoldReason.INPUT_STALE
            if self._hand_valid[side] and self._hand_arrival[side] is not None and now_ns - self._hand_arrival[side] > INPUT_STALE_NS:
                self._hand_valid[side] = False
                self._hand_reason[side] = "input_stale"

    def _apply_ctrl(self) -> None:
        for entry in self.joints:
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
        if self.paused:
            return self._step_snapshot()
        with self._arm_lock:
            arm_mailbox = self._arm_mailbox
        if arm_mailbox is not None and arm_mailbox.generation != self._applied_arm_generation:
            frame = arm_mailbox.frame
            self._arm_side("left", frame.left_q, frame.left_qdot, bool(frame.valid_mask & LEFT_VALID), frame.left_hold_reason, arm_mailbox.arrival_ns)
            self._arm_side("right", frame.right_q, frame.right_qdot, bool(frame.valid_mask & RIGHT_VALID), frame.right_hold_reason, arm_mailbox.arrival_ns)
            self._applied_arm_generation = arm_mailbox.generation
        with self._tracking_lock:
            tracking_mailbox = self._tracking_mailbox
        if tracking_mailbox is not None and tracking_mailbox.generation != self._applied_tracking_generation:
            self._process_tracking(tracking_mailbox)
            self._applied_tracking_generation = tracking_mailbox.generation
        self._refresh_stale(now)
        self._apply_ctrl()
        self._mujoco.mj_step(self.model, self.data)
        self.tick += 1
        return self._step_snapshot()

    step = physics_tick

    def _step_snapshot(self) -> PlantStep:
        arm_mask = (LEFT_VALID if self._arm_valid["left"] else 0) | (RIGHT_VALID if self._arm_valid["right"] else 0)
        hand_mask = (LEFT_VALID if self._hand_valid["left"] else 0) | (RIGHT_VALID if self._hand_valid["right"] else 0)
        finite = bool(
            np.all(np.isfinite(self.data.qpos))
            and np.all(np.isfinite(self.data.qvel))
            and np.all(np.isfinite(self.data.ctrl))
            and np.isfinite(self.data.time)
        )
        return PlantStep(self.tick, self.sim_time_ns, arm_mask, hand_mask, finite)

    @property
    def arm_valid_mask(self) -> int:
        return (LEFT_VALID if self._arm_valid["left"] else 0) | (RIGHT_VALID if self._arm_valid["right"] else 0)

    @property
    def hand_valid_mask(self) -> int:
        return (LEFT_VALID if self._hand_valid["left"] else 0) | (RIGHT_VALID if self._hand_valid["right"] else 0)

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
        if window is None:
            window = ViewerWindow(
                getattr(plant, "model", None),
                getattr(plant, "data", None),
                headless=self.headless,
                shutdown=self._shutdown_from_window,
            )
        self.window = window
        self._next_sequence = 0
    def connect(self, node: Any) -> None:
        connect = getattr(self.plant, "connect", None)
        if connect is None:
            raise TypeError("plant does not support Zenoh connections")
        connect(node, self.session.apply)

    def _shutdown_from_window(self) -> None:
        self.send_control(ControlCommand.SHUTDOWN)

    def send_control(self, command: ControlCommand) -> Any:
        self._next_sequence += 1
        timestamp_ns = max(1, int(self._clock_ns()))
        return self.session.apply(ControlFrame(self._next_sequence, timestamp_ns, command))
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
                self.plant.physics_tick(now)
                count += 1
                if not self.headless and now >= render_deadline:
                    sync = getattr(self.window, "sync", None)
                    if sync is not None:
                        sync()
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
    parser.add_argument("--model", type=Path, default=None)
    parser.add_argument("--manifest", type=Path, default=None)
    args = parser.parse_args(argv)
    if args.ticks is None and args.headless:
        args.ticks = PHYSICS_HZ
    if args.headless and args.model is None and args.manifest is None:
        plant = PlantController.synthetic_fixture()
        synthetic = True
    else:
        plant = PlantController(args.model, args.manifest, strict_artifacts=True)
        synthetic = False
    runtime = ViewerRuntime(plant, headless=args.headless)
    count = runtime.run(ticks=args.ticks, auto_start=args.auto_start)
    finite = bool(np.all(np.isfinite(plant.data.qpos)) and np.all(np.isfinite(plant.data.qvel)) and np.all(np.isfinite(plant.data.ctrl)))
    print(f"headless={args.headless} ticks={count} simulated_seconds={plant.sim_time_ns / 1e9:.6f} finite={finite} synthetic={synthetic}")
    return 0 if finite and (args.ticks is None or count == args.ticks) else 1


__all__ = ["PHYSICS_HZ", "PlantController", "PlantStep", "RENDER_HZ", "ViewerRuntime", "main"]

if __name__ == "__main__":
    raise SystemExit(main())
