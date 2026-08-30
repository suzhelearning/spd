"""200 Hz dual-arm Jacobian IK controller and fail-closed production entry point."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import sys
import time
from pathlib import Path
from typing import Any, Callable

import numpy as np

try:
    import mujoco
except ImportError:  # pragma: no cover
    mujoco = None  # type: ignore[assignment]

from .alignment import AlignedPose, SideAlignment, _pose_matrix
from .model_compiler.artifacts import ArtifactError, verify_artifacts
from .qp_arm import ArmQPSolver
from .wire import (
    ARM_TARGETS_KEY,
    CONTROL_KEY,
    TRACKING_KEY,
    ArmTargetFrame,
    ArmTargetHoldReason,
    ControlCommand,
    ControlFrame,
    TrackingFrame,
    decode_control,
    decode_tracking,
    encode_arm_target,
)
from .zenoh_transport import LatestSample, ZenohNode


_PERIOD_NS = 5_000_000


def _hold_reason(value: str | None) -> ArmTargetHoldReason:
    return {
        None: ArmTargetHoldReason.NONE,
        "stale": ArmTargetHoldReason.INPUT_STALE,
        "inactive": ArmTargetHoldReason.INACTIVE,
        "aligning": ArmTargetHoldReason.ALIGNING,
        "epoch_change": ArmTargetHoldReason.EPOCH_CHANGE,
        "timestamp_rollback": ArmTargetHoldReason.EPOCH_CHANGE,
        "invalid_pose": ArmTargetHoldReason.INPUT_STALE,
        "disconnected": ArmTargetHoldReason.DISCONNECTED,
        "paused": ArmTargetHoldReason.PAUSED,
        "solver": ArmTargetHoldReason.SOLVER_FAILURE,
    }.get(value, ArmTargetHoldReason.SOLVER_FAILURE)


@dataclass(slots=True)
class _SideOutput:
    q: np.ndarray
    qdot: np.ndarray
    valid: bool
    reason: ArmTargetHoldReason


class DualArmController:
    """Keep alignment, QP, and HOLD state independently for both arms."""

    def __init__(
        self,
        model: Any | None = None,
        data: Any | None = None,
        *,
        left_solver: ArmQPSolver | None = None,
        right_solver: ArmQPSolver | None = None,
        left_alignment: SideAlignment | None = None,
        right_alignment: SideAlignment | None = None,
        period_ns: int = _PERIOD_NS,
        publisher: Any | None = None,
    ) -> None:
        if left_solver is None or right_solver is None:
            if model is None:
                raise ValueError("model or both side solvers are required")
            if mujoco is None:
                raise ImportError("mujoco is required for DualArmController")
            shared_data = data if data is not None else mujoco.MjData(model)
            left_solver = left_solver or ArmQPSolver(model, shared_data, side="left")
            right_solver = right_solver or ArmQPSolver(model, shared_data, side="right")
        self.left_solver = left_solver
        self.right_solver = right_solver
        self.left_alignment = left_alignment or SideAlignment()
        self.right_alignment = right_alignment or SideAlignment()
        self.period_ns = int(period_ns)
        if self.period_ns <= 0:
            raise ValueError("period_ns must be positive")
        self.publisher = publisher
        self.tracking_mailbox: LatestSample[TrackingFrame] = LatestSample()
        self.control_mailbox: LatestSample[ControlFrame] = LatestSample()
        self._tracking_generation = 0
        self._control_generation = 0
        self._tracking: TrackingFrame | None = None
        self._paused = False
        self._running = True
        self._sequence = 0
        self._last_tick_ns: int | None = None
        self.tick_count = 0
        self.left_q = np.asarray(self.left_solver.home, dtype=float).copy()
        self.right_q = np.asarray(self.right_solver.home, dtype=float).copy()
        self._left_qdot = np.zeros(7, dtype=float)
        self._right_qdot = np.zeros(7, dtype=float)

    @property
    def running(self) -> bool:
        return self._running

    def connect(self, node: ZenohNode) -> None:
        """Attach existing Zenoh ownership to tracking/control input and target output."""
        node.declare_latest_subscriber(TRACKING_KEY, decode_tracking, self.tracking_mailbox)
        node.declare_latest_subscriber(CONTROL_KEY, decode_control, self.control_mailbox)
        self.publisher = node.declare_publisher(ARM_TARGETS_KEY)

    def accept_tracking(self, frame: TrackingFrame | bytes | bytearray | memoryview) -> None:
        self._tracking = decode_tracking(frame) if isinstance(frame, (bytes, bytearray, memoryview)) else frame
        if not isinstance(self._tracking, TrackingFrame):
            raise TypeError("tracking frame must be TrackingFrame or encoded bytes")

    def accept_control(self, frame: ControlFrame | bytes | bytearray | memoryview) -> None:
        control = decode_control(frame) if isinstance(frame, (bytes, bytearray, memoryview)) else frame
        if not isinstance(control, ControlFrame):
            raise TypeError("control frame must be ControlFrame or encoded bytes")
        command = control.command
        if command is ControlCommand.START:
            self._paused = False
            self._running = True
        elif command is ControlCommand.PAUSE:
            self._paused = True
        elif command is ControlCommand.RESUME:
            self._paused = False
            self.left_alignment.realign()
            self.right_alignment.realign()
        elif command is ControlCommand.REALIGN:
            self.left_alignment.realign()
            self.right_alignment.realign()
        elif command is ControlCommand.RESET:
            self.left_alignment.reset()
            self.right_alignment.reset()
            self.left_solver.reset()
            self.right_solver.reset()
            self.left_q = np.asarray(self.left_solver.home, dtype=float).copy()
            self.right_q = np.asarray(self.right_solver.home, dtype=float).copy()
            self._left_qdot.fill(0.0)
            self._right_qdot.fill(0.0)
        elif command is ControlCommand.SHUTDOWN:
            self._running = False

    def _poll_mailboxes(self) -> None:
        sample = self.tracking_mailbox.take_new(self._tracking_generation)
        if sample is not None:
            self._tracking_generation, self._tracking = sample
        sample_control = self.control_mailbox.take_new(self._control_generation)
        if sample_control is not None:
            self._control_generation, control = sample_control
            self.accept_control(control)

    @staticmethod
    def _wrist(hand: np.ndarray) -> np.ndarray:
        if hand.shape != (26, 7):
            raise ValueError("hand must have shape (26, 7)")
        return _pose_matrix(hand[1])

    def _solve_side(
        self,
        solver: ArmQPSolver,
        alignment: SideAlignment,
        q: np.ndarray,
        qdot: np.ndarray,
        hand: np.ndarray,
        active: bool,
        epoch: int,
        timestamp_ns: int,
        now_ns: int,
        dt: float,
    ) -> _SideOutput:
        try:
            aligned: AlignedPose = alignment.accept(
                self._wrist(hand), active, epoch, timestamp_ns, now_ns=now_ns
            )
        except (TypeError, ValueError):
            return _SideOutput(q, np.zeros(7), False, ArmTargetHoldReason.INPUT_STALE)
        if not aligned.valid:
            return _SideOutput(q, np.zeros(7), False, _hold_reason(aligned.hold_reason))
        result = solver.solve(q, aligned.target_pose, dt)
        if not result.success:
            return _SideOutput(q, np.zeros(7), False, ArmTargetHoldReason.SOLVER_FAILURE)
        next_q = q + result.dq
        if not np.all(np.isfinite(next_q)):
            return _SideOutput(q, np.zeros(7), False, ArmTargetHoldReason.SOLVER_FAILURE)
        return _SideOutput(next_q, result.dq / dt, True, ArmTargetHoldReason.NONE)

    def tick(self, now_ns: int | None = None) -> ArmTargetFrame:
        self._poll_mailboxes()
        now = int(time.monotonic_ns() if now_ns is None else now_ns)
        dt = self.period_ns * 1.0e-9
        self._sequence += 1
        tracking = self._tracking
        epoch = int(tracking.tracking_epoch) if tracking is not None else 1
        source_timestamp = int(tracking.bridge_monotonic_ns) if tracking is not None else max(1, now)
        if self._paused:
            left = _SideOutput(self.left_q, np.zeros(7), False, ArmTargetHoldReason.PAUSED)
            right = _SideOutput(self.right_q, np.zeros(7), False, ArmTargetHoldReason.PAUSED)
        elif tracking is None:
            left = _SideOutput(self.left_q, np.zeros(7), False, ArmTargetHoldReason.DISCONNECTED)
            right = _SideOutput(self.right_q, np.zeros(7), False, ArmTargetHoldReason.DISCONNECTED)
        else:
            left = self._solve_side(
                self.left_solver, self.left_alignment, self.left_q, self._left_qdot,
                tracking.left_hand, tracking.left_active, tracking.tracking_epoch,
                tracking.bridge_monotonic_ns, now, dt,
            )
            right = self._solve_side(
                self.right_solver, self.right_alignment, self.right_q, self._right_qdot,
                tracking.right_hand, tracking.right_active, tracking.tracking_epoch,
                tracking.bridge_monotonic_ns, now, dt,
            )
        self.left_q, self._left_qdot = left.q.copy(), left.qdot.copy()
        self.right_q, self._right_qdot = right.q.copy(), right.qdot.copy()
        valid_mask = (1 if left.valid else 0) | (2 if right.valid else 0)
        frame = ArmTargetFrame(
            sequence=self._sequence,
            tracking_epoch=max(1, epoch),
            source_timestamp_ns=max(1, source_timestamp),
            control_timestamp_ns=max(1, now),
            valid_mask=valid_mask,
            left_hold_reason=left.reason,
            right_hold_reason=right.reason,
            left_q=tuple(self.left_q),
            right_q=tuple(self.right_q),
            left_qdot=tuple(self._left_qdot),
            right_qdot=tuple(self._right_qdot),
        )
        if self.publisher is not None:
            self.publisher.put(encode_arm_target(frame))
        self.tick_count += 1
        self._last_tick_ns = now
        return frame

    def run(
        self,
        ticks: int | None = None,
        *,
        clock: Callable[[], int] = time.monotonic_ns,
        sleep: Callable[[float], None] = time.sleep,
    ) -> list[ArmTargetFrame]:
        """Run on absolute 5 ms deadlines; a miss skips directly to the next period."""
        outputs: list[ArmTargetFrame] = []
        deadline = int(clock())
        count = 0
        while self._running and (ticks is None or count < int(ticks)):
            now = int(clock())
            if now < deadline:
                sleep((deadline - now) * 1.0e-9)
                now = int(clock())
            outputs.append(self.tick(now))
            count += 1
            deadline += self.period_ns
            if now >= deadline:
                deadline = now + self.period_ns
        return outputs


def _synthetic_xml() -> str:
    def chain(prefix: str, x: float) -> str:
        bodies = ""
        for index in range(7):
            site = f'<site name="{prefix}_wrist_target" pos="0.12 0 0" size="0.01"/>' if index == 6 else ""
            bodies = f'<body name="{prefix}_link{index}" pos="{x if index == 0 else 0.12} 0 0"><joint name="{prefix}_joint{index}" type="hinge" axis="0 0 1" range="-2 2"/><geom type="capsule" fromto="0 0 0 0.12 0 0" size="0.015"/>{site}{bodies}</body>'
        return bodies
    return f'<mujoco><option gravity="0 0 0"/><worldbody>{chain("l", -0.45)}{chain("r", 0.45)}</worldbody></mujoco>'


def build_synthetic_fixture() -> tuple[DualArmController, np.ndarray, np.ndarray]:
    """Build the small two-sided fixture used by focused tests and CLI self-test."""
    if mujoco is None:
        raise ImportError("mujoco is required for the synthetic fixture")
    model = mujoco.MjModel.from_xml_string(_synthetic_xml())
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    left_ids = tuple(range(7))
    right_ids = tuple(range(7, 14))
    left_solver = ArmQPSolver(model, data, site_name="l_wrist_target", joint_ids=left_ids)
    right_solver = ArmQPSolver(model, data, site_name="r_wrist_target", joint_ids=right_ids)
    left_pose = np.eye(4)
    right_pose = np.eye(4)
    left_pose[:3, :3] = data.site_xmat[left_solver.site_id].reshape(3, 3)
    left_pose[:3, 3] = data.site_xpos[left_solver.site_id]
    right_pose[:3, :3] = data.site_xmat[right_solver.site_id].reshape(3, 3)
    right_pose[:3, 3] = data.site_xpos[right_solver.site_id]
    controller = DualArmController(
        left_solver=left_solver,
        right_solver=right_solver,
        left_alignment=SideAlignment(neutral_robot=left_pose),
        right_alignment=SideAlignment(neutral_robot=right_pose),
    )
    return controller, left_pose, right_pose


def _synthetic_tracking(left_pose: np.ndarray, right_pose: np.ndarray, sequence: int, timestamp_ns: int) -> TrackingFrame:
    left = np.zeros((26, 7), dtype=np.float32)
    right = np.zeros((26, 7), dtype=np.float32)
    left[:, 6] = 1.0
    right[:, 6] = 1.0
    for hand, pose in ((left, left_pose), (right, right_pose)):
        hand[1, :3] = pose[:3, 3]
    return TrackingFrame(
        sequence=sequence,
        tracking_epoch=1,
        source_timestamp_ns=timestamp_ns,
        bridge_monotonic_ns=timestamp_ns,
        left_active=True,
        right_active=True,
        head_valid=True,
        left_scale=1.0,
        right_scale=1.0,
        head_pose=np.array((0, 0, 1.6, 0, 0, 0, 1), dtype=np.float32),
        left_hand=left,
        right_hand=right,
    )


def _verified_model(model_path: Path, manifest_path: Path, urdf_path: Path) -> Any:
    if mujoco is None:
        raise RuntimeError("mujoco is required for production artifact loading")
    verified = verify_artifacts(manifest_path, urdf_path)
    if model_path.resolve() != verified.arm_model.resolve():
        raise ArtifactError("production IK must load the manifest arm_ik.xml artifact")
    model = mujoco.MjModel.from_xml_path(str(model_path))
    if model.nq != 14 or model.nv != 14:
        raise ArtifactError("arm_ik.xml must have exactly 14 DoF")
    return model


def _self_test(ticks: int) -> int:
    controller, left_pose, right_pose = build_synthetic_fixture()
    start = time.monotonic_ns()
    outputs: list[ArmTargetFrame] = []
    for sequence in range(1, int(ticks) + 1):
        timestamp = start + sequence * _PERIOD_NS
        controller.accept_tracking(_synthetic_tracking(left_pose, right_pose, sequence, timestamp))
        outputs.append(controller.tick(timestamp))
    finite = sum(
        int(np.all(np.isfinite(frame.left_q + frame.right_q + frame.left_qdot + frame.right_qdot)))
        for frame in outputs
    )
    failures = sum(
        int(frame.left_hold_reason is ArmTargetHoldReason.SOLVER_FAILURE or frame.right_hold_reason is ArmTargetHoldReason.SOLVER_FAILURE)
        for frame in outputs
    )
    rate = (len(outputs) - 1) / ((outputs[-1].control_timestamp_ns - outputs[0].control_timestamp_ns) * 1.0e-9) if len(outputs) > 1 else 0.0
    print(f"self-test: ticks={len(outputs)} finite={finite} solver_failures={failures} rate_hz={rate:.2f}")
    return 0 if len(outputs) == int(ticks) and finite == len(outputs) and failures == 0 else 1


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--ticks", type=int, default=400)
    parser.add_argument("--model", type=Path, default=None)
    parser.add_argument("--manifest", type=Path, default=None)
    parser.add_argument("--urdf", type=Path, default=None)
    args = parser.parse_args(argv)
    if args.self_test:
        if args.ticks <= 0:
            parser.error("--ticks must be positive")
        return _self_test(args.ticks)
    if args.model is None or args.manifest is None or args.urdf is None:
        print("production IK requires --model, --manifest, and --urdf", file=sys.stderr)
        return 2
    try:
        _verified_model(args.model, args.manifest, args.urdf)
    except (ArtifactError, FileNotFoundError, OSError, RuntimeError, ValueError) as exc:
        print(f"artifact validation failed: {exc}", file=sys.stderr)
        return 2
    print("validated arm_ik.xml; runtime transport setup is required")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = ["DualArmController", "build_synthetic_fixture", "main"]
