"""Headless/live-friendly SPD VR runtime wiring one simulator and recorder."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
from pathlib import Path
import time
from typing import Any

import numpy as np

from .arm_target_protocol import ArmTargetFrame, ArmTargetHoldReason
from .camera import SyntheticCameraProvider
from .episode import EpisodeCommandType, EpisodeController
from .recorder import EpisodeRecorder
from .scenes.model_scene import write_scene_model
from .scenes.registry import get_task
from .simulator import UnifiedSimulator


@dataclass
class _MockRetargeter:
    """Deterministic zero-cost hand retargeter used only by ``--mock``."""

    def reset_filter(self, *_: Any) -> None:
        return None

    def retarget(self, frame: Any) -> Any:
        return type("MockTarget", (), {
            "left_qpos": np.zeros(20, dtype=np.float64),
            "right_qpos": np.zeros(20, dtype=np.float64),
            "left_valid": bool(frame.left_active),
            "right_valid": bool(frame.right_active),
            "left_hold_reason": "none" if frame.left_active else "inactive",
            "right_hold_reason": "none" if frame.right_active else "inactive",
        })()


def _mock_hand_frame(timestamp_ns: int, sequence: int, epoch: int) -> Any:
    from .pico_hands import PicoHandFrame
    left = np.zeros((26, 7), dtype=np.float32)
    right = np.zeros((26, 7), dtype=np.float32)
    left[..., 6] = 1.0
    right[..., 6] = 1.0
    for joint in range(26):
        left[joint, :3] = (0.01 * joint, 0.001 * joint, 0.002 * joint)
        right[joint, :3] = (-0.01 * joint, 0.001 * joint, 0.002 * joint)
    return PicoHandFrame(left, right, True, True, epoch, sequence, timestamp_ns)


def _arm_frame(simulator: UnifiedSimulator, sequence: int, epoch: int, timestamp_ns: int) -> ArmTargetFrame:
    left = tuple(float((entry.range[0] + entry.range[1]) * 0.5) for entry in simulator.joints if entry.side == "left" and entry.group == "arm")
    right = tuple(float((entry.range[0] + entry.range[1]) * 0.5) for entry in simulator.joints if entry.side == "right" and entry.group == "arm")
    return ArmTargetFrame(
        sequence=sequence,
        tracking_epoch=epoch,
        source_timestamp_ns=timestamp_ns,
        control_timestamp_ns=timestamp_ns,
        valid_mask=3,
        hold_reason=ArmTargetHoldReason.NONE,
        left_q=left,
        right_q=right,
        left_qdot=(0.0,) * 7,
        right_qdot=(0.0,) * 7,
    )


def run_runtime(
    *,
    output: str | Path,
    scene: str,
    task: str,
    duration_s: float,
    seed: int = 0,
    headless: bool = True,
    mock: bool = True,
) -> Path:
    if duration_s <= 0.0:
        raise ValueError("duration_s must be positive")
    if not mock:
        raise RuntimeError(
            "spd_vr live execution is gated until the forthcoming APK publishes "
            "the PicoHands contract; use --mock for the simulator pilot"
        )
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    task_spec = get_task(scene, task)
    scene_result = task_spec.reset(seed)
    base_model = Path(__file__).resolve().parents[1] / "generated/tianji_wuji2_spd.xml"
    model_path = output / "tianji_wuji2_spd_scene.xml"
    write_scene_model(base_model, scene_result, model_path)
    manifest_path = Path(__file__).resolve().parents[1] / "generated/joint_manifest.yaml"
    recorder = EpisodeRecorder(output)
    provider = SyntheticCameraProvider() if headless else SyntheticCameraProvider()
    simulator = UnifiedSimulator(
        model_path=model_path,
        manifest_path=manifest_path,
        camera_provider=provider,
        recorder=None,
        hand_retargeter=_MockRetargeter() if mock else None,
    )
    simulator.set_task_object_body_names({item.name for item in scene_result.objects})
    controller = EpisodeController(simulator, task_spec, seed=seed, recorder=recorder)
    controller.enqueue(EpisodeCommandType.START)
    events = controller.process_all()
    if controller.state.value != "RECORDING":
        simulator.close()
        raise RuntimeError(f"episode failed to start: {events}")
    hand_sequence = 0
    arm_sequence = 0
    epoch = 1
    start_ns = time.monotonic_ns()
    robot_period = 8
    hand_period = 8
    last_camera_timestamp = -1
    def record_camera_results() -> None:
        nonlocal last_camera_timestamp
        for frames in simulator.drain_camera_results():
            timestamp = frames["top"].timestamp_ns
            if timestamp > last_camera_timestamp:
                recorder.append_cameras(frames)
                last_camera_timestamp = timestamp
    try:
        ticks = int(round(duration_s * simulator.physics_hz))
        for tick in range(ticks):
            sim_ns = int(round((tick + 1) * 1_000_000_000 / simulator.physics_hz))
            now_ns = start_ns + sim_ns
            if tick % 2 == 0:
                arm_sequence += 1
                simulator.on_arm_target(_arm_frame(simulator, arm_sequence, epoch, now_ns))
            if tick % hand_period == 0:
                hand_sequence += 1
                simulator.on_pico_hands(_mock_hand_frame(now_ns, hand_sequence, epoch))
            step = simulator.step()
            if tick % robot_period == 0:
                recorder.append_robot(
                    sim_ns,
                    simulator._manifest_qpos(),
                    simulator._manifest_qvel(),
                    simulator._manifest_ctrl(),
                    arm_valid_mask=step.arm_valid_mask,
                    hand_valid_mask=(1 if step.hand_left_valid else 0) | (2 if step.hand_right_valid else 0),
                )
                recorder.append_contacts(sim_ns, {"hand_object": False})
                if tick % hand_period == 0:
                    hand = _mock_hand_frame(now_ns, hand_sequence, epoch)
                    recorder.append_hands(
                        sim_ns,
                        hand.sequence_id,
                        hand.tracking_epoch,
                        hand.left_hand,
                        hand.right_hand,
                        left_active=hand.left_active,
                        right_active=hand.right_active,
                        left_scale=hand.left_scale,
                        right_scale=hand.right_scale,
                    )
            record_camera_results()
        # Allow the non-blocking camera worker to finish queued 30 Hz renders.
        deadline = time.time() + max(1.0, duration_s * 0.1)
        while time.time() < deadline:
            record_camera_results()
            if simulator._camera_queue is None or simulator._camera_queue.empty():
                break
            time.sleep(0.005)
        controller.enqueue(EpisodeCommandType.FINISH)
        finish_events = controller.process_all()
        if controller.state.value != "IDLE":
            raise RuntimeError(f"episode failed to finish: {finish_events}; {controller.last_error}")
        return recorder.output_root / "episodes" / str(controller.episode_id)
    finally:
        simulator.close()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--scene", default="jenga")
    parser.add_argument("--task", default="handover_lr")
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--mock", action="store_true")
    args = parser.parse_args(argv)
    episode = run_runtime(
        output=args.output,
        scene=args.scene,
        task=args.task,
        duration_s=args.duration,
        seed=args.seed,
        headless=args.headless,
        mock=args.mock,
    )
    print(json.dumps({"episode": str(episode)}, sort_keys=True))
    return 0


__all__ = ["main", "run_runtime"]

if __name__ == "__main__":
    raise SystemExit(main())
