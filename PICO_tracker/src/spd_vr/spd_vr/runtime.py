"""Headless/live-friendly SPD VR runtime wiring one simulator and recorder."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
import time
from typing import Any

import numpy as np

from .arm_target_protocol import ArmTargetFrame, ArmTargetHoldReason
from .camera import SyntheticCameraProvider
from .episode import EpisodeCommandType, EpisodeController
from .recorder import EpisodeRecorder
from .ros_input import LiveInputMailbox
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
        left_hold_reason=ArmTargetHoldReason.NONE,
        right_hold_reason=ArmTargetHoldReason.NONE,
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
    mock: bool = False,
    hands_topic: str = "/pico/hands",
    pause_topic: str = "/spd_vr/pause",
    arm_bind_host: str = "127.0.0.1",
    arm_bind_port: int = 15100,
    wrist_position_scale: float = 1.0,
    wrist_stable_frames: int = 10,
    wrist_max_position_step_m: float = 0.02,
    wrist_max_orientation_step_rad: float = 0.15,
) -> Path:
    if not math.isfinite(float(duration_s)) or duration_s <= 0.0:
        raise ValueError("duration_s must be positive")
    for name, value in (
        ("wrist_position_scale", wrist_position_scale),
        ("wrist_max_position_step_m", wrist_max_position_step_m),
        ("wrist_max_orientation_step_rad", wrist_max_orientation_step_rad),
    ):
        if not math.isfinite(float(value)) or float(value) <= 0.0:
            raise ValueError(f"{name} must be finite and positive")
    try:
        stable_frames = float(wrist_stable_frames)
    except (TypeError, ValueError) as exc:
        raise ValueError("wrist_stable_frames must be a non-zero positive integer") from exc
    if (
        not math.isfinite(stable_frames)
        or stable_frames <= 0.0
        or not stable_frames.is_integer()
    ):
        raise ValueError("wrist_stable_frames must be a non-zero positive integer")
    wrist_stable_frames = int(stable_frames)
    if not 1 <= int(arm_bind_port) <= 65535:
        raise ValueError("arm_bind_port must be between 1 and 65535")

    mailbox: LiveInputMailbox | None = None
    hand_retargeter: Any = _MockRetargeter()
    if not mock:
        # Keep ROS and the real Wuji dependency out of deterministic mock
        # imports.  LiveInputMailbox gives an explicit missing-dependency
        # error before any MuJoCo model is constructed.
        mailbox = LiveInputMailbox(hands_topic, pause_topic)
        try:
            from .retarget_pair import WujiRetargetPair

            config_dir = Path(__file__).resolve().parents[1] / "config"
            hand_retargeter = WujiRetargetPair(
                config_dir / "wuji2_pico_left.yaml",
                config_dir / "wuji2_pico_right.yaml",
            )
        except Exception:
            mailbox.close()
            raise

    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    task_spec = get_task(scene, task)
    scene_result = task_spec.reset(seed)
    base_model = Path(__file__).resolve().parents[1] / "generated/tianji_wuji2_spd.xml"
    model_path = output / "tianji_wuji2_spd_scene.xml"
    write_scene_model(base_model, scene_result, model_path)
    manifest_path = Path(__file__).resolve().parents[1] / "generated/joint_manifest.yaml"
    recorder = EpisodeRecorder(output)
    # The synthetic provider is retained for the recorder schema in both
    # modes; it is never used as a Wrist alignment input.
    provider = SyntheticCameraProvider()
    simulator: UnifiedSimulator | None = None
    try:
        simulator = UnifiedSimulator(
            model_path=model_path,
            manifest_path=manifest_path,
            camera_provider=provider,
            recorder=None,
            hand_retargeter=hand_retargeter,
        )
        simulator.set_task_object_body_names({item.name for item in scene_result.objects})
        if mailbox is not None:
            simulator.start_arm_udp(arm_bind_host, int(arm_bind_port))
        controller = EpisodeController(
            simulator,
            task_spec,
            seed=seed,
            recorder=recorder,
            run_metadata={
                "input": "xrobotoolkit_pico_hands",
                "left_site": "l_wrist_target",
                "right_site": "r_wrist_target",
                "wrist_position_scale": wrist_position_scale,
                "wrist_stable_frames": wrist_stable_frames,
                "wrist_max_position_step_m": wrist_max_position_step_m,
                "wrist_max_orientation_step_rad": wrist_max_orientation_step_rad,
            },
        )
        controller.enqueue(EpisodeCommandType.START)
        events = controller.process_all()
        if controller.state.value != "RECORDING":
            raise RuntimeError(f"episode failed to start: {events}")
        hand_sequence = 0
        arm_sequence = 0
        epoch = 1
        start_ns = time.monotonic_ns()
        robot_period_ticks = 8
        next_mock_arm_ns = 0
        next_mock_hand_ns = 0
        mock_arm_period_ns = int(
            round(1_000_000_000 / float(getattr(simulator, "arm_target_hz", 200)))
        )
        mock_hand_period_ns = int(
            round(1_000_000_000 / float(getattr(simulator, "hand_target_hz", 60)))
        )
        target_sim_ns = float(duration_s) * 1_000_000_000.0
        last_camera_timestamp = -1

        def record_camera_results() -> None:
            nonlocal last_camera_timestamp
            for frames in simulator.drain_camera_results():
                timestamp = frames["top"].timestamp_ns
                if timestamp > last_camera_timestamp:
                    recorder.append_cameras(frames)
                    last_camera_timestamp = timestamp

        def record_hand_frame(frame: Any) -> None:
            recorder.append_hands(
                frame.timestamp_ns,
                frame.sequence_id,
                frame.tracking_epoch,
                frame.left_hand,
                frame.right_hand,
                left_active=frame.left_active,
                right_active=frame.right_active,
                left_scale=frame.left_scale,
                right_scale=frame.right_scale,
            )

        while simulator.sim_time_ns < target_sim_ns:
            if mailbox is not None:
                mailbox.spin_once(0.0)
                for command in mailbox.take_episode_commands():
                    controller.enqueue(command)
            controller.process_all()
            if controller.state.value == "PAUSED":
                # A paused wall-clock interval does not consume simulated
                # time, and no input, physics, recording, or camera work is
                # performed on this path.
                time.sleep(0.001)
                continue
            if controller.state.value != "RECORDING":
                raise RuntimeError(
                    f"episode left recording state: {controller.state.value}; {controller.last_error}"
                )

            sim_ns = simulator.sim_time_ns
            if mailbox is None:
                if sim_ns >= next_mock_arm_ns:
                    arm_sequence += 1
                    now_ns = start_ns + sim_ns
                    simulator.on_arm_target(
                        _arm_frame(simulator, arm_sequence, epoch, now_ns)
                    )
                    while next_mock_arm_ns <= sim_ns:
                        next_mock_arm_ns += mock_arm_period_ns
                if sim_ns >= next_mock_hand_ns:
                    hand_sequence += 1
                    now_ns = start_ns + sim_ns
                    hand = _mock_hand_frame(now_ns, hand_sequence, epoch)
                    simulator.on_pico_hands(hand, now_ns=now_ns)
                    record_hand_frame(hand)
                    while next_mock_hand_ns <= sim_ns:
                        next_mock_hand_ns += mock_hand_period_ns
            else:
                latest = mailbox.take_latest_hands()
                if latest is not None:
                    from .pico_hands import PicoHandsInput

                    frame = PicoHandsInput(latest).frame
                    simulator.on_pico_hands(frame, now_ns=time.monotonic_ns())
                    record_hand_frame(frame)

            step = simulator.step()
            if (step.tick - 1) % robot_period_ticks == 0:
                recorder.append_robot(
                    step.sim_time_ns,
                    simulator._manifest_qpos(),
                    simulator._manifest_qvel(),
                    simulator._manifest_ctrl(),
                    arm_valid_mask=step.arm_valid_mask,
                    hand_valid_mask=(1 if step.hand_left_valid else 0)
                    | (2 if step.hand_right_valid else 0),
                )
                recorder.append_contacts(step.sim_time_ns, {"hand_object": False})
            record_camera_results()

        # Allow the non-blocking camera worker to finish queued 30 Hz renders.
        deadline = time.time() + max(1.0, float(duration_s) * 0.1)
        while time.time() < deadline:
            record_camera_results()
            camera_queue = getattr(simulator, "_camera_queue", None)
            if last_camera_timestamp >= 0 and (
                camera_queue is None or camera_queue.empty()
            ):
                break
            time.sleep(0.005)
        controller.enqueue(EpisodeCommandType.FINISH)
        finish_events = controller.process_all()
        if controller.state.value != "IDLE":
            raise RuntimeError(
                f"episode failed to finish: {finish_events}; {controller.last_error}"
            )
        return recorder.output_root / "episodes" / str(controller.episode_id)
    finally:
        if simulator is not None:
            simulator.close()
        if mailbox is not None:
            mailbox.close()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--scene", default="jenga")
    parser.add_argument("--task", default="handover_lr")
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--mock", action="store_true")
    parser.add_argument("--hands-topic", default="/pico/hands")
    parser.add_argument("--pause-topic", default="/spd_vr/pause")
    parser.add_argument("--arm-bind-host", default="127.0.0.1")
    parser.add_argument("--arm-bind-port", type=int, default=15100)
    parser.add_argument("--wrist-position-scale", type=float, default=1.0)
    parser.add_argument("--wrist-stable-frames", type=int, default=10)
    parser.add_argument("--wrist-max-position-step", type=float, default=0.02)
    parser.add_argument("--wrist-max-orientation-step", type=float, default=0.15)
    args = parser.parse_args(argv)
    episode = run_runtime(
        output=args.output,
        scene=args.scene,
        task=args.task,
        duration_s=args.duration,
        seed=args.seed,
        headless=args.headless,
        mock=args.mock,
        hands_topic=args.hands_topic,
        pause_topic=args.pause_topic,
        arm_bind_host=args.arm_bind_host,
        arm_bind_port=args.arm_bind_port,
        wrist_position_scale=args.wrist_position_scale,
        wrist_stable_frames=args.wrist_stable_frames,
        wrist_max_position_step_m=args.wrist_max_position_step,
        wrist_max_orientation_step_rad=args.wrist_max_orientation_step,
    )
    print(json.dumps({"episode": str(episode)}, sort_keys=True))
    return 0


__all__ = ["main", "run_runtime"]

if __name__ == "__main__":
    raise SystemExit(main())
