"""Hardware-free episode runtime for the canonical Python plant."""

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
from .recorder import EpisodeRecorder
from .viewer import PlantController


MOCK_EPOCH_NS = 1_700_000_000_000_000_000


class _TickPacer:
    """Pace a caller against an absolute monotonic deadline."""

    def __init__(self, period_ns: int, clock_ns: Any | None = None, sleep: Any | None = None) -> None:
        if int(period_ns) <= 0:
            raise ValueError("period_ns must be positive")
        self.period_ns = int(period_ns)
        self.clock_ns = time.monotonic_ns if clock_ns is None else clock_ns
        self.sleep = time.sleep if sleep is None else sleep
        self.deadline_ns: int | None = None

    def reset(self) -> None:
        self.deadline_ns = None

    def wait(self) -> int:
        now = int(self.clock_ns())
        if self.deadline_ns is None:
            self.deadline_ns = now
            return now
        if now < self.deadline_ns:
            self.sleep((self.deadline_ns - now) * 1e-9)
            now = int(self.clock_ns())
        self.deadline_ns += self.period_ns
        if self.deadline_ns <= now:
            self.deadline_ns = now + self.period_ns
        return now


# Kept as a narrow name for existing hardware-free callers; it has no device
# or network behavior.
_LiveTickPacer = _TickPacer


@dataclass
class _MockRetargeter:
    """Deterministic hand target used by explicit mock episodes."""

    def reset_filter(self, *_: Any) -> None:
        return None

    def retarget(self, frame: Any) -> Any:
        return type(
            "MockTarget",
            (),
            {
                "left_qpos": np.zeros(20, dtype=np.float64),
                "right_qpos": np.zeros(20, dtype=np.float64),
                "left_valid": bool(frame.left_active),
                "right_valid": bool(frame.right_active),
                "left_hold_reason": "none" if frame.left_active else "inactive",
                "right_hold_reason": "none" if frame.right_active else "inactive",
            },
        )()


def _mock_hand_frame(sim_time_ns: int, sequence: int, epoch: int) -> Any:
    from .pico_hands import PicoHandFrame

    left = np.zeros((26, 7), dtype=np.float32)
    right = np.zeros((26, 7), dtype=np.float32)
    left[..., 6] = 1.0
    right[..., 6] = 1.0
    timestamp_ns = MOCK_EPOCH_NS + int(sim_time_ns)
    return PicoHandFrame(left, right, True, True, epoch, sequence, timestamp_ns)


def _arm_frame(plant: PlantController, sequence: int, epoch: int, timestamp_ns: int) -> ArmTargetFrame:
    left = tuple(float(value) for value in plant._home[:7])
    right = tuple(float(value) for value in plant._home[27:34])
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
    scene: str = "hardware_free",
    task: str = "mock",
    duration_s: float,
    seed: int = 0,
    headless: bool = True,
    mock: bool = False,
) -> Path:
    """Write one deterministic episode without any live/device input path."""
    del seed, headless
    if not math.isfinite(float(duration_s)) or duration_s <= 0.0:
        raise ValueError("duration_s must be positive")
    if not mock:
        raise RuntimeError("runtime requires explicit --mock for hardware-free episodes")
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    plant = PlantController.synthetic_fixture(hand_retargeter=_MockRetargeter())
    camera = SyntheticCameraProvider()
    recorder = EpisodeRecorder(output)
    recorder.start_episode(1, {"scene": scene, "task": task, "synthetic": True})
    ticks = max(1, int(round(float(duration_s) * plant.physics_hz)))
    hand_sequence = 0
    arm_sequence = 0
    last_hand = -1
    last_camera = -1
    last_robot = -1
    try:
        for index in range(ticks):
            sim_ns = plant.sim_time_ns
            now_ns = max(1, time.monotonic_ns())
            arm_sequence += 1
            plant.submit_arm_target(
                _arm_frame(plant, arm_sequence, 1, MOCK_EPOCH_NS + sim_ns + 1),
                now_ns=now_ns,
            )
            if index % max(1, plant.physics_hz // plant.hand_target_hz) == 0:
                hand_sequence += 1
                hand = _mock_hand_frame(sim_ns, hand_sequence, 1)
                plant.submit_tracking(hand, now_ns=now_ns)
                recorder.append_hands(
                    hand.timestamp_ns,
                    hand.sequence_id,
                    hand.tracking_epoch,
                    hand.left_hand,
                    hand.right_hand,
                    left_active=True,
                    right_active=True,
                )
            step = plant.physics_tick(now_ns)
            if step.tick % max(1, plant.physics_hz // 60) == 0:
                recorder.append_robot(
                    step.sim_time_ns,
                    plant.data.qpos[:54],
                    plant.data.qvel[:54],
                    plant.data.ctrl[:54],
                    arm_valid_mask=step.arm_valid_mask,
                    hand_valid_mask=step.hand_valid_mask,
                )
                last_robot = step.sim_time_ns
            if step.sim_time_ns > last_camera and step.tick % max(1, plant.physics_hz // 30) == 0:
                frames = camera.capture(step.sim_time_ns)
                recorder.append_cameras(frames)
                last_camera = step.sim_time_ns
            last_hand = hand_sequence
        # Ensure every required stream has one sample for very short episodes.
        if last_robot < 0:
            recorder.append_robot(
                plant.sim_time_ns,
                plant.data.qpos[:54],
                plant.data.qvel[:54],
                plant.data.ctrl[:54],
            )
        if last_hand < 1:
            hand = _mock_hand_frame(plant.sim_time_ns, 1, 1)
            recorder.append_hands(hand.timestamp_ns, 1, 1, hand.left_hand, hand.right_hand, left_active=True, right_active=True)
        if last_camera < 0:
            recorder.append_cameras(camera.capture(plant.sim_time_ns))
        return recorder.finish_episode()
    finally:
        plant.close()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--scene", default="hardware_free")
    parser.add_argument("--task", default="mock")
    parser.add_argument("--duration", type=float, default=1.0)
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
    print(json.dumps({"episode": str(episode), "synthetic": True}, sort_keys=True))
    return 0


__all__ = ["main", "run_runtime"]

if __name__ == "__main__":
    raise SystemExit(main())
