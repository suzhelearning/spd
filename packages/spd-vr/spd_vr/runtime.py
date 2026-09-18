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
    """Write one deterministic schema-v1 episode without live device input."""
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
    last_hand_key: tuple[int, int] | None = None
    last_arm = -1
    last_camera = -1
    try:
        for index in range(ticks):
            sim_ns = plant.sim_time_ns
            now_ns = max(1, time.monotonic_ns())
            arm_sequence = index + 1
            plant.submit_arm_target(
                _arm_frame(plant, arm_sequence, 1, MOCK_EPOCH_NS + sim_ns + 1),
                now_ns=now_ns,
            )
            hand_due = index % max(1, plant.physics_hz // 60) == 0
            if hand_due:
                hand_sequence += 1
                plant.submit_tracking(_mock_hand_frame(sim_ns, hand_sequence, 1), now_ns=now_ns)
            step = plant.physics_tick(now_ns)
            if step.tick % max(1, plant.physics_hz // 120) == 0:
                qpos = np.asarray(plant.data.qpos[:54], dtype=np.float32)
                wire_qpos = np.concatenate((qpos[:7], qpos[27:34], qpos[7:27], qpos[34:54]))
                received_ns = time.monotonic_ns()
                recorder.append_arm_qpos(received_ns, np.concatenate((qpos[:7], qpos[27:34])))
                recorder.append_command(
                    received_ns,
                    wire_qpos,
                    sequence=arm_sequence,
                    stamp_utc_ns=time.time_ns(),
                    ready_mask=3 if step.hand_valid_mask == 3 else 1,
                    session_id="synthetic-runtime",
                    applied_sim_time_ns=step.sim_time_ns,
                    hold_mask=0 if step.hand_valid_mask == 3 else 6,
                )
                last_arm = step.sim_time_ns
            if hand_due and step.hand_valid_mask == 3:
                key = (1, hand_sequence)
                if key != last_hand_key:
                    qpos = np.asarray(plant.data.qpos[:54], dtype=np.float32)
                    recorder.append_hand_qpos(
                        time.monotonic_ns(),
                        np.concatenate((qpos[7:27], qpos[34:54])),
                        both_fresh=True,
                    )
                    last_hand_key = key
            if step.sim_time_ns > last_camera and step.tick % max(1, plant.physics_hz // 30) == 0:
                recorder.append_cameras(
                    camera.capture(step.sim_time_ns),
                    available_timestamp_ns=time.monotonic_ns(),
                )
                last_camera = step.sim_time_ns
        qpos = np.asarray(plant.data.qpos[:54], dtype=np.float32)
        if last_arm < 0:
            recorder.append_arm_qpos(time.monotonic_ns(), np.concatenate((qpos[:7], qpos[27:34])))
            recorder.append_command(
                time.monotonic_ns(),
                np.concatenate((qpos[:7], qpos[27:34], qpos[7:27], qpos[34:54])),
                sequence=max(1, arm_sequence),
                stamp_utc_ns=time.time_ns(),
                ready_mask=3 if last_hand_key is not None else 1,
                session_id="synthetic-runtime",
                applied_sim_time_ns=plant.sim_time_ns,
                hold_mask=0 if last_hand_key is not None else 6,
            )
        if last_hand_key is None:
            recorder.append_hand_qpos(
                time.monotonic_ns(),
                np.concatenate((qpos[7:27], qpos[34:54])),
                both_fresh=True,
            )
        if last_camera < 0:
            recorder.append_cameras(camera.capture(plant.sim_time_ns), available_timestamp_ns=time.monotonic_ns())
        return recorder.finish_episode(success=True)
    finally:
        if recorder.is_recording:
            recorder.abort_episode("runtime_interrupted")
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
