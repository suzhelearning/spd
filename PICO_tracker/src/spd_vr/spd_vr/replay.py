"""Replay and verify deterministic SPD VR episode streams."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import h5py
import numpy as np

from .camera import CAMERA_NAMES
from .recorder import validate_episode_path


class ReplayError(ValueError):
    pass


def _episode_dir(path: str | Path) -> Path:
    path = Path(path)
    if (path / "episode.hdf5").is_file():
        return path
    if path.is_file() and path.name == "episode.hdf5":
        return path.parent
    if (path / "episodes").is_dir():
        candidates = sorted(item for item in (path / "episodes").iterdir() if (item / "episode.hdf5").is_file())
        if len(candidates) == 1:
            return candidates[0]
    raise ReplayError(f"cannot resolve one episode from {path}")


def replay_episode(path: str | Path, *, simulator: Any | None = None, tolerance: float = 1e-6) -> dict[str, Any]:
    directory = _episode_dir(path)
    validation = validate_episode_path(directory)
    with h5py.File(directory / "episode.hdf5", "r") as handle:
        timestamps = handle["timestamps/robot_ns"][:]
        actions = handle["actions/qpos_target"][:]
        observations = handle["observations/qpos"][:]
        if timestamps.size and np.any(np.diff(timestamps.astype(np.int64)) <= 0):
            raise ReplayError("robot timestamps are not strictly increasing")
        if actions.shape != observations.shape or actions.shape[1:] != (54,):
            raise ReplayError("action/observation trajectory must be [N,54]")
        for camera in CAMERA_NAMES:
            camera_ts = handle[f"timestamps/cameras/{camera}_ns"][:]
            if camera_ts.size and np.any(np.diff(camera_ts.astype(np.int64)) <= 0):
                raise ReplayError(f"camera timestamp rollback: {camera}")
        if simulator is not None:
            if hasattr(simulator, "reset_for_replay"):
                simulator.reset_for_replay(json.loads((directory / "manifest.json").read_text()))
            mismatches = 0
            for action, expected in zip(actions, observations):
                if hasattr(simulator, "joints") and hasattr(simulator, "_actuator_ids"):
                    for entry in simulator.joints:
                        simulator.data.ctrl[simulator._actuator_ids[entry.actuator]] = action[entry.index]
                else:
                    simulator.data.ctrl[:] = action
                simulator._mujoco.mj_step(simulator.model, simulator.data)
                actual = (
                    simulator._manifest_qpos()
                    if hasattr(simulator, "_manifest_qpos")
                    else np.asarray(simulator.data.qpos)
                )
                if not np.allclose(actual, expected, atol=tolerance, rtol=0.0):
                    mismatches += 1
            if mismatches:
                raise ReplayError(f"replay qpos mismatch in {mismatches} frames")
    return {
        "valid": True,
        "episode_dir": str(directory),
        "robot_frames": int(timestamps.size),
        "camera_frames": validation["camera_frames"],
        "replayed": simulator is not None,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", type=Path)
    parser.add_argument("--headless", action="store_true")
    args = parser.parse_args(argv)
    print(json.dumps(replay_episode(args.path), sort_keys=True))
    return 0


__all__ = ["ReplayError", "main", "replay_episode"]

if __name__ == "__main__":
    raise SystemExit(main())
