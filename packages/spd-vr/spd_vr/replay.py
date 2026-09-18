"""Validate and inspect schema-v1 SPD episodes."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import h5py

from .recorder import validate_episode_path


class ReplayError(ValueError):
    pass


def replay_episode(path: str | Path, *, simulator: Any | None = None, tolerance: float = 1e-6) -> dict[str, Any]:
    """Validate one episode; schema-v1 intentionally has no action trajectory to replay."""
    del simulator, tolerance
    validation = validate_episode_path(path)
    episode_path = Path(validation["episode_path"])
    with h5py.File(episode_path, "r") as handle:
        arms = handle["observations/arms/qpos"]
        hands = handle["observations/hands/qpos"]
        camera_names = tuple(validation["camera_names"])
        return {
            "valid": True,
            "episode_path": str(episode_path),
            "schema_version": validation["schema_version"],
            "task": validation["task"],
            "success": bool(handle.attrs["success"]),
            "arm_frames": int(arms.shape[0]),
            "hand_frames": int(hands.shape[0]),
            "camera_frames": {
                name: int(handle[f"images/{name}/timestamp_ns"].shape[0]) for name in camera_names
            },
        }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", type=Path)
    args = parser.parse_args(argv)
    print(json.dumps(replay_episode(args.path), sort_keys=True))
    return 0


__all__ = ["ReplayError", "main", "replay_episode"]

if __name__ == "__main__":
    raise SystemExit(main())
