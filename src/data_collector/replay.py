"""Validate schema-v2 episodes and independently restore every recorded scene state."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import h5py
import mujoco
import numpy as np

from data_collector.trajectory import load_model, restore_frame


class ReplayError(ValueError):
    """A restored state disagrees with its recorded scene observations."""


def replay_episode(
    path: str | Path, *, tolerance: float = 1e-6, expected_model_sha256: str | None = None,
) -> dict[str, Any]:
    """Restore all frames from the embedded model; no original assets, renderer or mj_step."""
    from data_collector.recorder import validate_episode_path

    if not np.isfinite(tolerance) or tolerance < 0:
        raise ReplayError("replay tolerance must be finite and nonnegative")
    validation = validate_episode_path(path)
    episode_path = Path(validation["episode_path"])
    maxima = {
        "max_robot_qpos_error": 0.0, "max_robot_qvel_error": 0.0,
        "max_object_position_error": 0.0, "max_object_quaternion_error": 0.0,
    }
    with h5py.File(episode_path, "r") as handle:
        metadata = json.loads(handle["model/metadata"].asstr()[()])
        model = load_model(
            handle["model/mjb"][:].tobytes(), metadata, expected_model_sha256=expected_model_sha256,
        )
        data = mujoco.MjData(model)
        qpos_indices = np.asarray(metadata["robot_qpos_indices"], dtype=np.intp)
        qvel_indices = np.asarray(metadata["robot_qvel_indices"], dtype=np.intp)
        object_ids = np.asarray(metadata["object_body_ids"], dtype=np.intp)
        trajectory = handle["trajectory"]
        frame_count = int(trajectory["tick"].shape[0])
        for index in range(frame_count):
            frame = {name: trajectory[name][index] for name in metadata["fields"]}
            restore_frame(model, data, frame, metadata)
            errors = {
                "max_robot_qpos_error": float(np.max(np.abs(data.qpos[qpos_indices] - frame["robot_qpos"]))),
                "max_robot_qvel_error": float(np.max(np.abs(data.qvel[qvel_indices] - frame["robot_qvel"]))),
            }
            if len(object_ids):
                pose = frame["object_pose"]
                errors["max_object_position_error"] = float(np.max(np.abs(data.xpos[object_ids] - pose[:, :3])))
                restored_quat = data.xquat[object_ids]
                # q and -q represent the same world orientation.
                quaternion_error = np.minimum(
                    np.max(np.abs(restored_quat - pose[:, 3:]), axis=1),
                    np.max(np.abs(restored_quat + pose[:, 3:]), axis=1),
                )
                errors["max_object_quaternion_error"] = float(np.max(quaternion_error))
            for key, error in errors.items():
                if not np.isfinite(error) or error > tolerance:
                    raise ReplayError(f"frame {index}: {key}={error:g} exceeds tolerance {tolerance:g}")
                maxima[key] = max(maxima[key], error)
    return {
        **validation, "frames": frame_count, "restored_frames": frame_count,
        "restoration_valid": True, "tolerance": tolerance, **maxima,
        "max_object_pose_error": max(maxima["max_object_position_error"], maxima["max_object_quaternion_error"]),
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", type=Path)
    parser.add_argument("--tolerance", type=float, default=1e-6)
    parser.add_argument("--expected-model-sha256", help="Require this exact compiled-model SHA-256")
    args = parser.parse_args(argv)
    print(json.dumps(replay_episode(
        args.path, tolerance=args.tolerance, expected_model_sha256=args.expected_model_sha256,
    ), sort_keys=True))
    return 0


__all__ = ["ReplayError", "main", "replay_episode"]

if __name__ == "__main__":
    raise SystemExit(main())
