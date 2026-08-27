"""Create the exact 30 Hz training view from raw episode streams."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import tempfile
from typing import Any

import h5py
import numpy as np

from .camera import CAMERA_NAMES

MAX_IMAGE_AGE_NS = 50_000_000


class AlignmentError(ValueError):
    """Raised when a camera has no valid past frame for a sample."""


def _past_indices(camera_timestamps: np.ndarray, sample_timestamps: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    indices = np.searchsorted(camera_timestamps, sample_timestamps, side="right") - 1
    if np.any(indices < 0):
        raise AlignmentError("camera has no frame at or before a robot sample")
    selected = camera_timestamps[indices]
    ages = sample_timestamps - selected
    if np.any(ages < 0) or np.any(ages > MAX_IMAGE_AGE_NS):
        raise AlignmentError("camera frame age is outside the 50 ms contract")
    return indices.astype(np.int64), ages.astype(np.uint64)


def align_episode(input_h5: str | Path, output_h5: str | Path) -> dict[str, Any]:
    input_h5 = Path(input_h5)
    output_h5 = Path(output_h5)
    with h5py.File(input_h5, "r") as source:
        robot_ts = source["timestamps/robot_ns"][:]
        if robot_ts.ndim != 1 or robot_ts.size == 0:
            raise AlignmentError("raw robot stream is empty")
        # Raw robot/action are recorded at 60 Hz; even sequence numbers form
        # the exact 30 Hz grid. No interpolation is performed.
        selected_robot = np.arange(0, robot_ts.size, 2, dtype=np.int64)
        sample_ts = robot_ts[selected_robot]
        camera_indices: dict[str, np.ndarray] = {}
        camera_ages: dict[str, np.ndarray] = {}
        for camera in CAMERA_NAMES:
            timestamps = source[f"timestamps/cameras/{camera}_ns"][:]
            if timestamps.size == 0:
                raise AlignmentError(f"camera {camera} has no frames")
            camera_indices[camera], camera_ages[camera] = _past_indices(timestamps, sample_ts)

        output_h5.parent.mkdir(parents=True, exist_ok=True)
        fd, staging_name = tempfile.mkstemp(
            prefix=f".{output_h5.name}.staging-", dir=output_h5.parent
        )
        os.close(fd)
        staging = Path(staging_name)
        staging.unlink()
        try:
            with h5py.File(staging, "w") as target:
                target.attrs["schema_version"] = 1
                target.attrs["source"] = str(input_h5)
                target.create_dataset("timestamps/sample_ns", data=sample_ts, compression="gzip")
                for name in ("observations/qpos", "observations/qvel", "actions/qpos_target", "validity/arm_mask", "validity/hand_mask"):
                    parent, _, leaf = name.rpartition("/")
                    target.require_group(parent).create_dataset(leaf, data=source[name][selected_robot], compression="gzip")
                for camera in CAMERA_NAMES:
                    parent = target.require_group(f"cameras/{camera}")
                    parent.create_dataset("rgb", data=source[f"cameras/{camera}/rgb"][camera_indices[camera]], chunks=(1, 168, 224, 3), compression="gzip")
                    parent.create_dataset("segmentation", data=source[f"cameras/{camera}/segmentation"][camera_indices[camera]], chunks=(1, 168, 224, 2), compression="gzip")
                    parent.create_dataset("age_ns", data=camera_ages[camera], compression="gzip")
                    parent.create_dataset("source_timestamp_ns", data=source[f"timestamps/cameras/{camera}_ns"][:][camera_indices[camera]], compression="gzip")
                target.flush()
            output_h5.unlink(missing_ok=True)
            staging.replace(output_h5)
        except Exception:
            staging.unlink(missing_ok=True)
            raise
    return {"samples": int(sample_ts.size), "cameras": list(CAMERA_NAMES), "max_age_ms": 50.0}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_h5", type=Path)
    parser.add_argument("output_h5", type=Path)
    args = parser.parse_args(argv)
    print(align_episode(args.input_h5, args.output_h5))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
