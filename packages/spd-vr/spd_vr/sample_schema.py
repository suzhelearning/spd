"""Validation for the versioned PICO dual-hand NPZ input contract."""

from __future__ import annotations

import argparse
import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Mapping

import numpy as np

JOINT_COUNT = 26
POSE_WIDTH = 7
REQUIRED_ARRAYS = (
    "timestamp_ns",
    "sequence_id",
    "left_active",
    "right_active",
    "left_hand",
    "right_hand",
)
EXPECTED_DTYPES = {
    "timestamp_ns": np.dtype("<u8"),
    "sequence_id": np.dtype("<u8"),
    "left_active": np.dtype("bool"),
    "right_active": np.dtype("bool"),
    "left_hand": np.dtype("<f4"),
    "right_hand": np.dtype("<f4"),
}
QUATERNION_NORM_TOLERANCE = 1e-2


class SampleValidationError(ValueError):
    """Raised when an NPZ sample violates the immutable input contract."""


@dataclass(frozen=True)
class SampleSummary:
    frames: int
    duration_s: float
    frequency_hz: float
    left_active_ratio: float
    right_active_ratio: float
    gap_p95_ms: float
    gap_p99_ms: float


def _fail(message: str) -> None:
    raise SampleValidationError(message)


def _require_array(data: Mapping[str, np.ndarray], name: str) -> np.ndarray:
    if name not in data:
        _fail(f"missing array: {name}")
    value = np.asarray(data[name])
    expected = EXPECTED_DTYPES[name]
    if value.dtype != expected:
        _fail(f"{name}: expected dtype {expected.name}, got {value.dtype.name}")
    return value


def _check_shape(name: str, value: np.ndarray, frames: int) -> None:
    expected = (frames,) if name in {"timestamp_ns", "sequence_id", "left_active", "right_active"} else (frames, JOINT_COUNT, POSE_WIDTH)
    if value.shape != expected:
        _fail(f"{name}: expected shape {expected}, got {value.shape}")


def _check_monotonic(name: str, value: np.ndarray) -> None:
    if value.size > 1 and not bool(np.all(np.diff(value.astype(np.uint64)) > 0)):
        _fail(f"{name}: values must be strictly increasing")


def _check_hand(name: str, value: np.ndarray, active: np.ndarray) -> None:
    if not bool(np.all(np.isfinite(value))):
        _fail(f"{name}: all pose values must be finite")
    active_values = value[active]
    if active_values.size == 0:
        return
    quaternions = active_values[..., 3:7]
    norms = np.linalg.norm(quaternions, axis=-1)
    if not bool(np.all(np.isfinite(norms))):
        _fail(f"{name}: active quaternion norms must be finite")
    if not bool(np.all(np.abs(norms - 1.0) <= QUATERNION_NORM_TOLERANCE)):
        _fail(
            f"{name}: active quaternion norm must be within "
            f"{QUATERNION_NORM_TOLERANCE:g} of one"
        )


def validate_arrays(data: Mapping[str, np.ndarray]) -> SampleSummary:
    """Validate already-loaded NPZ arrays and return deterministic statistics."""
    missing = [name for name in REQUIRED_ARRAYS if name not in data]
    if missing:
        _fail("missing arrays: " + ", ".join(missing))
    extra = sorted(set(data) - set(REQUIRED_ARRAYS))
    if extra:
        _fail("unexpected arrays: " + ", ".join(extra))

    arrays = {name: _require_array(data, name) for name in REQUIRED_ARRAYS}
    frames = int(arrays["timestamp_ns"].shape[0])
    if frames <= 0:
        _fail("sample must contain at least one frame")
    for name, value in arrays.items():
        _check_shape(name, value, frames)

    _check_monotonic("timestamp_ns", arrays["timestamp_ns"])
    _check_monotonic("sequence_id", arrays["sequence_id"])
    _check_hand("left_hand", arrays["left_hand"], arrays["left_active"])
    _check_hand("right_hand", arrays["right_hand"], arrays["right_active"])

    timestamps = arrays["timestamp_ns"].astype(np.float64)
    if frames > 1:
        gaps_ms = np.diff(timestamps) / 1e6
        duration_s = float((timestamps[-1] - timestamps[0]) / 1e9)
        frequency_hz = float((frames - 1) / duration_s) if duration_s > 0.0 else 0.0
        gap_p95_ms = float(np.percentile(gaps_ms, 95))
        gap_p99_ms = float(np.percentile(gaps_ms, 99))
    else:
        duration_s = 0.0
        frequency_hz = 0.0
        gap_p95_ms = 0.0
        gap_p99_ms = 0.0

    return SampleSummary(
        frames=frames,
        duration_s=duration_s,
        frequency_hz=frequency_hz,
        left_active_ratio=float(np.mean(arrays["left_active"])),
        right_active_ratio=float(np.mean(arrays["right_active"])),
        gap_p95_ms=gap_p95_ms,
        gap_p99_ms=gap_p99_ms,
    )


def validate_sample(path: str | Path) -> SampleSummary:
    """Load and validate an NPZ file without allowing object arrays."""
    sample_path = Path(path)
    if not sample_path.is_file():
        raise FileNotFoundError(f"pico hand sample not found: {path}")
    try:
        with np.load(sample_path, allow_pickle=False) as data:
            return validate_arrays({name: data[name] for name in data.files})
    except SampleValidationError:
        raise
    except (OSError, ValueError, EOFError) as exc:
        raise SampleValidationError(f"cannot read sample {path}: {exc}") from exc


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", type=Path)
    args = parser.parse_args(argv)
    try:
        summary = validate_sample(args.path)
    except (FileNotFoundError, SampleValidationError) as exc:
        parser.exit(2, str(exc) + "\n")
    print(json.dumps(asdict(summary), sort_keys=True, separators=(",", ":")))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
