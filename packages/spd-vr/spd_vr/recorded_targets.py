"""Read schema-v1 observations as synthetic replay targets, never recorded commands.

Ordering comes from the supplied collection ``schema-v1.md``, sections 4 and 5:
left arm 7, right arm 7, left hand 20, right hand 20; independent streams use
backward as-of lookup, with the last stored sample winning duplicate timestamps.
The known ``tianji_wuji2_v1`` name order is shared with the JointCommand contract.
Missing optional name metadata uses that documented order, not guessed columns;
provided name/config metadata must agree with it.
"""
from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any, Mapping

import h5py
import numpy as np

from .ros_joint_command import JOINT_NAME_TUPLE, ROBOT_CONFIG, SCHEMA_VERSION


def _text(value: Any, label: str) -> str:
    if isinstance(value, bytes):
        value = value.decode("utf-8")
    if not isinstance(value, str) or not value:
        raise ValueError(f"{label} must be a nonempty UTF-8 string")
    return value


def _names(value: Any, label: str) -> tuple[str, ...]:
    if isinstance(value, (str, bytes)):
        value = json.loads(_text(value, label))
    if not isinstance(value, (list, tuple, np.ndarray)):
        raise ValueError(f"{label} must be a sequence of names")
    return tuple(_text(name, label) for name in value)


def _version(value: Any, label: str) -> None:
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, (int, np.integer)) or value != SCHEMA_VERSION:
        raise ValueError(f"{label} must be schema version {SCHEMA_VERSION}")


def _timestamps(group: h5py.Group) -> np.ndarray:
    dataset = group.get("timestamp_ns")
    if not isinstance(dataset, h5py.Dataset) or dataset.ndim != 1 or dataset.dtype.kind != "i" or dataset.dtype.itemsize != 8:
        raise ValueError(f"{group.name}/timestamp_ns must be an int64 vector")
    timestamps = dataset[:]
    if not len(timestamps) or timestamps[0] < 0 or np.any(timestamps[1:] < timestamps[:-1]):
        raise ValueError(f"{dataset.name} must be nonempty, nonnegative and non-decreasing")
    timestamps.setflags(write=False)
    return timestamps


def _check_metadata(metadata: Mapping[str, Any], label: str, expected_names: tuple[str, ...]) -> None:
    if "schema_version" in metadata:
        _version(metadata["schema_version"], f"{label} schema_version")
    if "robot_config" in metadata and _text(metadata["robot_config"], label) != ROBOT_CONFIG:
        raise ValueError(f"{label} robot_config conflicts with {ROBOT_CONFIG}")
    if "joint_names" in metadata and _names(metadata["joint_names"], label) != expected_names:
        raise ValueError(f"{label} joint_names conflict with the documented joint order")
    if "joint_unit" in metadata and _text(metadata["joint_unit"], label) != "rad":
        raise ValueError(f"{label} joint_unit must be rad")


def _check_node_metadata(node: h5py.Group | h5py.Dataset, names: tuple[str, ...]) -> None:
    _check_metadata(node.attrs, node.name, names)
    if isinstance(node, h5py.Group) and "joint_names" in node:
        dataset = node["joint_names"]
        if not isinstance(dataset, h5py.Dataset):
            raise ValueError(f"{node.name}/joint_names must be a dataset")
        _check_metadata({"joint_names": dataset[()]}, node.name, names)


class RecordedTargets:
    """Read-only, independent arm/hand as-of replay on their common time interval.

    Joint arrays and timestamp indexes are resident. JPEG payloads are read only
    on demand using a short-lived read-only file handle (no persistent handle to
    close, and no HDF5 access from ``sample``). Invalid input raises ValueError;
    missing/unreadable files retain the filesystem/HDF5 exception.
    """

    def __init__(self, path: str | Path) -> None:
        self.path = Path(path).expanduser().resolve()
        self.name = self.path.name
        self._image_times: dict[str, np.ndarray] = {}
        with h5py.File(self.path, "r") as handle:
            if "schema_version" not in handle.attrs or "robot_config" not in handle.attrs:
                raise ValueError("recording requires schema_version and robot_config attributes")
            _version(handle.attrs["schema_version"], "recording schema_version")
            self.robot_config = _text(handle.attrs["robot_config"], "robot_config")
            if self.robot_config != ROBOT_CONFIG:
                raise ValueError(f"unsupported robot_config: {self.robot_config}")
            self.task = _text(handle.attrs.get("task"), "task")
            _check_node_metadata(handle, JOINT_NAME_TUPLE)
            observations = handle.get("observations")
            if not isinstance(observations, h5py.Group):
                raise ValueError("recording requires observations")
            _check_node_metadata(observations, JOINT_NAME_TUPLE)
            streams = []
            for name, names in (("arms", JOINT_NAME_TUPLE[:14]), ("hands", JOINT_NAME_TUPLE[14:])):
                group = observations.get(name)
                if not isinstance(group, h5py.Group):
                    raise ValueError(f"recording requires observations/{name}")
                _check_node_metadata(group, names)
                timestamps = _timestamps(group)
                dataset = group.get("qpos")
                if not isinstance(dataset, h5py.Dataset) or dataset.shape != (len(timestamps), len(names)) or dataset.dtype.kind != "f":
                    raise ValueError(f"{group.name}/qpos must be floating-point [{len(timestamps)},{len(names)}]")
                _check_node_metadata(dataset, names)
                values = np.asarray(dataset[:], dtype=np.float64)
                if not np.isfinite(values).all():
                    raise ValueError(f"{dataset.name} contains non-finite joint positions")
                values.setflags(write=False)
                streams.append((timestamps, values))
            (self._arm_times, self._arms), (self._hand_times, self._hands) = streams
            self._origin_ns = max(int(self._arm_times[0]), int(self._hand_times[0]))
            self._end_ns = min(int(self._arm_times[-1]), int(self._hand_times[-1]))
            if self._end_ns < self._origin_ns:
                raise ValueError("arm and hand streams have no common time interval")
            self.duration_s = (self._end_ns - self._origin_ns) / 1_000_000_000
            images = handle.get("images")
            if images is not None:
                if not isinstance(images, h5py.Group):
                    raise ValueError("images must be a group")
                for name, group in images.items():
                    if not isinstance(group, h5py.Group):
                        raise ValueError(f"images/{name} must be a camera group")
                    timestamps = _timestamps(group)
                    jpeg = group.get("jpeg")
                    if not isinstance(jpeg, h5py.Dataset) or jpeg.shape != (len(timestamps),) or h5py.check_dtype(vlen=jpeg.dtype) != np.dtype("uint8"):
                        raise ValueError(f"{group.name}/jpeg must be vlen uint8 with one payload per timestamp")
                    self._image_times[name] = timestamps
            self.camera_names = tuple(self._image_times)
            for key in ("config", "dataset_config"):
                if key in handle.attrs:
                    self._check_config(json.loads(_text(handle.attrs[key], key)), key)
                if key in handle:
                    dataset = handle[key]
                    if not isinstance(dataset, h5py.Dataset) or dataset.shape != ():
                        raise ValueError(f"{key} must be scalar JSON metadata")
                    self._check_config(json.loads(_text(dataset[()], key)), key)
            self._check_config(handle.attrs, "recording")
        config_path = self.path.parent / "dataset_config.json"
        if config_path.exists():
            self._check_config(json.loads(config_path.read_text(encoding="utf-8")), str(config_path))

    def _check_config(self, config: Mapping[str, Any], label: str) -> None:
        if not isinstance(config, Mapping):
            raise ValueError(f"{label} must be a configuration mapping")
        _check_metadata(config, label, JOINT_NAME_TUPLE)
        if "camera_names" in config:
            names = _names(config["camera_names"], f"{label} camera_names")
            if len(names) != len(set(names)) or set(names) != set(self.camera_names):
                raise ValueError(f"{label} camera_names conflict with recording streams")
        if "image_encoding" in config and _text(config["image_encoding"], label) != "jpeg":
            raise ValueError(f"{label} image_encoding must be jpeg")

    def _timestamp(self, elapsed_s: float) -> int:
        elapsed = float(elapsed_s)
        if not math.isfinite(elapsed):
            raise ValueError("elapsed_s must be finite")
        if elapsed <= 0:
            return self._origin_ns
        if elapsed >= self.duration_s:
            return self._end_ns
        return min(self._end_ns, self._origin_ns + int(elapsed * 1_000_000_000))

    def sample(self, elapsed_s: float) -> np.ndarray:
        """Return a fresh float64 [54] target; clamp outside the common interval.

        No interpolation is applied. At the end, each stream holds its own last
        sample at or before the common endpoint, not its later unpaired tail.
        """
        timestamp = self._timestamp(elapsed_s)
        arm_index = int(np.searchsorted(self._arm_times, timestamp, side="right")) - 1
        hand_index = int(np.searchsorted(self._hand_times, timestamp, side="right")) - 1
        target = np.empty(54, dtype=np.float64)
        target[:14] = self._arms[arm_index]
        target[14:] = self._hands[hand_index]
        return target

    def image_jpeg(self, name: str, elapsed_s: float) -> bytes | None:
        """Read only the selected original JPEG; return None before its first frame.

        Unknown camera names raise KeyError. JPEG decoding belongs to the caller,
        not the target/publishing thread. Empty payloads are errors, never images.
        """
        timestamp = self._timestamp(elapsed_s)
        index = int(np.searchsorted(self._image_times[name], timestamp, side="right")) - 1
        if index < 0:
            return None
        with h5py.File(self.path, "r") as handle:
            payload = handle[f"images/{name}/jpeg"][index].tobytes()
        if not payload:
            raise ValueError(f"images/{name}/jpeg[{index}] is empty")
        return payload


__all__ = ["RecordedTargets"]
