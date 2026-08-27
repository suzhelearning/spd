"""Atomic, replay-oriented SPD VR episode recorder and schema validator."""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import shutil
import tempfile
from typing import Any, Mapping

import h5py
import numpy as np

from .camera import CAMERA_NAMES, CameraFrame, validate_frame_mapping


SCHEMA_VERSION = 1


def _sha256_array(value: np.ndarray) -> str:
    digest = hashlib.sha256()
    contiguous = np.ascontiguousarray(value)
    if contiguous.dtype.kind in {"O", "U", "S"}:
        for item in contiguous.reshape(-1):
            if isinstance(item, bytes):
                payload = item
            else:
                payload = str(item).encode("utf-8")
            digest.update(len(payload).to_bytes(8, "little"))
            digest.update(payload)
    else:
        digest.update(contiguous.tobytes(order="C"))
    return digest.hexdigest()


def _array(value: Any, shape: tuple[int, ...], name: str, dtype=np.float64) -> np.ndarray:
    result = np.asarray(value, dtype=dtype)
    if result.shape != shape:
        raise ValueError(f"{name} must have shape {shape}, got {result.shape}")
    if not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain finite values")
    return result.copy()


class EpisodeRecorder:
    """Collect in memory, then publish one fully validated HDF5 episode."""

    def __init__(
        self,
        output_root: str | Path,
        *,
        code_checksums: Mapping[str, str] | None = None,
        config_checksums: Mapping[str, str] | None = None,
        model_checksums: Mapping[str, str] | None = None,
        sample_checksums: Mapping[str, str] | None = None,
    ) -> None:
        self.output_root = Path(output_root)
        self.code_checksums = dict(code_checksums or {})
        self.config_checksums = dict(config_checksums or {})
        self.model_checksums = dict(model_checksums or {})
        self.sample_checksums = dict(sample_checksums or {})
        self._reset_buffers()

    def _reset_buffers(self) -> None:
        self.episode_id: str | None = None
        self.task_manifest: dict[str, Any] = {}
        self._robot: list[dict[str, Any]] = []
        self._hands: list[dict[str, Any]] = []
        self._cameras: dict[str, list[CameraFrame]] = {name: [] for name in CAMERA_NAMES}
        self._objects: list[dict[str, Any]] = []
        self._contacts: list[dict[str, Any]] = []
        self._started = False

    def start_episode(self, episode_id: str | int, task_manifest: Mapping[str, Any]) -> None:
        if self._started:
            raise RuntimeError("an episode is already recording")
        self._reset_buffers()
        self.episode_id = str(episode_id)
        self.task_manifest = json.loads(json.dumps(task_manifest))
        self._started = True

    def append_robot(
        self,
        timestamp_ns: int,
        qpos: Any,
        qvel: Any,
        qpos_target: Any,
        *,
        arm_valid_mask: int = 0,
        hand_valid_mask: int = 0,
    ) -> None:
        if not self._started:
            raise RuntimeError("episode is not recording")
        row = {
            "timestamp_ns": int(timestamp_ns),
            "qpos": _array(qpos, (54,), "qpos"),
            "qvel": _array(qvel, (54,), "qvel"),
            "qpos_target": _array(qpos_target, (54,), "qpos_target"),
            "arm_valid_mask": int(arm_valid_mask),
            "hand_valid_mask": int(hand_valid_mask),
        }
        if self._robot and row["timestamp_ns"] <= self._robot[-1]["timestamp_ns"]:
            raise ValueError("robot timestamps must be strictly increasing")
        self._robot.append(row)

    def append_hands(
        self,
        timestamp_ns: int,
        sequence_id: int,
        tracking_epoch: int,
        left_hand: Any,
        right_hand: Any,
        *,
        left_active: bool,
        right_active: bool,
        left_scale: float = 1.0,
        right_scale: float = 1.0,
    ) -> None:
        if not self._started:
            raise RuntimeError("episode is not recording")
        left_scale = float(left_scale)
        right_scale = float(right_scale)
        if not np.isfinite(left_scale) or left_scale <= 0.0 or not np.isfinite(right_scale) or right_scale <= 0.0:
            raise ValueError("hand scales must be finite and positive")
        row = {
            "timestamp_ns": int(timestamp_ns),
            "sequence_id": int(sequence_id),
            "tracking_epoch": int(tracking_epoch),
            "left_hand": _array(left_hand, (26, 7), "left_hand", np.float32),
            "right_hand": _array(right_hand, (26, 7), "right_hand", np.float32),
            "left_active": bool(left_active),
            "right_active": bool(right_active),
            "left_scale": np.float32(left_scale),
            "right_scale": np.float32(right_scale),
        }
        if self._hands and (
            row["timestamp_ns"] <= self._hands[-1]["timestamp_ns"]
            or row["sequence_id"] <= self._hands[-1]["sequence_id"]
        ):
            raise ValueError("hand timestamp and sequence must be strictly increasing")
        self._hands.append(row)

    def append_cameras(self, frames: Mapping[str, CameraFrame]) -> None:
        if not self._started:
            raise RuntimeError("episode is not recording")
        frames = validate_frame_mapping(frames)
        for name in CAMERA_NAMES:
            frame = frames[name]
            if self._cameras[name] and frame.timestamp_ns <= self._cameras[name][-1].timestamp_ns:
                raise ValueError(f"camera {name} timestamps must be strictly increasing")
            self._cameras[name].append(frame)

    def append_objects(self, timestamp_ns: int, state: Any) -> None:
        if not self._started:
            raise RuntimeError("episode is not recording")
        timestamp_ns = int(timestamp_ns)
        if self._objects and timestamp_ns <= self._objects[-1]["timestamp_ns"]:
            raise ValueError("object timestamps must be strictly increasing")
        self._objects.append({"timestamp_ns": timestamp_ns, "state": json.loads(json.dumps(state))})

    def append_contacts(self, timestamp_ns: int, contacts: Any) -> None:
        if not self._started:
            raise RuntimeError("episode is not recording")
        timestamp_ns = int(timestamp_ns)
        if self._contacts and timestamp_ns <= self._contacts[-1]["timestamp_ns"]:
            raise ValueError("contact timestamps must be strictly increasing")
        self._contacts.append({"timestamp_ns": timestamp_ns, "contacts": json.loads(json.dumps(contacts))})

    def submit(
        self,
        sim_time_ns: int,
        qpos: Any,
        qvel: Any,
        qpos_target: Any,
        *,
        arm_valid_mask: int = 0,
        hand_valid_mask: int = 0,
        **_: Any,
    ) -> None:
        if not self._started:
            return
        self.append_robot(
            sim_time_ns,
            qpos,
            qvel,
            qpos_target,
            arm_valid_mask=arm_valid_mask,
            hand_valid_mask=hand_valid_mask,
        )

    def _datasets(self) -> dict[str, np.ndarray]:
        robot = self._robot
        hands = self._hands
        datasets: dict[str, np.ndarray] = {
            "timestamps/robot_ns": np.asarray([row["timestamp_ns"] for row in robot], dtype=np.uint64),
            "observations/qpos": np.asarray([row["qpos"] for row in robot], dtype=np.float64).reshape((-1, 54)),
            "observations/qvel": np.asarray([row["qvel"] for row in robot], dtype=np.float64).reshape((-1, 54)),
            "actions/qpos_target": np.asarray([row["qpos_target"] for row in robot], dtype=np.float64).reshape((-1, 54)),
            "validity/arm_mask": np.asarray([row["arm_valid_mask"] for row in robot], dtype=np.uint8),
            "validity/hand_mask": np.asarray([row["hand_valid_mask"] for row in robot], dtype=np.uint8),
            "timestamps/hands_ns": np.asarray([row["timestamp_ns"] for row in hands], dtype=np.uint64),
            "pico/hands/sequence_id": np.asarray([row["sequence_id"] for row in hands], dtype=np.uint64),
            "pico/hands/tracking_epoch": np.asarray([row["tracking_epoch"] for row in hands], dtype=np.uint64),
            "pico/hands/left_active": np.asarray([row["left_active"] for row in hands], dtype=np.bool_),
            "pico/hands/right_active": np.asarray([row["right_active"] for row in hands], dtype=np.bool_),
            "pico/hands/left_scale": np.asarray([row["left_scale"] for row in hands], dtype=np.float32),
            "pico/hands/right_scale": np.asarray([row["right_scale"] for row in hands], dtype=np.float32),
            "pico/hands/left_hand": np.asarray([row["left_hand"] for row in hands], dtype=np.float32).reshape((-1, 26, 7)),
            "pico/hands/right_hand": np.asarray([row["right_hand"] for row in hands], dtype=np.float32).reshape((-1, 26, 7)),
            "timestamps/objects_ns": np.asarray([row["timestamp_ns"] for row in self._objects], dtype=np.uint64),
            "objects/state_json": np.asarray([json.dumps(row["state"], sort_keys=True) for row in self._objects], dtype=h5py.string_dtype()),
            "timestamps/contacts_ns": np.asarray([row["timestamp_ns"] for row in self._contacts], dtype=np.uint64),
            "contacts/json": np.asarray([json.dumps(row["contacts"], sort_keys=True) for row in self._contacts], dtype=h5py.string_dtype()),
            "contacts/hand_object": np.asarray([_has_hand_object_contact(row["contacts"]) for row in self._contacts], dtype=np.bool_),
        }
        for name in CAMERA_NAMES:
            frames = self._cameras[name]
            datasets[f"timestamps/cameras/{name}_ns"] = np.asarray([frame.timestamp_ns for frame in frames], dtype=np.uint64)
            datasets[f"cameras/{name}/rgb"] = np.asarray([frame.rgb for frame in frames], dtype=np.uint8).reshape((-1, 168, 224, 3))
            datasets[f"cameras/{name}/segmentation"] = np.asarray([frame.segmentation for frame in frames], dtype=np.int32).reshape((-1, 168, 224, 2))
        return datasets

    @staticmethod
    def _write_dataset(handle: h5py.File, name: str, value: np.ndarray) -> None:
        parent, _, leaf = name.rpartition("/")
        group = handle.require_group(parent) if parent else handle
        if value.ndim == 0 or (value.ndim > 0 and value.shape[0] == 0):
            group.create_dataset(leaf, data=value)
            return
        chunks = (max(1, min(32, value.shape[0])), *value.shape[1:])
        group.create_dataset(leaf, data=value, chunks=chunks, compression="gzip", compression_opts=4, shuffle=True)

    def finish_episode(self) -> Path:
        if not self._started or self.episode_id is None:
            raise RuntimeError("no episode is recording")
        datasets = self._datasets()
        if datasets["observations/qpos"].shape[1:] != (54,):
            raise ValueError("qpos dataset must be [N,54]")
        if datasets["observations/qpos"].shape[0] == 0:
            raise ValueError("episode has no robot frames")
        if datasets["timestamps/hands_ns"].shape[0] == 0:
            raise ValueError("episode has no PICO hand frames")
        if any(datasets[f"timestamps/cameras/{name}_ns"].shape[0] == 0 for name in CAMERA_NAMES):
            raise ValueError("episode is missing one or more policy camera streams")
        if not np.all(np.isfinite(datasets["observations/qpos"])):
            raise ValueError("non-finite observation")
        episode_parent = self.output_root / "episodes"
        episode_parent.mkdir(parents=True, exist_ok=True)
        final_dir = episode_parent / self.episode_id
        if final_dir.exists():
            raise FileExistsError(final_dir)
        staging = Path(tempfile.mkdtemp(prefix=f".{self.episode_id}.staging-", dir=episode_parent))
        h5_path = staging / "episode.hdf5"
        try:
            with h5py.File(h5_path, "w") as handle:
                handle.attrs["schema_version"] = SCHEMA_VERSION
                for name, value in datasets.items():
                    self._write_dataset(handle, name, value)
                handle.flush()
            manifest = {
                "schema_version": SCHEMA_VERSION,
                "episode_id": self.episode_id,
                "task_reset_manifest": self.task_manifest,
                "lengths": {
                    "robot": int(datasets["timestamps/robot_ns"].shape[0]),
                    "hands": int(datasets["timestamps/hands_ns"].shape[0]),
                    "objects": int(datasets["timestamps/objects_ns"].shape[0]),
                    "contacts": int(datasets["timestamps/contacts_ns"].shape[0]),
                    "cameras": {name: int(datasets[f"timestamps/cameras/{name}_ns"].shape[0]) for name in CAMERA_NAMES},
                },
                "calibration_revisions": {
                    name: sorted({frame.calibration_revision for frame in self._cameras[name]})
                    for name in CAMERA_NAMES
                },
                "checksums": {
                    name: _sha256_array(value)
                    for name, value in datasets.items()
                },
                "code_checksums": self.code_checksums,
                "config_checksums": self.config_checksums,
                "model_checksums": self.model_checksums,
                "sample_checksums": self.sample_checksums,
            }
            (staging / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True), encoding="utf-8")
            validate_episode_path(staging)
            os.replace(staging, final_dir)
            self._reset_buffers()
            return final_dir
        except Exception:
            shutil.rmtree(staging, ignore_errors=True)
            raise

    def discard_episode(self, reason: str) -> None:
        self._reset_buffers()

def _has_hand_object_contact(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, Mapping):
        if "hand_object" in value:
            return bool(value["hand_object"])
        return any(bool(item) for item in value.values())
    if isinstance(value, (list, tuple, set)):
        return bool(value)
    return bool(value)


def _resolve_episode(path: str | Path) -> Path:
    path = Path(path)
    if path.is_file() and path.name == "episode.hdf5":
        return path.parent
    if (path / "episode.hdf5").is_file():
        return path
    episodes = path / "episodes"
    if episodes.is_dir():
        candidates = sorted(item for item in episodes.iterdir() if (item / "episode.hdf5").is_file())
        if len(candidates) == 1:
            return candidates[0]
        if not candidates:
            raise FileNotFoundError(f"no episode.hdf5 under {path}")
        raise ValueError(f"multiple episodes under {path}; pass one episode directory")
    raise FileNotFoundError(path)


def validate_episode_path(path: str | Path) -> dict[str, Any]:
    episode_dir = _resolve_episode(path)
    manifest_path = episode_dir / "manifest.json"
    h5_path = episode_dir / "episode.hdf5"
    if not manifest_path.is_file() or not h5_path.is_file():
        raise ValueError("episode publication requires episode.hdf5 and manifest.json")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if manifest.get("schema_version") != SCHEMA_VERSION:
        raise ValueError("unsupported episode schema version")
    with h5py.File(h5_path, "r") as handle:
        required = [
            "timestamps/robot_ns",
            "observations/qpos",
            "observations/qvel",
            "actions/qpos_target",
            "validity/arm_mask",
            "validity/hand_mask",
            "timestamps/hands_ns",
            "pico/hands/sequence_id",
            "pico/hands/tracking_epoch",
            "pico/hands/left_active",
            "pico/hands/right_active",
            "pico/hands/left_scale",
            "pico/hands/right_scale",
            "pico/hands/left_hand",
            "pico/hands/right_hand",
            "timestamps/objects_ns",
            "objects/state_json",
            "timestamps/contacts_ns",
            "contacts/json",
            "contacts/hand_object",
        ]
        for camera in CAMERA_NAMES:
            required.extend((
                f"timestamps/cameras/{camera}_ns",
                f"cameras/{camera}/rgb",
                f"cameras/{camera}/segmentation",
            ))
        for name in required:
            if name not in handle:
                raise ValueError(f"missing dataset: {name}")
        qpos = handle["observations/qpos"][:]
        qvel = handle["observations/qvel"][:]
        target = handle["actions/qpos_target"][:]
        robot_ns = handle["timestamps/robot_ns"][:]
        hands_ns = handle["timestamps/hands_ns"][:]
        left_hand = handle["pico/hands/left_hand"][:]
        right_hand = handle["pico/hands/right_hand"][:]
        left_active = handle["pico/hands/left_active"][:]
        right_active = handle["pico/hands/right_active"][:]
        left_scale = handle["pico/hands/left_scale"][:]
        right_scale = handle["pico/hands/right_scale"][:]
        sequence_id = handle["pico/hands/sequence_id"][:]
        tracking_epoch = handle["pico/hands/tracking_epoch"][:]
        arm_mask = handle["validity/arm_mask"][:]
        hand_mask = handle["validity/hand_mask"][:]
        for name, values in (
            ("timestamps/robot_ns", robot_ns),
            ("timestamps/hands_ns", hands_ns),
            ("pico/hands/sequence_id", sequence_id),
            ("pico/hands/tracking_epoch", tracking_epoch),
        ):
            if values.dtype != np.dtype("uint64"):
                raise ValueError(f"{name} must be uint64")
        if sequence_id.shape != hands_ns.shape or tracking_epoch.shape != hands_ns.shape:
            raise ValueError("hand metadata length mismatch")
        for name, values in (("validity/arm_mask", arm_mask), ("validity/hand_mask", hand_mask)):
            if values.shape != robot_ns.shape or values.dtype != np.dtype("uint8"):
                raise ValueError(f"{name} must be uint8 with robot length")
        if qpos.ndim != 2 or qpos.shape[1] != 54 or qvel.shape != qpos.shape or target.shape != qpos.shape:
            raise ValueError("robot datasets must all be [N,54]")
        if robot_ns.ndim != 1 or robot_ns.shape[0] != qpos.shape[0]:
            raise ValueError("robot timestamp length mismatch")
        if hands_ns.ndim != 1 or left_hand.shape != right_hand.shape or left_hand.ndim != 3 or left_hand.shape[1:] != (26, 7):
            raise ValueError("hand datasets must be [N,26,7]")
        if any(array.dtype != np.dtype("float32") for array in (left_hand, right_hand, left_scale, right_scale)):
            raise ValueError("hand numeric datasets must be float32")
        if left_active.shape != (hands_ns.shape[0],) or right_active.shape != (hands_ns.shape[0],):
            raise ValueError("hand active length mismatch")
        if left_active.dtype != np.dtype("bool") or right_active.dtype != np.dtype("bool"):
            raise ValueError("hand active datasets must be bool")
        if left_scale.shape != hands_ns.shape or right_scale.shape != hands_ns.shape:
            raise ValueError("hand scale length mismatch")
        if (
            left_hand.shape[0] != hands_ns.shape[0]
            or not np.all(np.isfinite(left_hand))
            or not np.all(np.isfinite(right_hand))
            or not np.all(np.isfinite(left_scale))
            or not np.all(np.isfinite(right_scale))
            or np.any(left_scale <= 0.0)
            or np.any(right_scale <= 0.0)
        ):
            raise ValueError("hand dataset length or finite-value mismatch")
        lengths = manifest.get("lengths")
        if not isinstance(lengths, dict):
            raise ValueError("manifest lengths are missing")
        if int(lengths.get("robot", -1)) != qpos.shape[0]:
            raise ValueError("manifest robot length mismatch")
        if int(lengths.get("hands", -1)) != hands_ns.shape[0]:
            raise ValueError("manifest hand length mismatch")
        for stream in ("objects", "contacts"):
            if int(lengths.get(stream, -1)) != handle[f"timestamps/{stream}_ns"].shape[0]:
                raise ValueError(f"manifest {stream} length mismatch")
        for name in ("observations/qpos", "observations/qvel", "actions/qpos_target"):
            if not np.all(np.isfinite(handle[name][:])):
                raise ValueError(f"non-finite dataset: {name}")
        for name in (
            "timestamps/robot_ns",
            "timestamps/hands_ns",
            "pico/hands/sequence_id",
            "timestamps/objects_ns",
            "timestamps/contacts_ns",
        ):
            values = handle[name][:]
            if values.size > 1 and np.any(np.diff(values.astype(np.int64)) <= 0):
                raise ValueError(f"timestamp/sequence rollback: {name}")
        camera_lengths = lengths.get("cameras")
        if not isinstance(camera_lengths, dict):
            raise ValueError("manifest camera lengths are missing")
        for camera in CAMERA_NAMES:
            timestamp_name = f"timestamps/cameras/{camera}_ns"
            rgb_name = f"cameras/{camera}/rgb"
            seg_name = f"cameras/{camera}/segmentation"
            timestamp = handle[timestamp_name][:]
            rgb = handle[rgb_name]
            seg = handle[seg_name]
            if int(camera_lengths.get(camera, -1)) != timestamp.shape[0]:
                raise ValueError(f"manifest camera length mismatch: {camera}")
            if timestamp.ndim != 1 or rgb.shape[0] != timestamp.shape[0] or seg.shape[0] != timestamp.shape[0]:
                raise ValueError(f"camera length mismatch: {camera}")
            if rgb.shape[1:] != (168, 224, 3) or rgb.dtype != np.uint8:
                raise ValueError(f"invalid RGB dataset: {camera}")
            if seg.shape[1:] != (168, 224, 2) or seg.dtype != np.int32:
                raise ValueError(f"invalid segmentation dataset: {camera}")
            if timestamp.size > 1 and np.any(np.diff(timestamp.astype(np.int64)) <= 0):
                raise ValueError(f"camera timestamp rollback: {camera}")
        for name, expected in manifest.get("checksums", {}).items():
            if name not in handle:
                raise ValueError(f"checksum dataset missing: {name}")
            actual = _sha256_array(handle[name][:])
            if actual != expected:
                raise ValueError(f"checksum mismatch: {name}")
    return {
        "episode_dir": str(episode_dir),
        "episode_id": manifest.get("episode_id"),
        "schema_version": SCHEMA_VERSION,
        "robot_frames": int(manifest["lengths"]["robot"]),
        "hand_frames": int(manifest["lengths"]["hands"]),
        "camera_frames": manifest["lengths"]["cameras"],
        "valid": True,
    }


__all__ = ["EpisodeRecorder", "SCHEMA_VERSION", "validate_episode_path"]
