"""Schema-v1 episode recorder for real robot state and RGB streams."""

from __future__ import annotations

from dataclasses import dataclass
import io
import json
import os
from pathlib import Path
import queue
import re
import threading
import time
from typing import Any, Mapping, Sequence

import h5py
import numpy as np

from .camera import CAMERA_NAMES, IMAGE_HEIGHT, IMAGE_WIDTH


SCHEMA_VERSION = 1
ROBOT_CONFIG = "tianji_wuji2_v1"
JOINT_UNIT = "rad"
POLICY_RATE_HZ = 30
JPEG_QUALITY = 90

_ARM_NAMES = tuple(f"Joint{index}_{side}" for side in ("L", "R") for index in range(1, 8))
_HAND_BASE_NAMES = (
    "thumb_cmc_flex",
    "thumb_cmc_abd",
    "thumb_mcp",
    "thumb_ip",
    "index_finger_mcp_flex",
    "index_finger_mcp_abd",
    "index_finger_pip",
    "index_finger_dip",
    "middle_finger_mcp_flex",
    "middle_finger_mcp_abd",
    "middle_finger_pip",
    "middle_finger_dip",
    "ring_finger_mcp_flex",
    "ring_finger_mcp_abd",
    "ring_finger_pip",
    "ring_finger_dip",
    "pinky_mcp_flex",
    "pinky_mcp_abd",
    "pinky_pip",
    "pinky_dip",
)
JOINT_NAMES = _ARM_NAMES + tuple(
    f"{side}_{name}" for side in ("l", "r") for name in _HAND_BASE_NAMES
)
_STRING = h5py.string_dtype(encoding="utf-8")


class RecorderError(RuntimeError):
    """Raised when recording cannot continue or publish safely."""


class RecorderQueueOverflow(RecorderError):
    """Raised when the bounded writer queue cannot accept another sample."""


@dataclass(frozen=True)
class _Event:
    kind: str
    payload: Any


def _as_attr_text(value: Any) -> str:
    if isinstance(value, bytes):
        return value.decode("utf-8")
    return str(value)


def _normalise_timestamp(value: Any, origin_ns: int) -> int:
    if isinstance(value, bool):
        raise ValueError("timestamp_ns must be an integer")
    raw = int(value)
    if raw < 0:
        raise ValueError("timestamp_ns must be non-negative")
    # Internal simulation callers may already provide episode-relative time.
    relative = raw if raw < origin_ns else raw - origin_ns
    if relative > np.iinfo(np.int64).max:
        raise ValueError("timestamp_ns exceeds int64")
    return relative


def _validate_qpos(value: Any, size: int, name: str) -> np.ndarray:
    result = np.asarray(value, dtype=np.float32)
    if result.shape != (size,):
        raise ValueError(f"{name} must have shape ({size},), got {result.shape}")
    if not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain finite values")
    return result.copy()


def _validate_rgb(value: Any, name: str) -> np.ndarray:
    result = np.asarray(value)
    expected = (IMAGE_HEIGHT, IMAGE_WIDTH, 3)
    if result.dtype != np.uint8 or result.shape != expected:
        raise ValueError(f"{name} must be uint8[{IMAGE_HEIGHT},{IMAGE_WIDTH},3]")
    return result.copy()


def _episode_stem(episode_id: str | int) -> str:
    text = str(episode_id)
    if text.isdigit():
        return f"episode_{int(text):06d}"
    safe = re.sub(r"[^A-Za-z0-9_.-]+", "_", text).strip("._")
    if not safe:
        raise ValueError("episode_id must contain a filename-safe value")
    return f"episode_{safe}"




class EpisodeRecorder:
    """Write one schema-v1 episode through one bounded background HDF5 writer."""

    def __init__(
        self,
        output_root: str | Path,
        *,
        robot_config: str = ROBOT_CONFIG,
        joint_names: Sequence[str] = JOINT_NAMES,
        camera_names: Sequence[str] = CAMERA_NAMES,
        collection_config: Mapping[str, Any] | None = None,
        queue_size: int = 256,
    ) -> None:
        self.output_root = Path(output_root)
        self.output_root.mkdir(parents=True, exist_ok=True)
        self.robot_config = str(robot_config)
        self.joint_names = tuple(str(name) for name in joint_names)
        self.camera_names = tuple(str(name) for name in camera_names)
        if len(self.joint_names) != 54 or len(set(self.joint_names)) != 54:
            raise ValueError("joint_names must contain 54 unique names")
        if not self.camera_names or len(set(self.camera_names)) != len(self.camera_names):
            raise ValueError("camera_names must contain at least one unique camera")
        if any(name not in CAMERA_NAMES for name in self.camera_names):
            raise ValueError(f"unknown camera name in {self.camera_names}")
        if int(queue_size) < 2:
            raise ValueError("queue_size must be at least 2")
        self._queue_size = int(queue_size)
        self._collection_config = dict(collection_config or {})
        self._lock = threading.RLock()
        self._state = "idle"
        self._episode_id: str | None = None
        self._partial_path: Path | None = None
        self._final_path: Path | None = None
        self._origin_ns = 0
        self._queue: queue.Queue[_Event] | None = None
        self._thread: threading.Thread | None = None
        self._done = threading.Event()
        self._abort_event = threading.Event()
        self._published_path: Path | None = None
        self._error: BaseException | None = None

    @property
    def is_recording(self) -> bool:
        with self._lock:
            return self._state == "recording"

    @property
    def is_busy(self) -> bool:
        with self._lock:
            return self._state not in {"idle"}

    @property
    def error(self) -> BaseException | None:
        with self._lock:
            return self._error

    @property
    def partial_path(self) -> Path | None:
        return self._partial_path

    def _dataset_config(self) -> dict[str, Any]:
        return {
            "schema_version": SCHEMA_VERSION,
            "robot_config": self.robot_config,
            "joint_names": list(self.joint_names),
            "joint_unit": JOINT_UNIT,
            "policy_rate_hz": POLICY_RATE_HZ,
            "camera_names": list(self.camera_names),
            "image_width": IMAGE_WIDTH,
            "image_height": IMAGE_HEIGHT,
            "image_encoding": "jpeg",
            "decoded_color_order": "RGB",
            "jpeg_quality": JPEG_QUALITY,
        }

    def _ensure_dataset_config(self) -> None:
        path = self.output_root / "dataset_config.json"
        expected = self._dataset_config()
        if path.exists():
            actual = json.loads(path.read_text(encoding="utf-8"))
            if actual != expected:
                raise ValueError("dataset_config.json does not match the requested dataset contract")
        else:
            temporary = path.with_suffix(".json.tmp")
            temporary.write_text(json.dumps(expected, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
            os.replace(temporary, path)
        if self._collection_config:
            collection_path = self.output_root / "collection_config.json"
            if collection_path.exists():
                actual = json.loads(collection_path.read_text(encoding="utf-8"))
                if actual != self._collection_config:
                    raise ValueError("collection_config.json does not match the requested collection contract")
            else:
                temporary = collection_path.with_suffix(".json.tmp")
                temporary.write_text(
                    json.dumps(self._collection_config, indent=2, ensure_ascii=False) + "\n",
                    encoding="utf-8",
                )
                os.replace(temporary, collection_path)

    @staticmethod
    def _create_h5(path: Path, *, task: str, robot_config: str) -> h5py.File:
        handle = h5py.File(path, "w")
        handle.attrs["schema_version"] = SCHEMA_VERSION
        handle.attrs["task"] = str(task)
        handle.attrs["success"] = False
        handle.attrs["robot_config"] = str(robot_config)
        arms = handle.require_group("observations").require_group("arms")
        hands = handle["observations"].require_group("hands")
        arms.create_dataset("timestamp_ns", shape=(0,), maxshape=(None,), dtype=np.int64, chunks=(256,))
        arms.create_dataset("qpos", shape=(0, 14), maxshape=(None, 14), dtype=np.float32, chunks=(256, 14))
        hands.create_dataset("timestamp_ns", shape=(0,), maxshape=(None,), dtype=np.int64, chunks=(256,))
        hands.create_dataset("qpos", shape=(0, 40), maxshape=(None, 40), dtype=np.float32, chunks=(256, 40))
        commands = handle["observations"].create_group("commands")
        commands.create_dataset("timestamp_ns", shape=(0,), maxshape=(None,), dtype=np.int64, chunks=(256,))
        commands.create_dataset("stamp_utc_ns", shape=(0,), maxshape=(None,), dtype=np.int64, chunks=(256,))
        commands.create_dataset("sequence", shape=(0,), maxshape=(None,), dtype=np.uint64, chunks=(256,))
        commands.create_dataset("ready_mask", shape=(0,), maxshape=(None,), dtype=np.uint8, chunks=(256,))
        commands.create_dataset("hold_mask", shape=(0,), maxshape=(None,), dtype=np.uint8, chunks=(256,))
        commands.create_dataset("applied_sim_time_ns", shape=(0,), maxshape=(None,), dtype=np.int64, chunks=(256,))
        commands.create_dataset("position_rad", shape=(0, 54), maxshape=(None, 54), dtype=np.float32, chunks=(256, 54))
        commands.create_dataset("session_id", shape=(0,), maxshape=(None,), dtype=_STRING, chunks=(256,))
        commands.create_dataset("status", shape=(0,), maxshape=(None,), dtype=_STRING, chunks=(256,))
        commands.create_dataset("reject_reason", shape=(0,), maxshape=(None,), dtype=_STRING, chunks=(256,))
        return handle

    def start_episode(self, episode_id: str | int, task_manifest: Mapping[str, Any] | str) -> None:
        with self._lock:
            if self._state != "idle":
                raise RuntimeError("recorder is busy with another episode")
            self._ensure_dataset_config()
            stem = _episode_stem(episode_id)
            partial = self.output_root / f"{stem}.partial.h5"
            final = self.output_root / f"{stem}.h5"
            if partial.exists() or final.exists():
                raise FileExistsError(final if final.exists() else partial)
            if isinstance(task_manifest, Mapping):
                task = task_manifest.get("task") or task_manifest.get("scene") or str(episode_id)
            else:
                task = task_manifest
            with self._create_h5(partial, task=str(task), robot_config=self.robot_config) as handle:
                if isinstance(task_manifest, Mapping):
                    handle.attrs["task_manifest"] = json.dumps(dict(task_manifest), sort_keys=True)
                    for key in ("scene", "seed"):
                        if key in task_manifest:
                            handle.attrs[key] = task_manifest[key]
            self._episode_id = str(episode_id)
            self._partial_path = partial
            self._final_path = final
            self._origin_ns = time.monotonic_ns()
            self._queue = queue.Queue(maxsize=self._queue_size)
            self._done.clear()
            self._abort_event.clear()
            self._published_path = None
            self._error = None
            self._state = "recording"
            self._thread = threading.Thread(target=self._writer, name="spd-vr-hdf5-writer", daemon=True)
            self._thread.start()

    def _put(self, kind: str, payload: Any) -> None:
        with self._lock:
            if self._state != "recording" or self._queue is None:
                raise RuntimeError("no episode is recording")
            try:
                self._queue.put_nowait(_Event(kind, payload))
            except queue.Full as exc:
                self._error = RecorderQueueOverflow("schema-v1 writer queue overflow")
                self._state = "failed"
                self._abort_event.set()
                raise self._error from exc

    def append_arm_qpos(self, timestamp_ns: int, qpos: Any) -> None:
        timestamp = _normalise_timestamp(timestamp_ns, self._origin_ns)
        self._put("arms", (timestamp, _validate_qpos(qpos, 14, "arms.qpos")))

    def append_hand_qpos(self, timestamp_ns: int, qpos: Any, *, both_fresh: bool = True) -> None:
        if not both_fresh:
            return
        timestamp = _normalise_timestamp(timestamp_ns, self._origin_ns)
        self._put("hands", (timestamp, _validate_qpos(qpos, 40, "hands.qpos")))

    def append_image(self, name: str, timestamp_ns: int, rgb: Any) -> None:
        if name not in self.camera_names:
            return
        timestamp = _normalise_timestamp(timestamp_ns, self._origin_ns)
        self._put("image", (name, timestamp, _validate_rgb(rgb, f"images/{name}.rgb")))

    def append_cameras(
        self,
        frames: Mapping[str, Any],
        *,
        available_timestamp_ns: int | None = None,
    ) -> None:
        for name in self.camera_names:
            frame = frames.get(name)
            if frame is None:
                continue
            timestamp = (
                int(available_timestamp_ns)
                if available_timestamp_ns is not None
                else int(getattr(frame, "timestamp_ns"))
            )
            self.append_image(name, timestamp, getattr(frame, "rgb"))

    def append_robot(
        self,
        timestamp_ns: int,
        qpos: Any,
        qvel: Any | None = None,
        qpos_target: Any | None = None,
        *,
        arm_valid_mask: int = 0,
        hand_valid_mask: int = 0,
    ) -> None:
        """Compatibility ingress that records only the real 54-DoF qpos."""
        del qvel, qpos_target
        vector = _validate_qpos(qpos, 54, "robot.qpos")
        self.append_arm_qpos(timestamp_ns, np.concatenate((vector[:7], vector[27:34])))
        if int(hand_valid_mask) == 3:
            self.append_hand_qpos(timestamp_ns, np.concatenate((vector[7:27], vector[34:54])))
        del arm_valid_mask

    def submit(self, sim_time_ns: int, qpos: Any, **kwargs: Any) -> None:
        self.append_robot(sim_time_ns, qpos, **kwargs)

    def append_command(
        self,
        timestamp_ns: int,
        position_rad: Any,
        *,
        sequence: int,
        stamp_utc_ns: int,
        ready_mask: int,
        session_id: str,
        applied_sim_time_ns: int,
        hold_mask: int = 0,
        status: str = "accepted",
        reject_reason: str = "",
    ) -> None:
        """Record one complete target snapshot separately from actual qpos."""
        if not session_id:
            raise ValueError("command session_id must be non-empty")
        if int(sequence) <= 0 or int(stamp_utc_ns) <= 0:
            raise ValueError("command sequence and stamp must be positive")
        values = _validate_qpos(position_rad, 54, "commands.position_rad")
        self._put(
            "command",
            {
                "timestamp_ns": _normalise_timestamp(timestamp_ns, self._origin_ns),
                "stamp_utc_ns": int(stamp_utc_ns),
                "sequence": int(sequence),
                "ready_mask": int(ready_mask),
                "hold_mask": int(hold_mask),
                "applied_sim_time_ns": int(applied_sim_time_ns),
                "position_rad": values,
                "session_id": str(session_id),
                "status": str(status),
                "reject_reason": str(reject_reason),
            },
        )

    def _writer(self) -> None:
        assert self._partial_path is not None
        path = self._partial_path
        finish_requested = False
        try:
            with h5py.File(path, "r+") as handle:
                datasets: dict[str, tuple[h5py.Dataset, h5py.Dataset]] = {
                    "arms": (handle["observations/arms/timestamp_ns"], handle["observations/arms/qpos"]),
                    "hands": (handle["observations/hands/timestamp_ns"], handle["observations/hands/qpos"]),
                }
                image_datasets: dict[str, tuple[h5py.Dataset, h5py.Dataset]] = {}
                for name in self.camera_names:
                    group = handle.require_group("images").require_group(name)
                    timestamps = group.create_dataset(
                        "timestamp_ns", shape=(0,), maxshape=(None,), dtype=np.int64, chunks=(64,)
                    )
                    jpeg = group.create_dataset(
                        "jpeg",
                        shape=(0,),
                        maxshape=(None,),
                        dtype=h5py.vlen_dtype(np.dtype("uint8")),
                        chunks=(64,),
                    )
                    image_datasets[name] = (timestamps, jpeg)
                command_group = handle["observations/commands"]
                command_datasets = {
                    name: command_group[name]
                    for name in (
                        "timestamp_ns", "stamp_utc_ns", "sequence", "ready_mask",
                        "hold_mask", "applied_sim_time_ns", "position_rad",
                        "session_id", "status", "reject_reason",
                    )
                }
                last_timestamp: dict[str, int] = {}
                while True:
                    try:
                        event = self._queue.get(timeout=0.1)  # type: ignore[union-attr]
                    except queue.Empty:
                        if self._abort_event.is_set():
                            break
                        continue
                    if event.kind == "finish":
                        handle.attrs["success"] = bool(event.payload)
                        handle.flush()
                        finish_requested = True
                        break
                    if event.kind == "abort":
                        handle.flush()
                        break
                    if event.kind == "discard":
                        break
                    if event.kind == "command":
                        payload = event.payload
                        timestamp = int(payload["timestamp_ns"])
                        previous = last_timestamp.get("commands")
                        if previous is not None and timestamp < previous:
                            raise ValueError("commands timestamps must be non-decreasing")
                        index = command_datasets["timestamp_ns"].shape[0]
                        if previous is not None and timestamp == previous:
                            index -= 1
                        else:
                            for dataset in command_datasets.values():
                                dataset.resize((index + 1,) + dataset.shape[1:])
                            last_timestamp["commands"] = timestamp
                        command_datasets["timestamp_ns"][index] = timestamp
                        command_datasets["stamp_utc_ns"][index] = payload["stamp_utc_ns"]
                        command_datasets["sequence"][index] = payload["sequence"]
                        command_datasets["ready_mask"][index] = payload["ready_mask"]
                        command_datasets["hold_mask"][index] = payload["hold_mask"]
                        command_datasets["applied_sim_time_ns"][index] = payload["applied_sim_time_ns"]
                        command_datasets["position_rad"][index] = payload["position_rad"]
                        command_datasets["session_id"][index] = payload["session_id"]
                        command_datasets["status"][index] = payload["status"]
                        command_datasets["reject_reason"][index] = payload["reject_reason"]
                        continue
                    if event.kind == "arms" or event.kind == "hands":
                        timestamp, qpos = event.payload
                        ts, values = datasets[event.kind]
                        previous = last_timestamp.get(event.kind)
                        if previous is not None and timestamp < previous:
                            raise ValueError(f"{event.kind} timestamps must be non-decreasing")
                        if previous is not None and timestamp == previous:
                            values[-1] = qpos
                        else:
                            index = ts.shape[0]
                            ts.resize((index + 1,))
                            values.resize((index + 1, values.shape[1]))
                            ts[index] = timestamp
                            values[index] = qpos
                            last_timestamp[event.kind] = timestamp
                        continue
                    if event.kind == "image":
                        name, timestamp, rgb = event.payload
                        ts, jpeg = image_datasets[name]
                        previous = last_timestamp.get(f"image:{name}")
                        if previous is not None and timestamp < previous:
                            raise ValueError(f"camera {name} timestamps must be non-decreasing")
                        from PIL import Image

                        buffer = io.BytesIO()
                        Image.fromarray(rgb, mode="RGB").save(buffer, format="JPEG", quality=JPEG_QUALITY)
                        encoded = np.frombuffer(buffer.getvalue(), dtype=np.uint8)
                        if previous is not None and timestamp == previous:
                            jpeg[-1] = encoded
                        else:
                            index = ts.shape[0]
                            ts.resize((index + 1,))
                            jpeg.resize((index + 1,))
                            ts[index] = timestamp
                            jpeg[index] = encoded
                            last_timestamp[f"image:{name}"] = timestamp
                        continue
                    raise ValueError(f"unknown recorder event: {event.kind}")
            if finish_requested:
                validate_episode_path(path)
                final = self._final_path
                if final is None:
                    raise RuntimeError("final episode path is missing")
                if final.exists():
                    raise FileExistsError(final)
                os.replace(path, final)
                with self._lock:
                    self._published_path = final
        except BaseException as exc:
            with self._lock:
                self._error = exc
                self._state = "failed"
        finally:
            self._done.set()

    def _wait_writer(self) -> None:
        self._done.wait()
        thread = self._thread
        if thread is not None:
            thread.join(timeout=5.0)
        if self._error is not None:
            raise RecorderError(str(self._error)) from self._error

    def finish_episode(self, success: bool = True) -> Path:
        with self._lock:
            if self._state != "recording" or self._queue is None:
                raise RuntimeError("no episode is recording")
            self._state = "finishing"
            self._queue.put(_Event("finish", bool(success)))
        self._wait_writer()
        with self._lock:
            result = self._published_path
        if result is None:
            raise RecorderError("writer finished without publishing an episode")
        with self._lock:
            self._state = "idle"
            self._episode_id = None
            self._partial_path = None
            self._final_path = None
            self._queue = None
            self._thread = None
        return result

    def _end_without_publish(self, kind: str) -> None:
        with self._lock:
            if self._state not in {"recording", "failed"} or self._queue is None:
                return
            self._state = "ending"
            if not self._abort_event.is_set():
                self._queue.put(_Event(kind, None))
        try:
            self._wait_writer()
        except RecorderError:
            if not self._abort_event.is_set():
                raise
        partial = self._partial_path
        if kind == "discard" and partial is not None:
            partial.unlink(missing_ok=True)
        with self._lock:
            self._state = "idle"
            self._episode_id = None
            self._partial_path = None
            self._final_path = None
            self._queue = None
            self._thread = None
            self._error = None

    def discard_episode(self, reason: str = "operator_discard") -> None:
        del reason
        self._end_without_publish("discard")

    def abort_episode(self, reason: str = "interrupted") -> None:
        del reason
        self._end_without_publish("abort")

    def close(self) -> None:
        if self.is_recording or self.error is not None:
            self.abort_episode("recorder_close")


def _resolve_h5(path: str | Path) -> tuple[Path, Path]:
    candidate = Path(path)
    if candidate.is_file() and candidate.suffix in {".h5", ".hdf5"}:
        return candidate, candidate.parent / "dataset_config.json"
    if candidate.is_dir():
        files = sorted(candidate.glob("episode_*.h5"))
        if len(files) == 1:
            return files[0], candidate / "dataset_config.json"
        if not files:
            raise FileNotFoundError(f"no episode_*.h5 under {candidate}")
        raise ValueError(f"multiple episodes under {candidate}; pass one episode file")
    raise FileNotFoundError(candidate)


def _check_monotonic(values: np.ndarray, name: str) -> None:
    if values.dtype != np.dtype("int64"):
        raise ValueError(f"{name} must be int64")
    if values.size > 1 and np.any(np.diff(values) < 0):
        raise ValueError(f"{name} must be non-decreasing")


def _validate_jpeg(value: Any, name: str) -> None:
    payload = bytes(np.asarray(value, dtype=np.uint8).tolist())
    if not payload:
        raise ValueError(f"{name} contains an empty JPEG")
    from PIL import Image

    with Image.open(io.BytesIO(payload)) as image:
        if image.format != "JPEG" or image.size != (IMAGE_WIDTH, IMAGE_HEIGHT) or image.mode not in {"RGB", "L"}:
            raise ValueError(f"{name} is not a {IMAGE_WIDTH}x{IMAGE_HEIGHT} JPEG")
        image.load()


def validate_episode_path(path: str | Path) -> dict[str, Any]:
    h5_path, config_path = _resolve_h5(path)
    if not config_path.is_file():
        raise ValueError("dataset_config.json is missing")
    config = json.loads(config_path.read_text(encoding="utf-8"))
    required_config = {
        "schema_version",
        "robot_config",
        "joint_names",
        "joint_unit",
        "policy_rate_hz",
        "camera_names",
        "image_width",
        "image_height",
        "image_encoding",
        "decoded_color_order",
        "jpeg_quality",
    }
    if set(config) != required_config or config["schema_version"] != SCHEMA_VERSION:
        raise ValueError("dataset_config.json does not match schema-v1")
    if len(config["joint_names"]) != 54 or len(set(config["joint_names"])) != 54:
        raise ValueError("dataset_config joint_names must contain 54 unique names")
    if config["joint_unit"] != JOINT_UNIT or config["policy_rate_hz"] != POLICY_RATE_HZ:
        raise ValueError("dataset_config joint or policy contract mismatch")
    if (config["image_width"], config["image_height"]) != (IMAGE_WIDTH, IMAGE_HEIGHT):
        raise ValueError("dataset_config image dimensions mismatch")
    if config["image_encoding"] != "jpeg" or config["decoded_color_order"] != "RGB" or config["jpeg_quality"] != JPEG_QUALITY:
        raise ValueError("dataset_config image contract mismatch")
    cameras = tuple(config["camera_names"])
    if not cameras or any(name not in CAMERA_NAMES for name in cameras):
        raise ValueError("dataset_config camera_names are invalid")
    success: bool
    with h5py.File(h5_path, "r") as handle:
        if int(handle.attrs.get("schema_version", -1)) != SCHEMA_VERSION:
            raise ValueError("unsupported episode schema version")
        if _as_attr_text(handle.attrs.get("robot_config", "")) != str(config["robot_config"]):
            raise ValueError("episode robot_config does not match dataset_config")
        task = _as_attr_text(handle.attrs.get("task", ""))
        if not task:
            raise ValueError("episode task attribute is missing")
        if "success" not in handle.attrs or not isinstance(handle.attrs["success"], (bool, np.bool_)):
            raise ValueError("episode success attribute must be bool")
        success = bool(handle.attrs["success"])
        if "actions" in handle or "action_type" in handle.attrs:
            raise ValueError("schema-v1 episodes must not contain actions")
        for stream, width in (("arms", 14), ("hands", 40)):
            timestamp = handle[f"observations/{stream}/timestamp_ns"][:]
            qpos = handle[f"observations/{stream}/qpos"][:]
            _check_monotonic(timestamp, f"observations/{stream}/timestamp_ns")
            if np.any(timestamp < 0):
                raise ValueError(f"observations/{stream}/timestamp_ns must be non-negative")
            if qpos.dtype != np.dtype("float32") or qpos.shape != (timestamp.shape[0], width):
                raise ValueError(f"observations/{stream}/qpos must be float32[N,{width}]")
            if qpos.shape[0] == 0 or not np.all(np.isfinite(qpos)):
                raise ValueError(f"observations/{stream}/qpos is empty or non-finite")
        commands = handle["observations/commands"]
        command_timestamp = commands["timestamp_ns"][:]
        command_stamp = commands["stamp_utc_ns"][:]
        command_sequence = commands["sequence"][:]
        command_mask = commands["ready_mask"][:]
        command_hold = commands["hold_mask"][:]
        command_sim = commands["applied_sim_time_ns"][:]
        command_position = commands["position_rad"][:]
        if command_position.dtype != np.dtype("float32") or command_position.shape != (command_timestamp.shape[0], 54):
            raise ValueError("observations/commands/position_rad must be float32[N,54]")
        _check_monotonic(command_timestamp, "observations/commands/timestamp_ns")
        if command_timestamp.shape[0] == 0 or not np.all(np.isfinite(command_position)):
            raise ValueError("observations/commands is empty or non-finite")
        if command_stamp.dtype != np.dtype("int64") or np.any(command_stamp <= 0):
            raise ValueError("observations/commands/stamp_utc_ns is invalid")
        if command_sequence.dtype != np.dtype("uint64") or command_sequence.shape != command_timestamp.shape:
            raise ValueError("observations/commands/sequence is invalid")
        if command_sequence.size > 1 and np.any(np.diff(command_sequence.astype(np.int64)) <= 0):
            raise ValueError("observations/commands/sequence is not strictly increasing")
        if command_mask.dtype != np.dtype("uint8") or command_hold.dtype != np.dtype("uint8"):
            raise ValueError("observations/commands masks have invalid dtype")
        if command_sim.dtype != np.dtype("int64") or np.any(command_sim < 0):
            raise ValueError("observations/commands/applied_sim_time_ns is invalid")
        for name in cameras:
            timestamp = handle[f"images/{name}/timestamp_ns"][:]
            jpeg = handle[f"images/{name}/jpeg"]
            _check_monotonic(timestamp, f"images/{name}/timestamp_ns")
            if np.any(timestamp < 0):
                raise ValueError(f"images/{name}/timestamp_ns must be non-negative")
            if jpeg.shape != timestamp.shape or timestamp.shape[0] == 0:
                raise ValueError(f"images/{name} length mismatch or empty")
            for index in range(timestamp.shape[0]):
                _validate_jpeg(jpeg[index], f"images/{name}/jpeg[{index}]")
    return {
        "episode_path": str(h5_path),
        "schema_version": SCHEMA_VERSION,
        "task": task,
        "success": success,
        "camera_names": list(cameras),
        "valid": True,
    }


__all__ = [
    "EpisodeRecorder",
    "JPEG_QUALITY",
    "JOINT_NAMES",
    "POLICY_RATE_HZ",
    "ROBOT_CONFIG",
    "RecorderError",
    "RecorderQueueOverflow",
    "SCHEMA_VERSION",
    "validate_episode_path",
]
