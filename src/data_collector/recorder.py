"""Bounded asynchronous storage and validation for whole-scene trajectories."""
from __future__ import annotations

from dataclasses import dataclass, field
import hashlib
import json
import os
from pathlib import Path
import queue
import re
import threading
import tempfile
import time
from typing import Any, Mapping

import h5py
import numpy as np

from interfaces.ros_joint_command import JOINT_NAMES, ROBOT_CONFIG


SCHEMA_VERSION = 2
PHYSICS_HZ = 480
STATE_RATE_HZ = 60
TICKS_PER_FRAME = PHYSICS_HZ // STATE_RATE_HZ
JOINT_UNIT = "rad"
_VALIDATION_ROWS = 256
_REWIND_DTYPE = np.dtype([("frame_count", "<i8"), ("monotonic_ns", "<i8")])


class RecorderError(RuntimeError):
    """Recording cannot continue or publish safely."""


class RecorderQueueOverflow(RecorderError):
    """The bounded writer queue cannot accept another complete frame."""


@dataclass(frozen=True)
class _Event:
    kind: str
    payload: Any


@dataclass
class _TruncateRequest:
    frame_count: int
    acknowledged: threading.Event = field(default_factory=threading.Event)
    previous: tuple[int, int, float] | None = None
    error: BaseException | None = None


def _as_attr_text(value: Any) -> str:
    return value.decode("utf-8") if isinstance(value, bytes) else str(value)


def _canonical_json(value: Any) -> str:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False, allow_nan=False)


def _dataset_config() -> dict[str, Any]:
    return {
        "schema_version": SCHEMA_VERSION,
        "robot_config": ROBOT_CONFIG,
        "robot_joint_names": list(JOINT_NAMES),
        "joint_unit": JOINT_UNIT,
        "physics_hz": PHYSICS_HZ,
        "state_rate_hz": STATE_RATE_HZ,
    }


def _episode_stem(episode_id: str | int) -> str:
    text = str(episode_id)
    if text.isdigit():
        return f"episode_{int(text):06d}"
    safe = re.sub(r"[^A-Za-z0-9_.-]+", "_", text).strip("._")
    if not safe:
        raise ValueError("episode_id must contain a filename-safe value")
    return f"episode_{safe}"


def _task_name(manifest: Mapping[str, Any]) -> str:
    task = manifest.get("task") or manifest.get("scene")
    if not isinstance(task, str) or not task:
        raise ValueError("task_manifest must name a task or scene")
    return task


def _field_specs(metadata: Mapping[str, Any]) -> dict[str, tuple[np.dtype, tuple[int, ...]]]:
    return {
        name: (np.dtype(spec["dtype"]), tuple(spec["shape"]))
        for name, spec in metadata["fields"].items()
    }


def _check_clock(tick: int, monotonic_ns: int, sim_time: float,
                 previous: tuple[int, int, float] | None) -> tuple[int, int, float]:
    if tick < 0 or monotonic_ns < 0 or sim_time < 0:
        raise ValueError("tick, monotonic_ns and sim_time must be non-negative")
    if previous is not None:
        if tick - previous[0] != TICKS_PER_FRAME:
            raise ValueError("trajectory ticks must advance by exactly 8 (no missing or duplicate frames)")
        if monotonic_ns <= previous[1]:
            raise ValueError("monotonic_ns must be strictly increasing")
        # Floating-point integration error scales with absolute simulation time.
        tolerance = max(1e-10, 32 * np.finfo(np.float64).eps * max(sim_time, previous[2], 1.0))
        if abs((sim_time - previous[2]) - TICKS_PER_FRAME / PHYSICS_HZ) > tolerance:
            raise ValueError("sim_time must advance by exactly 8/480 seconds")
    return tick, monotonic_ns, sim_time


def _check_frame(frame: Mapping[str, Any], specs: Mapping[str, tuple[np.dtype, tuple[int, ...]]],
                 previous: tuple[int, int, float] | None) -> tuple[int, int, float]:
    if set(frame) != set(specs):
        raise ValueError(f"trajectory fields mismatch: missing={set(specs) - set(frame)}, extra={set(frame) - set(specs)}")
    for name, (dtype, shape) in specs.items():
        if shape and not isinstance(frame[name], np.ndarray):
            raise ValueError(f"trajectory/{name} must be an owned NumPy array")
        value = np.asarray(frame[name])
        if value.dtype != dtype or value.shape != shape:
            raise ValueError(f"trajectory/{name} must have dtype {dtype} and shape {shape}, got {value.dtype} {value.shape}")
        if not np.all(np.isfinite(value)):
            raise ValueError(f"trajectory/{name} contains non-finite values")
    return _check_clock(int(frame["tick"]), int(frame["monotonic_ns"]), float(frame["sim_time"]), previous)


class EpisodeRecorder:
    """Transfer owned scene snapshots through a strictly bounded writer queue.

    After append_frame succeeds the caller must not mutate its arrays. Each queue
    event contains one entire row; no images, commands, or independent streams exist.
    """

    def __init__(self, output_root: str | Path, *, queue_size: int = 256) -> None:
        if isinstance(queue_size, bool) or not isinstance(queue_size, int) or queue_size < 2:
            raise ValueError("queue_size must be an integer of at least 2")
        self.output_root = Path(output_root)
        self.output_root.mkdir(parents=True, exist_ok=True)
        self._queue_size = queue_size
        self._lock = threading.RLock()
        self._state = "idle"
        self._partial_path: Path | None = None
        self._final_path: Path | None = None
        self._queue: queue.Queue[_Event] | None = None
        self._thread: threading.Thread | None = None
        self._done = threading.Event()
        self._abort_event = threading.Event()
        self._abort_reason = ""
        self._published_path: Path | None = None
        self._error: BaseException | None = None
        self._specs: dict[str, tuple[np.dtype, tuple[int, ...]]] = {}
        self._previous: tuple[int, int, float] | None = None

    @property
    def is_recording(self) -> bool:
        with self._lock:
            return self._state == "recording"

    @property
    def is_busy(self) -> bool:
        with self._lock:
            return self._state != "idle"

    @property
    def error(self) -> BaseException | None:
        with self._lock:
            return self._error

    @property
    def partial_path(self) -> Path | None:
        with self._lock:
            return self._partial_path

    def _ensure_dataset_config(self, directory: Path) -> None:
        path = directory / "dataset_config.json"
        expected = _dataset_config()
        if path.exists():
            if json.loads(path.read_text(encoding="utf-8")) != expected:
                raise ValueError("dataset_config.json does not match schema-v2; use a different dataset directory")
            return
        # Publish a fully written config without replacing a concurrent contract.
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=directory,
                                         prefix=".dataset_config.", suffix=".tmp", delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(_canonical_json(expected) + "\n")
        try:
            try:
                os.link(temporary, path)
            except FileExistsError:
                if json.loads(path.read_text(encoding="utf-8")) != expected:
                    raise ValueError("dataset_config.json does not match schema-v2")
        finally:
            temporary.unlink(missing_ok=True)

    def start_episode(self, episode_id: str | int, task_manifest: Mapping[str, Any], *,
                      model_bytes: bytes, metadata: dict[str, Any],
                      output_dir: str | Path | None = None) -> None:
        from data_collector.trajectory import load_model

        with self._lock:
            if self._state != "idle":
                raise RuntimeError("recorder is busy with another episode")
            if not isinstance(model_bytes, bytes) or not model_bytes:
                raise ValueError("model_bytes must be nonempty bytes")
            # Detach metadata from caller mutation and reject non-JSON/non-finite values.
            metadata_json = _canonical_json(metadata)
            stored_metadata = json.loads(metadata_json)
            if _canonical_json(task_manifest) != _canonical_json(stored_metadata.get("task_manifest")):
                raise ValueError("task_manifest does not match model metadata")
            task = _task_name(task_manifest)
            load_model(model_bytes, stored_metadata)
            if stored_metadata["robot_joint_names"] != list(JOINT_NAMES):
                raise ValueError("model metadata robot joint order does not match the dataset contract")
            self._specs = _field_specs(stored_metadata)
            directory = self.output_root if output_dir is None else Path(output_dir)
            directory.mkdir(parents=True, exist_ok=True)
            self._ensure_dataset_config(directory)
            stem = _episode_stem(episode_id)
            partial, final = directory / f"{stem}.partial.h5", directory / f"{stem}.h5"
            if partial.exists() or final.exists():
                raise FileExistsError(final if final.exists() else partial)
            with h5py.File(partial, "x") as handle:
                handle.attrs.update(schema_version=SCHEMA_VERSION, robot_config=ROBOT_CONFIG,
                                    task=task, success=False, complete=False)
                model_group = handle.create_group("model")
                model_group.create_dataset("mjb", data=np.frombuffer(model_bytes, dtype=np.uint8))
                model_group.create_dataset("metadata", data=metadata_json, dtype=h5py.string_dtype("utf-8"))
                model_group.attrs["model_sha256"] = stored_metadata["model_sha256"]
                model_group.attrs["metadata_sha256"] = hashlib.sha256(metadata_json.encode("utf-8")).hexdigest()
                trajectory = handle.create_group("trajectory")
                for name, (dtype, shape) in self._specs.items():
                    trajectory.create_dataset(name, shape=(0, *shape), maxshape=(None, *shape),
                                              dtype=dtype, chunks=(64, *shape))
            self._partial_path, self._final_path = partial, final
            self._queue = queue.Queue(maxsize=self._queue_size)
            self._previous = None
            self._done.clear()
            self._abort_event.clear()
            self._abort_reason = ""
            self._published_path = None
            self._error = None
            self._state = "recording"
            self._thread = threading.Thread(target=self._writer, name="spd-hdf5-writer", daemon=True)
            try:
                self._thread.start()
            except BaseException as exc:
                self._error = exc
                self._thread = None
                self._state = "failed"
                self._done.set()
                raise

    def append_frame(self, frame: dict[str, Any]) -> None:
        with self._lock:
            if self._error is not None:
                raise RecorderError(str(self._error)) from self._error
            if self._state != "recording" or self._queue is None:
                raise RuntimeError("no episode is recording")
            try:
                previous = _check_frame(frame, self._specs, self._previous)
                # Copy only the small mapping, never its owned snapshot arrays.
                self._queue.put_nowait(_Event("frame", dict(frame)))
                self._previous = previous
            except (ValueError, TypeError, OverflowError, queue.Full) as exc:
                error = RecorderQueueOverflow("schema-v2 writer queue overflow") if isinstance(exc, queue.Full) else exc
                self._error = error
                self._state = "failed"
                self._abort_reason = str(error)
                self._abort_event.set()
                if error is exc:
                    raise
                raise error from exc

    def truncate_frames(self, frame_count: int) -> None:
        """Retain a written prefix and acknowledge its flush before returning.

        Call only from the serialized collection control worker with physics
        paused. Append and finish are rejected until the mutation completes.
        Rewind events identify boundaries before zero-based ``frame_count``;
        their monotonic timestamps are real wall-clock observations, not rewound.
        """
        if isinstance(frame_count, (bool, np.bool_)) or not isinstance(frame_count, (int, np.integer)) or frame_count < 0:
            raise ValueError("frame_count must be a non-negative integer")
        request = _TruncateRequest(int(frame_count))
        with self._lock:
            if self._error is not None:
                raise RecorderError(str(self._error)) from self._error
            if self._state != "recording" or self._queue is None:
                raise RuntimeError("no episode is recording")
            self._state = "truncating"
        # Never hold the state lock while waiting for queue space or the writer:
        # its failure path needs that lock before it can signal completion.
        self._put_end(_Event("truncate", request))
        while not request.acknowledged.wait(timeout=0.1):
            if self._done.is_set():
                break
        with self._lock:
            if self._error is not None:
                raise RecorderError(str(self._error)) from self._error
            if not request.acknowledged.is_set():
                self._state = "failed"
                self._error = RecorderError("writer stopped before acknowledging truncation")
                raise self._error
            self._state = "recording"
            if request.error is not None:
                raise request.error
            self._previous = request.previous

    def _writer(self) -> None:
        assert self._partial_path is not None and self._queue is not None
        path = self._partial_path
        finish_requested = False
        pending_truncate: _TruncateRequest | None = None
        try:
            with h5py.File(path, "r+") as handle:
                datasets = dict(handle["trajectory"].items())
                index = 0
                while True:
                    try:
                        event = self._queue.get(timeout=0.1)
                    except queue.Empty:
                        if self._abort_event.is_set():
                            break
                        continue
                    if event.kind == "frame":
                        # Roll back every dataset extent if an individual write fails.
                        try:
                            for name, dataset in datasets.items():
                                dataset.resize(index + 1, axis=0)
                                dataset[index] = event.payload[name]
                        except BaseException:
                            for dataset in datasets.values():
                                dataset.resize(index, axis=0)
                            raise
                        index += 1
                    elif event.kind == "truncate":
                        pending_truncate = event.payload
                        count = pending_truncate.frame_count
                        if count > index:
                            pending_truncate.error = ValueError("frame_count exceeds the written trajectory prefix")
                        else:
                            previous = None if count == 0 else (
                                int(datasets["tick"][count - 1]),
                                int(datasets["monotonic_ns"][count - 1]),
                                float(datasets["sim_time"][count - 1]),
                            )
                            for dataset in datasets.values():
                                dataset.resize(count, axis=0)
                            events = handle.require_group("collection_events")
                            if "rewind" not in events:
                                events.create_dataset("rewind", shape=(0,), maxshape=(None,),
                                                      dtype=_REWIND_DTYPE, chunks=(64,))
                            rewinds = events["rewind"]
                            # Counts are sorted: every rewind removes the events
                            # belonging to the suffix that it discards.
                            retained = rewinds.shape[0]
                            while retained and int(rewinds[retained - 1]["frame_count"]) > count:
                                retained -= 1
                            rewinds.resize(retained + 1, axis=0)
                            rewinds[retained] = (count, time.monotonic_ns())
                            handle.flush()
                            index = count
                            pending_truncate.previous = previous
                        pending_truncate.acknowledged.set()
                        pending_truncate = None
                    elif event.kind == "finish":
                        if not self._abort_event.is_set():
                            handle.attrs["success"] = event.payload
                            finish_requested = True
                        break
                    elif event.kind in {"abort", "discard"}:
                        break
                    else:
                        raise ValueError(f"unknown recorder event: {event.kind}")
                if self._abort_reason:
                    handle.attrs["abort_reason"] = self._abort_reason
                handle.flush()
            if finish_requested:
                validate_episode_path(path, allow_partial=True)
                # Serialize completion with abort requests; no abort can publish.
                with self._lock:
                    if self._abort_event.is_set():
                        with h5py.File(path, "r+") as handle:
                            handle.attrs["success"] = False
                            handle.attrs["complete"] = False
                            handle.attrs["abort_reason"] = self._abort_reason
                            handle.flush()
                        return
                    final = self._final_path
                    if final is None:
                        raise RecorderError("final episode path is missing")
                    if final.exists():
                        raise FileExistsError(final)
                    with h5py.File(path, "r+") as handle:
                        handle.attrs["complete"] = True
                        handle.flush()
                    os.replace(path, final)
                    self._published_path = final
        except BaseException as exc:
            with self._lock:
                if self._error is None:
                    self._error = exc
                self._state = "failed"
            # Publication failure must not leave a successfully completed partial.
            try:
                with h5py.File(path, "r+") as handle:
                    handle.attrs["complete"] = False
                    handle.attrs["success"] = False
                    handle.attrs["abort_reason"] = str(exc)
                    handle.flush()
            except Exception:
                pass  # Preserve the original failure, including an unusable partial.
        finally:
            if pending_truncate is not None:
                pending_truncate.error = self._error
                pending_truncate.acknowledged.set()
            self._done.set()

    def _put_end(self, event: _Event) -> None:
        assert self._queue is not None
        # A failed writer needs the state lock before signalling completion.
        while not self._done.is_set():
            try:
                self._queue.put(event, timeout=0.1)
                return
            except queue.Full:
                continue

    def _wait_writer(self) -> None:
        self._done.wait()
        if self._thread is not None:
            self._thread.join()
        if self._error is not None:
            raise RecorderError(str(self._error)) from self._error

    def _release(self) -> None:
        self._state = "idle"
        self._final_path = None
        self._queue = None
        self._thread = None
        self._previous = None
        self._specs = {}

    def finish_episode(self, success: bool = True) -> Path:
        if not isinstance(success, (bool, np.bool_)):
            raise ValueError("success must be bool")
        with self._lock:
            if self._error is not None:
                raise RecorderError(str(self._error)) from self._error
            if self._state != "recording" or self._queue is None:
                raise RuntimeError("no episode is recording")
            self._state = "finishing"
        self._put_end(_Event("finish", bool(success)))
        self._wait_writer()
        with self._lock:
            result = self._published_path
            if result is None:
                raise RecorderError("writer finished without publishing an episode")
            self._partial_path = None
            self._release()
            return result

    def _end_without_publish(self, kind: str, reason: str) -> None:
        with self._lock:
            if self._state == "idle" or self._queue is None:
                return
            if self._state == "truncating":
                raise RuntimeError("cannot end an episode during truncation")
            self._state = "ending"
            self._abort_reason = reason
            self._abort_event.set()
        self._put_end(_Event(kind, None))
        try:
            self._wait_writer()
        except RecorderError:
            pass  # Explicit discard/abort still joins and releases a failed writer.
        with self._lock:
            partial = self._partial_path
            if kind == "discard" and partial is not None:
                partial.unlink(missing_ok=True)
                self._partial_path = None
            self._release()

    def discard_episode(self) -> None:
        self._end_without_publish("discard", "operator_discard")

    def abort_episode(self, reason: str = "interrupted") -> None:
        self._end_without_publish("abort", str(reason))

    def close(self) -> None:
        if self.is_busy:
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


def validate_episode_path(path: str | Path, *, allow_partial: bool = False) -> dict[str, Any]:
    """Validate bounded row chunks; validity never confers episode completion."""
    from data_collector.trajectory import load_model

    h5_path, config_path = _resolve_h5(path)
    partial = h5_path.name.endswith(".partial.h5")
    if partial and not allow_partial:
        raise ValueError("partial episodes are not completed episodes")
    if not config_path.is_file():
        raise ValueError("dataset_config.json is missing")
    if json.loads(config_path.read_text(encoding="utf-8")) != _dataset_config():
        raise ValueError("dataset_config.json does not match schema-v2")
    with h5py.File(h5_path, "r") as handle:
        schema = handle.attrs.get("schema_version")
        if not isinstance(schema, (int, np.integer)) or int(schema) != SCHEMA_VERSION:
            raise ValueError("unsupported episode schema version")
        allowed_attrs = {"schema_version", "robot_config", "task", "success", "complete", "abort_reason"}
        if set(handle.attrs) - allowed_attrs:
            raise ValueError("unsupported schema-v2 root attributes")
        if _as_attr_text(handle.attrs.get("robot_config", "")) != ROBOT_CONFIG:
            raise ValueError("episode robot_config does not match dataset_config")
        for name in ("success", "complete"):
            if not isinstance(handle.attrs.get(name), (bool, np.bool_)):
                raise ValueError(f"episode {name} attribute must be bool")
        complete, success = bool(handle.attrs["complete"]), bool(handle.attrs["success"])
        if partial and complete:
            raise ValueError("a partial file cannot be marked complete")
        if not complete and not allow_partial:
            raise ValueError("episode is incomplete")
        if complete and "abort_reason" in handle.attrs:
            raise ValueError("an aborted episode cannot be complete")
        if set(handle) - {"model", "trajectory", "collection_events"} or not {"model", "trajectory"} <= set(handle):
            raise ValueError("schema-v2 requires model and trajectory, with optional collection_events")
        model_group = handle["model"]
        if not isinstance(model_group, h5py.Group) or set(model_group) != {"mjb", "metadata"}:
            raise ValueError("model must contain only mjb and metadata")
        if set(model_group.attrs) != {"model_sha256", "metadata_sha256"}:
            raise ValueError("model attributes must contain the model and metadata hashes")
        mjb, metadata_dataset = model_group["mjb"], model_group["metadata"]
        if not isinstance(mjb, h5py.Dataset) or mjb.dtype != np.dtype("uint8") or mjb.ndim != 1 or mjb.size == 0:
            raise ValueError("model/mjb must be nonempty uint8[bytes]")
        string_info = h5py.check_string_dtype(metadata_dataset.dtype) if isinstance(metadata_dataset, h5py.Dataset) else None
        if string_info is None or string_info.encoding != "utf-8" or metadata_dataset.shape != ():
            raise ValueError("model/metadata must be scalar UTF-8 JSON")
        metadata_json = _as_attr_text(metadata_dataset[()])
        metadata = json.loads(metadata_json)
        if _canonical_json(metadata) != metadata_json:
            raise ValueError("model metadata is not canonical JSON")
        if hashlib.sha256(metadata_json.encode("utf-8")).hexdigest() != _as_attr_text(model_group.attrs.get("metadata_sha256", "")):
            raise ValueError("model metadata SHA256 mismatch")
        model_bytes = mjb[:].tobytes()
        model_hash = hashlib.sha256(model_bytes).hexdigest()
        if model_hash != _as_attr_text(model_group.attrs.get("model_sha256", "")):
            raise ValueError("model MJB SHA256 mismatch")
        model = load_model(model_bytes, metadata, expected_model_sha256=model_hash)
        if metadata["robot_joint_names"] != list(JOINT_NAMES):
            raise ValueError("model robot joint order does not match dataset_config")
        task = _as_attr_text(handle.attrs.get("task", ""))
        if task != _task_name(metadata["task_manifest"]):
            raise ValueError("episode task does not match model task_manifest")
        specs = _field_specs(metadata)
        trajectory = handle["trajectory"]
        if not isinstance(trajectory, h5py.Group) or set(trajectory) != set(specs):
            raise ValueError("trajectory field set does not match model metadata")
        tick_dataset = trajectory["tick"]
        if not isinstance(tick_dataset, h5py.Dataset) or tick_dataset.ndim != 1:
            raise ValueError("trajectory/tick must be a one-dimensional dataset")
        frames = tick_dataset.shape[0]
        if frames == 0:
            raise ValueError("trajectory must contain at least one frame")
        for name, (dtype, shape) in specs.items():
            dataset = trajectory[name]
            if not isinstance(dataset, h5py.Dataset) or dataset.dtype != dtype or dataset.shape != (frames, *shape):
                raise ValueError(f"trajectory/{name} dtype, shape or row count mismatch")
        if "collection_events" in handle:
            events = handle["collection_events"]
            if not isinstance(events, h5py.Group) or set(events) != {"rewind"} or events.attrs:
                raise ValueError("collection_events must contain only rewind, with no attributes")
            rewinds = events["rewind"]
            if (not isinstance(rewinds, h5py.Dataset) or rewinds.ndim != 1
                    or rewinds.dtype != _REWIND_DTYPE or rewinds.attrs):
                raise ValueError("collection_events/rewind must be (frame_count:int64, monotonic_ns:int64) rows")
            previous_count, previous_time = -1, -1
            for start in range(0, rewinds.shape[0], _VALIDATION_ROWS):
                for event in rewinds[start:start + _VALIDATION_ROWS]:
                    count, timestamp = int(event["frame_count"]), int(event["monotonic_ns"])
                    if count < 0 or count > frames or count < previous_count:
                        raise ValueError("rewind frame counts must be ordered retained-prefix boundaries")
                    if timestamp < 0 or timestamp <= previous_time:
                        raise ValueError("rewind monotonic timestamps must be non-negative and strictly increasing")
                    previous_count, previous_time = count, timestamp
        qpos_indices = metadata["robot_qpos_indices"]
        qvel_indices = metadata["robot_qvel_indices"]
        if specs["qpos"][1] != (model.nq,) or specs["qvel"][1] != (model.nv,):
            raise ValueError("trajectory generalized coordinates disagree with model dimensions")
        previous = None
        for start in range(0, frames, _VALIDATION_ROWS):
            stop = min(start + _VALIDATION_ROWS, frames)
            chunk = {name: trajectory[name][start:stop] for name in specs}
            for name, values in chunk.items():
                if not np.all(np.isfinite(values)):
                    raise ValueError(f"trajectory/{name} contains non-finite values")
            for index in range(stop - start):
                previous = _check_clock(int(chunk["tick"][index]), int(chunk["monotonic_ns"][index]),
                                        float(chunk["sim_time"][index]), previous)
            if not np.array_equal(chunk["robot_qpos"], chunk["qpos"][:, qpos_indices]):
                raise ValueError("robot_qpos is inconsistent with full-scene qpos")
            if not np.array_equal(chunk["robot_qvel"], chunk["qvel"][:, qvel_indices]):
                raise ValueError("robot_qvel is inconsistent with full-scene qvel")
            if "hand_object" in chunk and not np.array_equal(chunk["hand_contact"], np.any(chunk["hand_object"], axis=2)):
                raise ValueError("hand_contact is inconsistent with hand_object")
            if "hand_object" not in chunk and np.any(chunk["hand_contact"]):
                raise ValueError("hand_contact must be false when the scene has no task objects")
    return {
        "episode_path": str(h5_path), "schema_version": SCHEMA_VERSION,
        "task": task, "success": success, "frames": frames, "complete": complete,
        "model_sha256": model_hash, "valid": True,
    }


__all__ = [
    "EpisodeRecorder", "JOINT_NAMES", "PHYSICS_HZ", "STATE_RATE_HZ", "ROBOT_CONFIG",
    "RecorderError", "RecorderQueueOverflow", "SCHEMA_VERSION", "validate_episode_path",
]
