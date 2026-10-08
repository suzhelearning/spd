"""Read-only schema-v2 episode indexing and kinematic frame serving."""
from __future__ import annotations

from collections import OrderedDict
from dataclasses import dataclass
import hashlib
import json
import os
from pathlib import Path
import threading
from typing import Any, Mapping

import h5py
import mujoco
import numpy as np

from data_collector.recorder import JOINT_NAMES, PHYSICS_HZ, ROBOT_CONFIG, SCHEMA_VERSION, STATE_RATE_HZ
from data_collector.trajectory import TrajectoryError, load_model

from .scene import SceneError, export_scene


SAMPLE_RATE = STATE_RATE_HZ
CHUNK_FRAMES = 240
_FRAME_STRIDE = PHYSICS_HZ // STATE_RATE_HZ
_MAX_CHUNKS = 2
_ALLOWED_ROOT_ATTRS = frozenset(("schema_version", "robot_config", "task", "success", "complete", "abort_reason"))


class ReplayError(ValueError):
    """An episode cannot be served without violating the replay contract."""


@dataclass(frozen=True)
class _FileIdentity:
    device: int
    inode: int
    size: int
    mtime_ns: int
    ctime_ns: int


@dataclass(frozen=True)
class _Descriptor:
    metadata: dict[str, Any]
    frames: int
    task: str
    scene: str
    title: str
    model_sha256: str


def _expected_dataset_config() -> dict[str, Any]:
    return {
        "schema_version": SCHEMA_VERSION,
        "robot_config": ROBOT_CONFIG,
        "robot_joint_names": list(JOINT_NAMES),
        "joint_unit": "rad",
        "physics_hz": PHYSICS_HZ,
        "state_rate_hz": STATE_RATE_HZ,
    }


def _source_identity(path: Path) -> _FileIdentity:
    try:
        status = path.stat()
    except OSError as exc:
        raise ReplayError(f"无法读取轨迹文件: {path}: {exc}") from exc
    if not path.is_file():
        raise ReplayError(f"轨迹不是普通文件: {path}")
    return _FileIdentity(
        device=int(status.st_dev),
        inode=int(status.st_ino),
        size=int(status.st_size),
        mtime_ns=int(status.st_mtime_ns),
        ctime_ns=int(status.st_ctime_ns),
    )


def _episode_id(path: Path, identity: _FileIdentity) -> str:
    payload = "\0".join((
        str(path), str(identity.device), str(identity.inode), str(identity.size),
        str(identity.mtime_ns), str(identity.ctime_ns),
    )).encode("utf-8", "surrogateescape")
    return hashlib.sha256(payload).hexdigest()


def _display_path(path: Path, root: Path) -> str:
    try:
        return path.relative_to(root).as_posix()
    except ValueError:
        return str(path)


def _text(value: Any, label: str) -> str:
    if isinstance(value, bytes):
        try:
            return value.decode("utf-8")
        except UnicodeDecodeError as exc:
            raise ReplayError(f"{label} 不是 UTF-8 文本") from exc
    if isinstance(value, np.bytes_):
        try:
            return bytes(value).decode("utf-8")
        except UnicodeDecodeError as exc:
            raise ReplayError(f"{label} 不是 UTF-8 文本") from exc
    if isinstance(value, str):
        return value
    raise ReplayError(f"{label} 必须是文本")


def _integer(value: Any, label: str) -> int:
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, (int, np.integer)):
        raise ReplayError(f"{label} 必须是整数")
    return int(value)


def _boolean(value: Any, label: str) -> bool:
    if not isinstance(value, (bool, np.bool_)):
        raise ReplayError(f"{label} 必须是布尔值")
    return bool(value)


def _canonical_json(value: Any, label: str) -> str:
    try:
        return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False, allow_nan=False)
    except (TypeError, ValueError) as exc:
        raise ReplayError(f"{label} 不是规范 JSON") from exc


def _sha256_text(value: str, label: str) -> str:
    if len(value) != 64 or any(character not in "0123456789abcdef" for character in value):
        raise ReplayError(f"{label} 必须是小写 SHA-256")
    return value


def _load_dataset_config(path: Path) -> None:
    config_path = path.parent / "dataset_config.json"
    if not config_path.is_file():
        raise ReplayError("缺少 dataset_config.json")
    try:
        document = json.loads(config_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ReplayError(f"无法读取 dataset_config.json: {exc}") from exc
    if document != _expected_dataset_config():
        raise ReplayError("dataset_config.json 不符合 schema-v2 数据集配置")


def _field_spec(value: Any, name: str) -> tuple[np.dtype[Any], tuple[int, ...]]:
    if not isinstance(value, dict) or set(value) != {"dtype", "shape"}:
        raise ReplayError(f"model metadata fields.{name} 必须包含 dtype 和 shape")
    dtype_name, shape = value["dtype"], value["shape"]
    if not isinstance(dtype_name, str) or not isinstance(shape, list):
        raise ReplayError(f"model metadata fields.{name} 格式无效")
    try:
        dtype = np.dtype(dtype_name)
    except TypeError as exc:
        raise ReplayError(f"model metadata fields.{name} dtype 无效") from exc
    if dtype.kind not in "biuf":
        raise ReplayError(f"model metadata fields.{name} dtype 不受支持")
    converted_shape: list[int] = []
    for dimension in shape:
        if isinstance(dimension, bool) or not isinstance(dimension, int) or dimension < 0:
            raise ReplayError(f"model metadata fields.{name} shape 无效")
        converted_shape.append(int(dimension))
    return dtype, tuple(converted_shape)


def _validate_time_grid(ticks: h5py.Dataset, times: h5py.Dataset, frames: int) -> None:
    """Validate the source's 60 Hz tick/simulation-time grid in bounded reads."""
    previous_tick: int | None = None
    previous_time: float | None = None
    seconds_per_frame = 1.0 / SAMPLE_RATE
    epsilon = np.finfo(np.float64).eps
    for start in range(0, frames, CHUNK_FRAMES):
        stop = min(start + CHUNK_FRAMES, frames)
        tick_values = np.asarray(ticks[start:stop])
        time_values = np.asarray(times[start:stop], dtype=np.float64)
        if tick_values.shape != (stop - start,) or time_values.shape != (stop - start,):
            raise ReplayError("trajectory time-grid 数据集长度在读取期间发生变化")
        if np.any(tick_values < 0) or np.any(time_values < 0) or not np.all(np.isfinite(time_values)):
            raise ReplayError("trajectory time-grid 含有非有限值或负值")
        if previous_tick is not None:
            if int(tick_values[0]) - previous_tick != _FRAME_STRIDE:
                raise ReplayError("trajectory tick 不是规则的 60 Hz 网格")
            delta = float(time_values[0]) - previous_time  # type: ignore[operator]
            tolerance = max(1e-10, 32.0 * epsilon * max(float(time_values[0]), previous_time, 1.0))
            if abs(delta - seconds_per_frame) > tolerance:
                raise ReplayError("trajectory sim_time 不是规则的 60 Hz 网格")
        if len(tick_values) > 1 and np.any(np.diff(tick_values) != _FRAME_STRIDE):
            raise ReplayError("trajectory tick 不是规则的 60 Hz 网格")
        if len(time_values) > 1:
            left, right = time_values[:-1], time_values[1:]
            tolerance = np.maximum(1e-10, 32.0 * epsilon * np.maximum(np.maximum(left, right), 1.0))
            if np.any(np.abs((right - left) - seconds_per_frame) > tolerance):
                raise ReplayError("trajectory sim_time 不是规则的 60 Hz 网格")
        previous_tick, previous_time = int(tick_values[-1]), float(time_values[-1])


def _episode_labels(metadata: Mapping[str, Any], root_task: str) -> tuple[str, str, str]:
    task_manifest = metadata.get("task_manifest")
    scene_manifest = metadata.get("scene_manifest")
    if not isinstance(task_manifest, dict) or not isinstance(scene_manifest, dict):
        raise ReplayError("model metadata 缺少 task_manifest 或 scene_manifest")
    task = task_manifest.get("task") or task_manifest.get("scene")
    scene = task_manifest.get("scene") or scene_manifest.get("scene")
    if not isinstance(task, str) or not task:
        raise ReplayError("task_manifest 必须包含 task 或 scene 名称")
    if not isinstance(scene, str) or not scene:
        raise ReplayError("task_manifest 必须包含 scene 名称")
    if root_task != task:
        raise ReplayError("episode task 属性与 model metadata 不一致")
    title = task_manifest.get("task_title_zh") or task_manifest.get("title") or task
    if not isinstance(title, str) or not title:
        raise ReplayError("task 标题必须是非空文本")
    return task, scene, title


def _descriptor(handle: h5py.File, path: Path) -> _Descriptor:
    if path.name.endswith(".partial.h5"):
        raise ReplayError("partial 轨迹不是已完成 episode")
    root_attrs = set(handle.attrs)
    if root_attrs - _ALLOWED_ROOT_ATTRS:
        raise ReplayError("episode 包含不支持的 schema-v2 根属性")
    if _integer(handle.attrs.get("schema_version"), "schema_version") != SCHEMA_VERSION:
        raise ReplayError("不支持的 episode schema_version")
    if _text(handle.attrs.get("robot_config", ""), "robot_config") != ROBOT_CONFIG:
        raise ReplayError("episode robot_config 与 schema-v2 配置不一致")
    complete = _boolean(handle.attrs.get("complete"), "episode complete")
    _boolean(handle.attrs.get("success"), "episode success")
    if not complete:
        raise ReplayError("episode 尚未完成")
    if "abort_reason" in handle.attrs:
        raise ReplayError("带 abort_reason 的 episode 不能标记为完成")
    root_task = _text(handle.attrs.get("task", ""), "episode task")
    if not root_task:
        raise ReplayError("episode task 不能为空")
    if set(handle) - {"model", "trajectory", "collection_events"} or not {"model", "trajectory"} <= set(handle):
        raise ReplayError("schema-v2 episode 必须包含 model 和 trajectory")

    model_group = handle["model"]
    if not isinstance(model_group, h5py.Group) or set(model_group) != {"mjb", "metadata"}:
        raise ReplayError("model 必须仅包含 mjb 与 metadata")
    if set(model_group.attrs) != {"model_sha256", "metadata_sha256"}:
        raise ReplayError("model 必须包含模型和 metadata 哈希")
    mjb = model_group["mjb"]
    metadata_dataset = model_group["metadata"]
    if not isinstance(mjb, h5py.Dataset) or mjb.dtype != np.dtype("uint8") or mjb.ndim != 1 or not mjb.size:
        raise ReplayError("model/mjb 必须是非空 uint8 向量")
    if not isinstance(metadata_dataset, h5py.Dataset):
        raise ReplayError("model/metadata 必须是 UTF-8 JSON")
    string_info = h5py.check_string_dtype(metadata_dataset.dtype)
    if string_info is None or string_info.encoding != "utf-8" or metadata_dataset.shape != ():
        raise ReplayError("model/metadata 必须是标量 UTF-8 JSON")
    metadata_text = _text(metadata_dataset[()], "model/metadata")
    try:
        metadata = json.loads(metadata_text)
    except json.JSONDecodeError as exc:
        raise ReplayError("model/metadata 不是 JSON") from exc
    if not isinstance(metadata, dict) or _canonical_json(metadata, "model/metadata") != metadata_text:
        raise ReplayError("model/metadata 不是规范 JSON mapping")
    metadata_hash = hashlib.sha256(metadata_text.encode("utf-8")).hexdigest()
    if metadata_hash != _sha256_text(_text(model_group.attrs.get("metadata_sha256", ""), "metadata_sha256"), "metadata_sha256"):
        raise ReplayError("model metadata SHA-256 不匹配")
    model_hash = _sha256_text(_text(model_group.attrs.get("model_sha256", ""), "model_sha256"), "model_sha256")
    if metadata.get("model_sha256") != model_hash:
        raise ReplayError("model metadata 的 model_sha256 不匹配")
    if metadata.get("snapshot_format") != "mujoco_mjb":
        raise ReplayError("不支持的模型快照格式")
    recorded_version = metadata.get("mujoco_version")
    if recorded_version != mujoco.mj_versionString():
        raise ReplayError(
            f"MJB 需要 MuJoCo {recorded_version!r}，当前安装的是 {mujoco.mj_versionString()!r}"
        )
    if metadata.get("physics_hz") != PHYSICS_HZ or metadata.get("state_rate_hz") != SAMPLE_RATE:
        raise ReplayError("model metadata 的物理或状态采样率不符合 480/60 Hz")

    fields = metadata.get("fields")
    if not isinstance(fields, dict) or not fields:
        raise ReplayError("model metadata 缺少 fields")
    specs = {name: _field_spec(spec, str(name)) for name, spec in fields.items() if isinstance(name, str)}
    if len(specs) != len(fields) or not {"tick", "sim_time", "qpos"} <= set(specs):
        raise ReplayError("model metadata fields 不完整")
    if specs["tick"] != (np.dtype("int64"), ()) or specs["sim_time"] != (np.dtype("float64"), ()):
        raise ReplayError("trajectory tick/sim_time 字段不符合 schema-v2")

    trajectory = handle["trajectory"]
    if not isinstance(trajectory, h5py.Group) or set(trajectory) != set(specs):
        raise ReplayError("trajectory 字段集合与 model metadata 不一致")
    tick_dataset = trajectory["tick"]
    time_dataset = trajectory["sim_time"]
    if not isinstance(tick_dataset, h5py.Dataset) or tick_dataset.ndim != 1:
        raise ReplayError("trajectory/tick 必须是一维数据集")
    frames = int(tick_dataset.shape[0])
    if frames < 1:
        raise ReplayError("trajectory 至少需要一帧")
    for name, (dtype, shape) in specs.items():
        dataset = trajectory[name]
        if not isinstance(dataset, h5py.Dataset) or dataset.dtype != dtype or dataset.shape != (frames, *shape):
            raise ReplayError(f"trajectory/{name} 的 dtype、shape 或帧数不匹配")
    _validate_time_grid(tick_dataset, time_dataset, frames)
    task, scene, title = _episode_labels(metadata, root_task)
    return _Descriptor(
        metadata=metadata,
        frames=frames,
        task=task,
        scene=scene,
        title=title,
        model_sha256=model_hash,
    )


def _scan_candidates(root: Path) -> tuple[list[Path], list[dict[str, str]]]:
    candidates: list[Path] = []
    skipped: list[dict[str, str]] = []

    def report_walk_error(error: OSError) -> None:
        filename = Path(error.filename) if error.filename else root
        skipped.append({"path": _display_path(filename, root), "error": str(error)})

    for directory, directories, filenames in os.walk(root, topdown=True, onerror=report_walk_error, followlinks=False):
        directories.sort()
        for filename in sorted(filenames):
            if not filename.endswith(".h5") or filename.endswith(".render.h5"):
                continue
            candidates.append(Path(directory) / filename)
    return candidates, skipped


def scan_directory(directory: str | Path) -> tuple[Path, list[dict[str, Any]], dict[str, Path], list[dict[str, str]]]:
    """Index completed schema-v2 episodes without reading any MJB payload bytes."""
    try:
        root = Path(directory).expanduser().resolve(strict=True)
    except (OSError, RuntimeError, TypeError) as exc:
        raise ReplayError(f"目录无法解析: {directory}: {exc}") from exc
    if not root.is_dir():
        raise ReplayError(f"不是目录: {root}")

    candidates, skipped = _scan_candidates(root)
    summaries: list[dict[str, Any]] = []
    paths: dict[str, Path] = {}
    for candidate in candidates:
        relative_path = _display_path(candidate, root)
        try:
            resolved = candidate.resolve(strict=True)
            identity = _source_identity(resolved)
            _load_dataset_config(resolved)
            with h5py.File(resolved, "r") as handle:
                descriptor = _descriptor(handle, resolved)
            if _source_identity(resolved) != identity:
                raise ReplayError("轨迹在扫描期间发生变化")
            identifier = _episode_id(resolved, identity)
            if identifier in paths:
                raise ReplayError("轨迹标识冲突")
            summary = {
                "id": identifier,
                "relative_path": relative_path,
                "title": descriptor.title,
                "scene": descriptor.scene,
                "task": descriptor.task,
                "frames": descriptor.frames,
                "duration": (descriptor.frames - 1) / SAMPLE_RATE,
                "sample_rate": SAMPLE_RATE,
                "size_bytes": identity.size,
            }
            summaries.append(summary)
            paths[identifier] = resolved
        except (ReplayError, OSError, RuntimeError, TypeError, UnicodeDecodeError, ValueError) as exc:
            skipped.append({"path": relative_path, "error": str(exc)})
    summaries.sort(key=lambda summary: summary["relative_path"])
    skipped.sort(key=lambda entry: entry["path"])
    return root, summaries, paths, skipped


class ReplayEpisode:
    """One stat-bound, lazily chunked selected episode.

    The source HDF5 is opened only for bounded reads.  The model and one
    ``MjData`` are private to this episode and guarded by ``_lock`` because
    kinematic arrays are mutated for every requested frame block.
    """

    def __init__(self, path: Path, summary: dict[str, Any]) -> None:
        try:
            self.path = Path(path).expanduser().resolve(strict=True)
        except (OSError, RuntimeError, TypeError) as exc:
            raise ReplayError(f"轨迹文件无法解析: {path}: {exc}") from exc
        self._identity = _source_identity(self.path)
        self._summary = dict(summary)
        identifier = self._summary.get("id")
        if not isinstance(identifier, str) or identifier != _episode_id(self.path, self._identity):
            raise ReplayError("轨迹自目录扫描后已变化；请重新扫描")
        self._lock = threading.RLock()
        self._loaded = False
        self._descriptor: _Descriptor | None = None
        self._model: Any | None = None
        self._data: Any | None = None
        self._body_ids: np.ndarray | None = None
        self._object_body_ids: np.ndarray | None = None
        self._info: dict[str, Any] | None = None
        self._scene_glb: bytes | None = None
        self._chunks: OrderedDict[int, tuple[bytes, int]] = OrderedDict()

    def _assert_source_identity(self) -> None:
        if _source_identity(self.path) != self._identity:
            raise ReplayError("轨迹源文件已修改；请重新扫描后再加载")

    @staticmethod
    def _read_frame_rows(
        trajectory: h5py.Group,
        name: str,
        start: int,
        stop: int,
        shape: tuple[int, ...],
    ) -> np.ndarray:
        dataset = trajectory.get(name)
        if not isinstance(dataset, h5py.Dataset):
            raise ReplayError(f"trajectory/{name} 在读取期间缺失")
        values = np.asarray(dataset[start:stop])
        if values.shape != (stop - start, *shape):
            raise ReplayError(f"trajectory/{name} 在读取期间 shape 发生变化")
        if not np.all(np.isfinite(values)):
            raise ReplayError(f"trajectory/{name} 含有非有限值")
        return values

    def _set_kinematics(
        self,
        qpos: np.ndarray,
        mocap_pos: np.ndarray | None,
        mocap_quat: np.ndarray | None,
    ) -> None:
        model, data = self._model, self._data
        if model is None or data is None:
            raise ReplayError("episode 尚未加载模型")
        if qpos.shape != (int(model.nq),):
            raise ReplayError("trajectory/qpos shape 与模型不一致")
        data.qpos[:] = qpos
        if int(model.nmocap):
            if mocap_pos is None or mocap_quat is None:
                raise ReplayError("mocap 模型缺少 mocap 状态")
            if mocap_pos.shape != (int(model.nmocap), 3) or mocap_quat.shape != (int(model.nmocap), 4):
                raise ReplayError("trajectory mocap 状态 shape 与模型不一致")
            data.mocap_pos[:] = mocap_pos
            data.mocap_quat[:] = mocap_quat
        # Position-only kinematics intentionally avoids mj_forward/mj_step and
        # consequently never recomputes contacts or advances physics.
        mujoco.mj_kinematics(model, data)

    def _verify_object_pose(self, recorded: np.ndarray | None, frame: int) -> None:
        object_ids, data = self._object_body_ids, self._data
        if object_ids is None or data is None:
            raise ReplayError("episode 尚未加载对象映射")
        if not len(object_ids):
            return
        if recorded is None or recorded.shape != (len(object_ids), 7) or not np.all(np.isfinite(recorded)):
            raise ReplayError(f"frame {frame}: trajectory/object_pose 无效")
        position_error = float(np.max(np.abs(data.xpos[object_ids] - recorded[:, :3])))
        actual = data.xquat[object_ids]
        target = recorded[:, 3:]
        quaternion_error = np.minimum(
            np.max(np.abs(actual - target), axis=1),
            np.max(np.abs(actual + target), axis=1),
        )
        error = max(position_error, float(np.max(quaternion_error)))
        if not np.isfinite(error) or error > 1e-6:
            raise ReplayError(f"frame {frame}: 运动学 object_pose 误差 {error:g} 超过 1e-6")

    def _read_initial_state(self, trajectory: h5py.Group, metadata: Mapping[str, Any]) -> tuple[np.ndarray, np.ndarray | None, np.ndarray | None, np.ndarray | None]:
        fields = metadata["fields"]
        qpos = self._read_frame_rows(trajectory, "qpos", 0, 1, tuple(fields["qpos"]["shape"]))[0]
        if "mocap_pos" in fields:
            mocap_pos = self._read_frame_rows(
                trajectory, "mocap_pos", 0, 1, tuple(fields["mocap_pos"]["shape"])
            )[0]
            mocap_quat = self._read_frame_rows(
                trajectory, "mocap_quat", 0, 1, tuple(fields["mocap_quat"]["shape"])
            )[0]
        else:
            mocap_pos = mocap_quat = None
        if "object_pose" in fields:
            object_pose = self._read_frame_rows(
                trajectory, "object_pose", 0, 1, tuple(fields["object_pose"]["shape"])
            )[0]
        else:
            object_pose = None
        return qpos, mocap_pos, mocap_quat, object_pose

    def _ensure_loaded(self) -> None:
        with self._lock:
            if self._loaded:
                self._assert_source_identity()
                return
            self._assert_source_identity()
            try:
                _load_dataset_config(self.path)
                with h5py.File(self.path, "r") as handle:
                    descriptor = _descriptor(handle, self.path)
                    model_bytes = handle["model/mjb"][:].tobytes()
                    if hashlib.sha256(model_bytes).hexdigest() != descriptor.model_sha256:
                        raise ReplayError("model/mjb SHA-256 不匹配")
                    trajectory = handle["trajectory"]
                    if not isinstance(trajectory, h5py.Group):
                        raise ReplayError("trajectory 在读取期间不是 group")
                    initial_state = self._read_initial_state(trajectory, descriptor.metadata)
                self._assert_source_identity()
                model = load_model(model_bytes, descriptor.metadata, expected_model_sha256=descriptor.model_sha256)
                data = mujoco.MjData(model)
                self._model, self._data = model, data
                self._object_body_ids = np.asarray(descriptor.metadata["object_body_ids"], dtype=np.intp)
                self._set_kinematics(*initial_state[:3])
                self._verify_object_pose(initial_state[3], 0)
                scene_glb, body_ids, center = export_scene(model, data, descriptor.metadata)
                body_index_array = np.asarray(body_ids, dtype=np.intp)
                body_names = [
                    mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, int(body_id))
                    or ("world" if int(body_id) == 0 else f"body_{int(body_id)}")
                    for body_id in body_index_array
                ]
                self._body_ids = body_index_array
                self._descriptor = descriptor
                self._scene_glb = scene_glb
                self._info = {
                    "id": self._summary["id"],
                    "title": descriptor.title,
                    "scene": descriptor.scene,
                    "task": descriptor.task,
                    "frames": descriptor.frames,
                    "duration": (descriptor.frames - 1) / SAMPLE_RATE,
                    "sample_rate": SAMPLE_RATE,
                    "body_ids": [int(body_id) for body_id in body_index_array],
                    "body_names": body_names,
                    "center": center,
                    "chunk_frames": CHUNK_FRAMES,
                    "frame_stride": len(body_index_array) * 7,
                    "model_sha256": descriptor.model_sha256,
                }
                self._assert_source_identity()
                self._loaded = True
            except ReplayError:
                self._model = self._data = self._object_body_ids = None
                raise
            except (TrajectoryError, SceneError, OSError, RuntimeError, TypeError, ValueError) as exc:
                self._model = self._data = self._object_body_ids = None
                raise ReplayError(f"无法加载 replay episode: {exc}") from exc

    @property
    def info(self) -> dict[str, Any]:
        self._ensure_loaded()
        with self._lock:
            if self._info is None:
                raise ReplayError("episode 信息未初始化")
            self._assert_source_identity()
            return {
                **self._info,
                "body_ids": list(self._info["body_ids"]),
                "body_names": list(self._info["body_names"]),
                "center": list(self._info["center"]),
            }

    @property
    def scene_glb(self) -> bytes:
        self._ensure_loaded()
        with self._lock:
            if self._scene_glb is None:
                raise ReplayError("GLB 未初始化")
            self._assert_source_identity()
            return self._scene_glb

    def _build_chunk(self, chunk_start: int) -> tuple[bytes, int]:
        descriptor, model, data, body_ids = self._descriptor, self._model, self._data, self._body_ids
        if descriptor is None or model is None or data is None or body_ids is None:
            raise ReplayError("episode 尚未加载")
        stop = min(chunk_start + CHUNK_FRAMES, descriptor.frames)
        if chunk_start < 0 or chunk_start >= stop:
            raise ReplayError("chunk 起点越界")
        fields = descriptor.metadata["fields"]
        self._assert_source_identity()
        try:
            with h5py.File(self.path, "r") as handle:
                trajectory = handle.get("trajectory")
                if not isinstance(trajectory, h5py.Group):
                    raise ReplayError("trajectory 在读取期间缺失")
                qpos = self._read_frame_rows(
                    trajectory, "qpos", chunk_start, stop, tuple(fields["qpos"]["shape"])
                )
                if int(model.nmocap):
                    mocap_pos = self._read_frame_rows(
                        trajectory, "mocap_pos", chunk_start, stop, tuple(fields["mocap_pos"]["shape"])
                    )
                    mocap_quat = self._read_frame_rows(
                        trajectory, "mocap_quat", chunk_start, stop, tuple(fields["mocap_quat"]["shape"])
                    )
                else:
                    mocap_pos = mocap_quat = None
                object_ids = self._object_body_ids
                if object_ids is None:
                    raise ReplayError("episode 尚未加载对象映射")
                if len(object_ids):
                    object_pose = self._read_frame_rows(
                        trajectory, "object_pose", chunk_start, stop, tuple(fields["object_pose"]["shape"])
                    )
                else:
                    object_pose = None
        except ReplayError:
            raise
        except (OSError, RuntimeError, TypeError, ValueError) as exc:
            raise ReplayError(f"无法读取 trajectory chunk: {exc}") from exc
        finally:
            self._assert_source_identity()

        result = np.empty((stop - chunk_start, len(body_ids), 7), dtype="<f4")
        for offset in range(stop - chunk_start):
            self._set_kinematics(
                qpos[offset],
                None if mocap_pos is None else mocap_pos[offset],
                None if mocap_quat is None else mocap_quat[offset],
            )
            self._verify_object_pose(None if object_pose is None else object_pose[offset], chunk_start + offset)
            result[offset, :, :3] = data.xpos[body_ids]
            # Browser GLTF/Three.js quaternions use xyzw; MuJoCo stores wxyz.
            result[offset, :, 3:] = data.xquat[body_ids][:, (1, 2, 3, 0)]
        if not np.all(np.isfinite(result)):
            raise ReplayError("运动学输出包含非有限 body pose")
        return result.tobytes(), stop - chunk_start

    def _chunk(self, chunk_start: int) -> tuple[bytes, int]:
        cached = self._chunks.get(chunk_start)
        if cached is not None:
            self._chunks.move_to_end(chunk_start)
            return cached
        value = self._build_chunk(chunk_start)
        self._chunks[chunk_start] = value
        self._chunks.move_to_end(chunk_start)
        while len(self._chunks) > _MAX_CHUNKS:
            self._chunks.popitem(last=False)
        return value

    def frame_block(self, start: int, count: int) -> tuple[bytes, int]:
        """Return little-endian ``[frame][body][xyz,xyzw]`` Float32 bytes."""
        if isinstance(start, bool) or not isinstance(start, int):
            raise ReplayError("start 必须是整数")
        if isinstance(count, bool) or not isinstance(count, int) or count < 1 or count > CHUNK_FRAMES:
            raise ReplayError(f"count 必须为 1–{CHUNK_FRAMES}")
        self._ensure_loaded()
        with self._lock:
            descriptor, body_ids = self._descriptor, self._body_ids
            if descriptor is None or body_ids is None:
                raise ReplayError("episode 尚未加载")
            self._assert_source_identity()
            if start < 0 or start >= descriptor.frames:
                raise ReplayError("start 超出 trajectory 帧范围")
            actual_count = min(count, descriptor.frames - start)
            chunk_start = (start // CHUNK_FRAMES) * CHUNK_FRAMES
            first, first_frames = self._chunk(chunk_start)
            frame_bytes = len(body_ids) * 7 * np.dtype("<f4").itemsize
            offset = start - chunk_start
            available = min(actual_count, first_frames - offset)
            first_slice = first[offset * frame_bytes:(offset + available) * frame_bytes]
            if available == actual_count:
                return first_slice, actual_count
            second, second_frames = self._chunk(chunk_start + CHUNK_FRAMES)
            remainder = actual_count - available
            if second_frames < remainder:
                raise ReplayError("trajectory chunk 在读取期间缩短")
            return first_slice + second[:remainder * frame_bytes], actual_count


__all__ = ["CHUNK_FRAMES", "ReplayEpisode", "ReplayError", "SAMPLE_RATE", "scan_directory"]
