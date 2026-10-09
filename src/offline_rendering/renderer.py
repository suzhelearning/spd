"""Read-only trajectory replay into immutable, streamed native-EGL render files.

Import this module only after the worker configures its EGL device and thread
limits. A snapshotted camera document may override the embedded model in memory.
"""
from __future__ import annotations

from contextlib import contextmanager
from dataclasses import dataclass
import hashlib
import io
import json
import os
from pathlib import Path
import struct
from typing import Any, Iterator

import h5py
import mujoco
import numpy as np
from PIL import Image, __version__ as pillow_version

from cameras.camera import apply_camera_config
from data_collector.recorder import validate_episode_path
from data_collector.trajectory import _cameras, load_model, restore_frame
from offline_rendering.config import CAMERA_NAMES, INSTANCE_POLICY, RENDER_SCHEMA_VERSION, RenderSettings

_RESTORE_TOLERANCE = 1e-6
_RENDER_CONTRACT = {
    "version": 2,
    "backend": "mujoco_native_egl",
    "camera_names": list(CAMERA_NAMES),
    "hidden_geom_groups": [0, 3],
    "rgb_color_space": "RGB",
    "jpeg_subsampling": 0,
    "jpeg_progressive": False,
    "jpeg_optimize": False,
    "camera_transform": "camera_to_world_xyz_rotation_matrix_local_minus_z_view",
    "segmentation_channels": ["object_id", "object_type"],
    "instance_policy": INSTANCE_POLICY,
    "restoration_tolerance": _RESTORE_TOLERANCE,
}
_FRAME_TYPES = {"source_index": "<i8", "tick": "<i8", "sim_time": "<f8", "monotonic_ns": "<i8"}
_ERROR_KEYS = (
    "max_robot_qpos_error", "max_robot_qvel_error",
    "max_object_position_error", "max_object_quaternion_error",
)


class RenderError(RuntimeError):
    """A source, native render, or immutable output violates the render contract."""


def _canonical(value: Any) -> str:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False, allow_nan=False)


def _json_hash(value: Any) -> str:
    return hashlib.sha256(_canonical(value).encode("utf-8")).hexdigest()


def _file_hash(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while block := stream.read(8 * 1024 * 1024):
            digest.update(block)
    return digest.hexdigest()


def _text(value: Any) -> str:
    return value.decode("utf-8") if isinstance(value, bytes) else str(value)


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise RenderError(message)


def _calibration(camera_config: Any, settings: RenderSettings) -> dict[str, Any]:
    revision = camera_config.get("calibration_revision") if isinstance(camera_config, dict) else None
    approved = isinstance(revision, str) and bool(revision.strip()) and "provisional" not in revision.casefold()
    if not approved and not settings.allow_provisional_cameras:
        raise RenderError(
            "camera calibration is absent, unapproved, or provisional; provide an approved effective "
            "camera configuration, or explicitly allow provisional cameras for diagnostic-only rendering"
        )
    return {"revision": revision, "approved": approved, "diagnostic_only": not approved,
            "allow_provisional_cameras": settings.allow_provisional_cameras}


def _instance_mapping(model: Any, metadata: dict[str, Any]) -> tuple[np.ndarray, dict[str, Any]]:
    lookup = np.full(model.ngeom, -2, dtype="<i4")
    lookup[np.asarray(model.geom_group) == 1] = -1
    table_geoms = [
        geom for geom in range(model.ngeom)
        if (name := mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom)) is not None
        and (name == "scene_table" or name.startswith("scene_detail_table_"))
    ]
    lookup[table_geoms] = -3
    assigned_geoms: set[int] = set()
    used: set[int] = set()
    objects = []
    for index, item in enumerate(metadata["scene_manifest"].get("objects", [])):
        instance_id = item.get("instance_id")
        _require(type(instance_id) is int and 0 < instance_id <= np.iinfo(np.int32).max,
                 f"task object {item['name']!r} must have a positive int32 instance_id")
        _require(instance_id not in used, f"duplicate task instance_id: {instance_id}")
        used.add(instance_id)
        geoms = metadata["object_geom_ids"][index]
        _require(not assigned_geoms.intersection(geoms), "task object geometry subtrees overlap")
        _require(not set(table_geoms).intersection(geoms), "table geometry cannot be a task object")
        assigned_geoms.update(geoms)
        lookup[geoms] = instance_id
        objects.append({"instance_id": instance_id, "name": item["name"],
                        "body_id": metadata["object_body_ids"][index], "geom_ids": geoms,
                        "class_id": item.get("class_id"), "class_name": item.get("class_name")})
    mapping = {
        "objects": objects,
        "reserved": [
            {"instance_id": 0, "name": "sky_or_non_geom", "geom_ids": []},
            {"instance_id": -1, "name": "robot", "geom_ids": np.flatnonzero(lookup == -1).tolist()},
            {"instance_id": -2, "name": "environment", "geom_ids": np.flatnonzero(lookup == -2).tolist()},
            {"instance_id": -3, "name": "table", "geom_ids": table_geoms},
        ],
        "geom_instance_id": lookup.tolist(),
    }
    return lookup, mapping


@dataclass
class _Source:
    path: Path
    handle: h5py.File
    metadata: dict[str, Any]
    model: Any
    frames: int
    identity: dict[str, Any]
    camera_ids: dict[str, int]
    lookup: np.ndarray
    output_metadata: dict[str, Any]

    def assert_unchanged(self) -> None:
        _require(_file_hash(self.path) == self.identity["source_sha256"],
                 f"source changed during rendering/validation: {self.path}")


@contextmanager
def _open_source(path: Path, settings: RenderSettings) -> Iterator[_Source]:
    path = Path(path).resolve(strict=True)
    _require(path.is_file() and path.suffix in {".h5", ".hdf5"}, "source must be one completed HDF5 episode")
    source_hash = _file_hash(path)
    settings_snapshot = settings.as_dict()
    validation = validate_episode_path(path)
    with h5py.File(path, "r") as handle:
        metadata_json = handle["model/metadata"].asstr()[()]
        metadata = json.loads(metadata_json)
        model = load_model(handle["model/mjb"][:].tobytes(), metadata,
                           expected_model_sha256=validation["model_sha256"])
        camera_override = settings_snapshot.get("camera_config")
        effective_camera_config = (
            camera_override if camera_override is not None else metadata["camera_config"]
        )
        if camera_override is not None:
            apply_camera_config(model, camera_override)
        calibration = _calibration(effective_camera_config, settings)
        camera_ids = {name: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, name) for name in CAMERA_NAMES}
        missing = [name for name, index in camera_ids.items() if index < 0]
        _require(not missing, f"embedded compiled model is missing named cameras: {', '.join(missing)}")
        lookup, mapping = _instance_mapping(model, metadata)
        identity = {
            "source_sha256": source_hash,
            "model_sha256": validation["model_sha256"],
            "source_metadata_sha256": hashlib.sha256(metadata_json.encode("utf-8")).hexdigest(),
            "settings_sha256": _json_hash(settings_snapshot),
        }
        output_metadata = {
            "source": {**identity, "schema_version": 2, "frames": validation["frames"]},
            "compiled_cameras": _cameras(model),
            "camera_config": effective_camera_config,
            "calibration": calibration,
            "settings": settings_snapshot,
            "renderer_contract": _RENDER_CONTRACT,
            "engine": {"name": "MuJoCo EGL", "mujoco_version": mujoco.mj_versionString(),
                       "pillow_version": pillow_version},
            "instance_mapping": mapping,
        }
        yield _Source(path, handle, metadata, model, validation["frames"], identity,
                      camera_ids, lookup, output_metadata)


def _restore(source: _Source, data: Any, frame: dict[str, Any], index: int,
             maxima: dict[str, float], indices: tuple[np.ndarray, np.ndarray, np.ndarray]) -> None:
    restore_frame(source.model, data, frame, source.metadata)
    qpos, qvel, object_ids = indices
    errors = {
        "max_robot_qpos_error": float(np.max(np.abs(data.qpos[qpos] - frame["robot_qpos"]))),
        "max_robot_qvel_error": float(np.max(np.abs(data.qvel[qvel] - frame["robot_qvel"]))),
    }
    if len(object_ids):
        pose = frame["object_pose"]
        quaternion = data.xquat[object_ids]
        errors["max_object_position_error"] = float(np.max(np.abs(data.xpos[object_ids] - pose[:, :3])))
        errors["max_object_quaternion_error"] = float(np.max(np.minimum(
            np.max(np.abs(quaternion - pose[:, 3:]), axis=1),
            np.max(np.abs(quaternion + pose[:, 3:]), axis=1),
        )))
    for name, error in errors.items():
        _require(np.isfinite(error) and error <= _RESTORE_TOLERANCE,
                 f"source frame {index}: {name}={error:g} exceeds {_RESTORE_TOLERANCE:g}")
        maxima[name] = max(maxima[name], error)


def _indices(source: _Source) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    return tuple(np.asarray(source.metadata[name], dtype=np.intp) for name in
                 ("robot_qpos_indices", "robot_qvel_indices", "object_body_ids"))


def _paths(source: Path, destination: Path) -> tuple[Path, Path, Path]:
    # Resolve parent aliases without following an existing output symlink.
    destination = Path(destination)
    destination = destination.parent.resolve() / destination.name
    _require(destination.suffix == ".h5", "render destination must end in .h5")
    _require(not destination.is_symlink(), f"refusing symlink render destination: {destination}")
    _require(destination != source and not (destination.exists() and os.path.samefile(source, destination)),
             "render destination cannot be the source episode")
    return destination, destination.with_suffix(".partial.h5"), destination.with_name(destination.name + ".lock")


def _cleanup_message(path: Path) -> str:
    return (f"render artifact already exists: {path}; no restart or overwrite is allowed. "
            "Confirm no writer is active, then explicitly inspect and remove stale partial/lock artifacts before rerunning")


@contextmanager
def _locked(path: Path) -> Iterator[None]:
    try:
        descriptor = os.open(path, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o600)
    except FileExistsError as exc:
        raise RenderError(_cleanup_message(path)) from exc
    try:
        os.write(descriptor, _canonical({"pid": os.getpid()}).encode("utf-8"))
        os.fsync(descriptor)
        yield
    finally:
        try:
            current = path.lstat()
            owned = os.fstat(descriptor)
            if (current.st_dev, current.st_ino) == (owned.st_dev, owned.st_ino):
                path.unlink()
        except FileNotFoundError:
            pass
        finally:
            os.close(descriptor)


def _sync_file(handle: h5py.File) -> None:
    handle.flush()
    descriptor = handle.id.get_vfd_handle()
    _require(isinstance(descriptor, int), "HDF5 driver does not expose a file descriptor for durable publication")
    os.fsync(descriptor)


def _sync_directory(path: Path) -> None:
    descriptor = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _create_datasets(handle: h5py.File, frames: int, settings: RenderSettings) -> dict[str, h5py.Dataset]:
    datasets = {}
    frame_group = handle.create_group("frames")
    for name, dtype in _FRAME_TYPES.items():
        datasets[f"frames/{name}"] = frame_group.create_dataset(
            name, (frames,), dtype=dtype, chunks=(min(frames, 256),), fletcher32=True,
        )
    cameras = handle.create_group("cameras")
    for camera in CAMERA_NAMES:
        group = cameras.create_group(camera)
        datasets[f"cameras/{camera}/jpeg"] = group.create_dataset(
            "jpeg", (frames,), dtype=h5py.vlen_dtype(np.dtype("uint8")), chunks=(1,),
        )
        for name, dtype, shape in (("instance_id", "<i4", (settings.height, settings.width)),
                                   ("position", "<f8", (3,)), ("rotation", "<f8", (3, 3))):
            datasets[f"cameras/{camera}/{name}"] = group.create_dataset(
                name, (frames, *shape), dtype=dtype, chunks=(1, *shape), compression="lzf", fletcher32=True,
            )
    return datasets


def _update_digest(digest: Any, value: Any, dtype: np.dtype, *, jpeg: bool = False) -> None:
    values = np.asarray(value, dtype=dtype)
    if jpeg:
        digest.update(struct.pack("<Q", values.size))
    digest.update(memoryview(np.ascontiguousarray(values)).cast("B"))


def _store(datasets: dict[str, h5py.Dataset], digests: dict[str, Any],
           path: str, index: int, value: Any) -> None:
    dataset = datasets[path]
    dataset[index] = value
    jpeg = path.endswith("/jpeg")
    _update_digest(digests[path], value, np.dtype("uint8") if jpeg else dataset.dtype, jpeg=jpeg)


def _camera_valid(position: np.ndarray, rotation: np.ndarray, context: str) -> None:
    _require(bool(np.all(np.isfinite(position))) and bool(np.all(np.isfinite(rotation))),
             f"nonfinite camera transform: {context}")
    _require(bool(np.allclose(rotation.T @ rotation, np.eye(3), rtol=0, atol=1e-6))
             and bool(np.isclose(np.linalg.det(rotation), 1.0, rtol=0, atol=1e-6)),
             f"camera rotation is not a proper camera-to-world rotation: {context}")


def _gpu_valid(info: Any) -> None:
    _require(isinstance(info, dict), "missing verified native EGL device information")
    _require(type(info.get("gpu_id")) is int and info["gpu_id"] >= 0,
             "GPU provenance must contain a nonnegative EGL device ID")
    for key in ("gl_vendor", "gl_renderer", "gl_version", "mujoco_version"):
        _require(isinstance(info.get(key), str) and bool(info[key].strip()), f"missing GPU provenance: {key}")
    _require("nvidia" in info["gl_vendor"].casefold(), "native rendering requires a verified NVIDIA EGL vendor")
    _require(info["mujoco_version"] == mujoco.mj_versionString(), "GPU probe MuJoCo version differs from renderer")


def _render(source: _Source, handle: h5py.File, settings: RenderSettings,
            device_info: dict[str, Any]) -> None:
    datasets = _create_datasets(handle, source.frames, settings)
    digests = {path: hashlib.sha256() for path in datasets}
    for key, value in source.identity.items():
        handle.attrs[key] = value
    handle.attrs["render_schema_version"] = RENDER_SCHEMA_VERSION
    handle.attrs["complete"] = False
    data = mujoco.MjData(source.model)
    indices = _indices(source)
    maxima = dict.fromkeys(_ERROR_KEYS, 0.0)
    option = mujoco.MjvOption()
    option.geomgroup[:] = 1
    option.geomgroup[0] = option.geomgroup[3] = 0
    # Framebuffer size is a render resource, not a camera/extrinsics modification.
    source.model.vis.global_.offwidth = settings.width
    source.model.vis.global_.offheight = settings.height
    pixel_buffer = np.empty((settings.height, settings.width, 3), dtype=np.uint8)
    instance_buffer = np.empty((settings.height, settings.width), dtype="<i4")
    trajectory = {name: source.handle[f"trajectory/{name}"] for name in source.metadata["fields"]}
    with mujoco.Renderer(source.model, height=settings.height, width=settings.width,
                         max_geom=max(10000, source.model.ngeom + 100)) as renderer:
        for index in range(source.frames):
            frame = {name: trajectory[name][index] for name in source.metadata["fields"]}
            _restore(source, data, frame, index, maxima, indices)
            for name in _FRAME_TYPES:
                _store(datasets, digests, f"frames/{name}", index, index if name == "source_index" else frame[name])
            for camera, camera_id in source.camera_ids.items():
                prefix = f"cameras/{camera}/"
                position = data.cam_xpos[camera_id]
                rotation = data.cam_xmat[camera_id].reshape(3, 3)
                _camera_valid(position, rotation, f"{camera} frame {index}")
                renderer.disable_segmentation_rendering()
                renderer.update_scene(data, camera=camera_id, scene_option=option)
                rgb = renderer.render(out=pixel_buffer)
                _require(rgb.dtype == np.uint8 and rgb.shape == pixel_buffer.shape,
                         f"native renderer returned invalid RGB: {camera} frame {index}")
                with io.BytesIO() as encoded:
                    Image.fromarray(rgb).save(encoded, format="JPEG", quality=settings.jpeg_quality,
                                              subsampling=0, optimize=False, progressive=False)
                    jpeg = np.frombuffer(encoded.getbuffer(), dtype=np.uint8)
                    _store(datasets, digests, prefix + "jpeg", index, jpeg)
                    del jpeg
                renderer.enable_segmentation_rendering()
                segmentation = renderer.render(out=pixel_buffer)
                _require(segmentation.shape == (settings.height, settings.width, 2)
                         and segmentation.dtype == np.int32,
                         f"native renderer returned invalid segmentation: {camera} frame {index}")
                geom_pixels = segmentation[:, :, 1] == int(mujoco.mjtObj.mjOBJ_GEOM)
                geom_ids = segmentation[:, :, 0][geom_pixels]
                _require(bool(np.all((geom_ids >= 0) & (geom_ids < source.model.ngeom))),
                         f"segmentation contains invalid GEOM IDs: {camera} frame {index}")
                instance_buffer.fill(0)
                instance_buffer[geom_pixels] = source.lookup[geom_ids]
                _store(datasets, digests, prefix + "instance_id", index, instance_buffer)
                _store(datasets, digests, prefix + "position", index, position)
                _store(datasets, digests, prefix + "rotation", index, rotation)
    for path, digest in digests.items():
        datasets[path].attrs["sha256"] = digest.hexdigest()
    metadata = {**source.output_metadata, "source_path": str(source.path), "gpu": device_info,
                "restoration_errors": maxima}
    handle.create_dataset("metadata", data=_canonical(metadata), dtype=h5py.string_dtype("utf-8"))
    handle.attrs["metadata_sha256"] = _json_hash(metadata)
    _sync_file(handle)


def _dataset(handle: h5py.File, path: str, shape: tuple[int, ...], dtype: str,
             *, jpeg: bool = False) -> h5py.Dataset:
    dataset = handle[path]
    _require(isinstance(dataset, h5py.Dataset) and dataset.shape == shape, f"invalid dataset shape: {path}")
    actual_dtype = h5py.check_vlen_dtype(dataset.dtype) if jpeg else dataset.dtype
    _require(actual_dtype == np.dtype(dtype), f"invalid dataset dtype: {path}")
    _require(set(dataset.attrs) == {"sha256"}, f"missing or unexpected dataset integrity attributes: {path}")
    if path.endswith("/instance_id"):
        _require(dataset.compression == "lzf", f"instance IDs must use lossless LZF: {path}")
    return dataset


def _validate_output(path: Path, source: _Source, settings: RenderSettings,
                     *, complete: bool, verify_restoration: bool) -> dict[str, Any]:
    with h5py.File(path, "r") as handle:
        expected_attrs = {"render_schema_version", "complete", "metadata_sha256", *source.identity}
        _require(set(handle.attrs) == expected_attrs, "render root attributes do not match schema 2")
        schema = handle.attrs["render_schema_version"]
        _require(isinstance(schema, (int, np.integer)) and not isinstance(schema, (bool, np.bool_))
                 and int(schema) == RENDER_SCHEMA_VERSION,
                 "unsupported render schema version: expected 2 with separate table (-3) segmentation; rerender source")
        _require(isinstance(handle.attrs["complete"], (bool, np.bool_))
                 and bool(handle.attrs["complete"]) == complete, "render completion flag is invalid")
        for name, expected in source.identity.items():
            _require(_text(handle.attrs[name]) == expected, f"render/source/settings identity mismatch: {name}")
        _require(set(handle) == {"metadata", "frames", "cameras"}, "invalid render root groups")
        metadata_dataset = handle["metadata"]
        string_type = h5py.check_string_dtype(metadata_dataset.dtype) if isinstance(metadata_dataset, h5py.Dataset) else None
        _require(string_type is not None and string_type.encoding == "utf-8" and metadata_dataset.shape == (),
                 "render metadata must be scalar UTF-8 JSON")
        metadata_json = metadata_dataset.asstr()[()]
        metadata = json.loads(metadata_json)
        _require(_canonical(metadata) == metadata_json, "render metadata is not canonical JSON")
        _require(_json_hash(metadata) == _text(handle.attrs["metadata_sha256"]), "render metadata checksum mismatch")
        _require(set(metadata) == {*source.output_metadata, "source_path", "gpu", "restoration_errors"},
                 "invalid render metadata fields")
        for name, expected in source.output_metadata.items():
            _require(_canonical(metadata.get(name)) == _canonical(expected), f"render metadata identity mismatch: {name}")
        _gpu_valid(metadata["gpu"])
        errors = metadata["restoration_errors"]
        _require(isinstance(errors, dict) and set(errors) == set(_ERROR_KEYS)
                 and all(isinstance(value, (int, float)) and np.isfinite(value)
                         and 0 <= value <= _RESTORE_TOLERANCE for value in errors.values()),
                 "invalid recorded restoration errors")
        _require(isinstance(handle["frames"], h5py.Group) and set(handle["frames"]) == set(_FRAME_TYPES),
                 "invalid render frame fields")
        _require(isinstance(handle["cameras"], h5py.Group) and set(handle["cameras"]) == set(CAMERA_NAMES),
                 "render named camera set mismatch")
        datasets = {f"frames/{name}": _dataset(handle, f"frames/{name}", (source.frames,), dtype)
                    for name, dtype in _FRAME_TYPES.items()}
        for camera in CAMERA_NAMES:
            prefix = f"cameras/{camera}/"
            _require(isinstance(handle[f"cameras/{camera}"], h5py.Group)
                     and set(handle[f"cameras/{camera}"]) == {"jpeg", "instance_id", "position", "rotation"},
                     f"invalid camera datasets: {camera}")
            for name, dtype, shape in (("jpeg", "uint8", ()), ("instance_id", "<i4", (settings.height, settings.width)),
                                       ("position", "<f8", (3,)), ("rotation", "<f8", (3, 3))):
                datasets[prefix + name] = _dataset(handle, prefix + name, (source.frames, *shape), dtype,
                                                   jpeg=name == "jpeg")
        digests = {name: hashlib.sha256() for name in datasets}
        allowed_labels = np.unique(np.concatenate((source.lookup, np.asarray([0], dtype="<i4"))))
        trajectory = {name: source.handle[f"trajectory/{name}"] for name in source.metadata["fields"]}
        data = mujoco.MjData(source.model) if verify_restoration else None
        indices = _indices(source)
        maxima = dict.fromkeys(_ERROR_KEYS, 0.0)
        for index in range(source.frames):
            if verify_restoration:
                frame = {name: trajectory[name][index] for name in source.metadata["fields"]}
                _restore(source, data, frame, index, maxima, indices)
            for name in _FRAME_TYPES:
                path_name = f"frames/{name}"
                value = datasets[path_name][index]
                expected = index if name == "source_index" else (
                    frame[name] if verify_restoration else trajectory[name][index]
                )
                _require(value == expected, f"render/source frame association mismatch: {name} row {index}")
                _update_digest(digests[path_name], value, datasets[path_name].dtype)
            for camera, camera_id in source.camera_ids.items():
                prefix = f"cameras/{camera}/"
                values = {name: datasets[prefix + name][index] for name in ("jpeg", "instance_id", "position", "rotation")}
                jpeg = values["jpeg"]
                _require(jpeg.ndim == 1 and jpeg.size > 0, f"empty/invalid JPEG: {camera} row {index}")
                with Image.open(io.BytesIO(jpeg.tobytes())) as image:
                    _require(image.format == "JPEG" and image.mode == "RGB"
                             and image.size == (settings.width, settings.height),
                             f"JPEG format, RGB colors, or resolution mismatch: {camera} row {index}")
                    image.load()
                _require(bool(np.all(np.isin(values["instance_id"], allowed_labels))),
                         f"unknown instance labels: {camera} row {index}")
                _camera_valid(values["position"], values["rotation"], f"{camera} row {index}")
                if verify_restoration:
                    _require(bool(np.allclose(values["position"], data.cam_xpos[camera_id], rtol=0, atol=1e-12))
                             and bool(np.allclose(values["rotation"], data.cam_xmat[camera_id].reshape(3, 3), rtol=0, atol=1e-12)),
                             f"camera transform differs from restored source: {camera} row {index}")
                for name, value in values.items():
                    key = prefix + name
                    _update_digest(digests[key], value, np.dtype("uint8") if name == "jpeg" else datasets[key].dtype,
                                   jpeg=name == "jpeg")
        for name, digest in digests.items():
            _require(digest.hexdigest() == _text(datasets[name].attrs["sha256"]), f"render data checksum mismatch: {name}")
        if verify_restoration:
            _require(all(abs(maxima[name] - errors[name]) <= 1e-12 for name in _ERROR_KEYS),
                     "recorded restoration error summary differs from source replay")
        return {"valid": True, "complete": complete, "frames": source.frames,
                "source": str(source.path), "output": str(path), **source.identity,
                "restoration_errors": errors, "calibration": metadata["calibration"]}


def validate_rendered_episode(path: Path, source: Path, settings: RenderSettings) -> dict[str, Any]:
    """Stream-check every output row, hash, source association, and restored camera pose."""
    path = Path(path)
    _require(not path.name.endswith(".partial.h5"), "partial render files are not completed outputs")
    with _open_source(source, settings) as episode:
        result = _validate_output(path, episode, settings, complete=True, verify_restoration=True)
        episode.assert_unchanged()
        return result


def render_episode(source: Path, destination: Path, settings: RenderSettings,
                   device_info: dict[str, Any]) -> dict[str, Any]:
    """Render or safely skip one episode; never overwrite or repair existing artifacts."""
    _gpu_valid(device_info)
    _require(os.environ.get("MUJOCO_GL") == "egl" and os.environ.get("PYOPENGL_PLATFORM") == "egl"
             and os.environ.get("MUJOCO_EGL_DEVICE_ID") == str(device_info["gpu_id"]),
             "worker must configure and probe its selected native EGL device before importing the renderer")
    source = Path(source).resolve(strict=True)
    destination, partial, lock = _paths(source, destination)
    # In particular, reject pending camera approval without leaving output artifacts.
    with _open_source(source, settings) as episode:
        destination.parent.mkdir(parents=True, exist_ok=True)
        with _locked(lock):
            _require(not os.path.lexists(partial), _cleanup_message(partial))
            if os.path.lexists(destination):
                _require(destination.is_file() and not destination.is_symlink(),
                         f"render destination is not a regular file: {destination}")
                result = _validate_output(destination, episode, settings, complete=True, verify_restoration=True)
                episode.assert_unchanged()
                return {**result, "status": "skipped"}
            created = False
            published = False
            try:
                with h5py.File(partial, "x") as handle:
                    created = True
                    _render(episode, handle, settings, device_info)
                # Writing already restored and verified every frame; do not forward the
                # whole model again just to validate the just-written image checksums.
                result = _validate_output(partial, episode, settings, complete=False, verify_restoration=False)
                episode.assert_unchanged()
                with h5py.File(partial, "r+") as handle:
                    handle.attrs.modify("complete", True)
                    _sync_file(handle)
                # A same-directory hard link publishes atomically without rename's
                # overwrite race. The lock remains held through directory fsync.
                os.link(partial, destination)
                published = True
                partial.unlink()
                _sync_directory(destination.parent)
                return {**result, "status": "rendered", "complete": True, "output": str(destination)}
            except BaseException:
                if created and not published:
                    # A caught failure never leaves an apparently completed partial.
                    # A hard crash instead leaves the lock and/or partial for inspection.
                    try:
                        with h5py.File(partial, "r+") as handle:
                            handle.attrs["complete"] = False
                            _sync_file(handle)
                    except Exception:
                        pass
                raise


__all__ = ["RenderError", "render_episode", "validate_rendered_episode"]
