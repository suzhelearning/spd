"""Read completed schema-2 render companions and their unchanged state source.

No HDF5 handle survives an operation, construction, pickling, or fork. A reader
may therefore be constructed before DataLoader workers are created. Each read
opens its own read-only handles and returns owned arrays; no actions are derived.
"""
from __future__ import annotations

from contextlib import ExitStack
from dataclasses import dataclass
import hashlib
import io
import json
from pathlib import Path
from typing import Any, Iterable

import h5py
import numpy as np
from PIL import Image

from offline_rendering.config import CAMERA_NAMES, INSTANCE_POLICY, RENDER_SCHEMA_VERSION
from training_data.augmentation import AugmentationPlan, VisualAugmenter, integer, positive_ids, validate_images

_FRAME_TYPES = {"source_index": "<i8", "tick": "<i8", "sim_time": "<f8", "monotonic_ns": "<i8"}
_REWIND_DTYPE = np.dtype([("frame_count", "<i8"), ("monotonic_ns", "<i8")])


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def _text(value: Any) -> str:
    return value.decode("utf-8") if isinstance(value, bytes) else str(value)


def _hash_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while block := stream.read(8 * 1024 * 1024):
            digest.update(block)
    return digest.hexdigest()


def _signature(path: Path) -> tuple[int, int, int, int, int]:
    stat = path.stat()
    return stat.st_dev, stat.st_ino, stat.st_size, stat.st_mtime_ns, stat.st_ctime_ns


def _json_dataset(handle: h5py.Group, path: str, expected_hash: str) -> dict[str, Any]:
    dataset = handle[path]
    info = h5py.check_string_dtype(dataset.dtype) if isinstance(dataset, h5py.Dataset) else None
    _require(info is not None and info.encoding == "utf-8" and dataset.shape == (), f"{path} must be scalar UTF-8 JSON")
    text = dataset.asstr()[()]
    _require(hashlib.sha256(text.encode("utf-8")).hexdigest() == expected_hash, f"{path} checksum mismatch")
    value = json.loads(text)
    _require(isinstance(value, dict), f"{path} must contain a JSON object")
    canonical = json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False, allow_nan=False)
    _require(canonical == text, f"{path} must contain canonical finite JSON")
    return value


def _dataset(handle: h5py.Group, path: str, shape: tuple[int, ...], dtype: str, *, jpeg: bool = False) -> h5py.Dataset:
    dataset = handle[path]
    _require(isinstance(dataset, h5py.Dataset) and dataset.shape == shape, f"{path} shape mismatch")
    actual = h5py.check_vlen_dtype(dataset.dtype) if jpeg else dataset.dtype
    _require(actual == np.dtype(dtype), f"{path} dtype mismatch")
    return dataset


def _mapping_ids(mapping: dict) -> tuple[int, ...]:
    _require(isinstance(mapping, dict) and set(mapping) == {"objects", "reserved", "geom_instance_id"}, "invalid instance mapping")
    _require(isinstance(mapping["objects"], list) and isinstance(mapping["reserved"], list), "invalid instance mapping lists")
    ids = positive_ids(item["instance_id"] for item in mapping["objects"])
    reserved = mapping["reserved"]
    _require(len(reserved) == 4, "schema 2 requires reserved IDs 0, -1, -2, -3")
    expected = {0: "sky_or_non_geom", -1: "robot", -2: "environment", -3: "table"}
    _require(all(type(item.get("instance_id")) is int for item in reserved), "reserved instance IDs must be integers")
    _require({item["instance_id"]: item.get("name") for item in reserved} == expected, "invalid reserved instance semantics")
    lookup = mapping["geom_instance_id"]
    _require(isinstance(lookup, list), "geom_instance_id must be a list")
    allowed = set(ids) | {-1, -2, -3}
    _require(all(type(label) is int and label in allowed for label in lookup), "invalid geom instance labels")
    assigned: set[int] = set()
    for item in [*mapping["objects"], *reserved]:
        geoms = item["geom_ids"]
        _require(isinstance(geoms, list), "geom IDs must be lists")
        if item["instance_id"] == 0:
            _require(not geoms, "sky/non-geom cannot own geometry")
        for geom in geoms:
            index = integer(geom, "geom ID", 0, len(lookup) - 1)
            _require(index not in assigned and lookup[index] == item["instance_id"], "overlapping or inconsistent geom mapping")
            assigned.add(index)
    _require(len(assigned) == len(lookup), "instance mapping omits geometry")
    return ids


@dataclass
class SequenceSample:
    rgb: np.ndarray                       # [T, C, H, W, 3], uint8
    instance_id: np.ndarray               # [T, C, H, W], int32
    camera_names: tuple[str, ...]
    frames: dict[str, np.ndarray]         # Original source_index and clocks.
    state: dict[str, np.ndarray]          # Every recorded trajectory field.
    collection_events: dict[str, np.ndarray]  # Selected recovery; full rewind log.
    camera_position: np.ndarray           # [T, C, 3]
    camera_rotation: np.ndarray           # [T, C, 3, 3]
    task: str
    success: bool
    augmentation: AugmentationPlan | None

    @property
    def frame_indices(self) -> np.ndarray:
        return self.frames["source_index"]


class RenderedSequence:
    """Read one immutable render/source pair, with optional post-load augmentation.

    The original trajectory is required for state/contact/recovery labels; its
    recorded source_path is the default, or supply source_path after relocation.
    The source's full SHA256 is checked on construction. File identity/size/time
    signatures are checked before and after each read. Header validation is not
    a substitute for the renderer's full pixel-digest/replay validation.
    """

    def __init__(self, path: str | Path, *, source_path: str | Path | None = None) -> None:
        self.path = Path(path).expanduser().resolve(strict=True)
        _require(not self.path.name.endswith(".partial.h5"), "partial renders are not training data")
        self._closed = False
        render_signature = _signature(self.path)
        with h5py.File(self.path, "r") as handle:
            self._check_render_header(handle)
            self.metadata = _json_dataset(handle, "metadata", _text(handle.attrs["metadata_sha256"]))
            metadata = self.metadata
            contract = metadata["renderer_contract"]
            _require(type(contract.get("version")) is int and contract["version"] == 2
                     and contract.get("instance_policy") == INSTANCE_POLICY, "render lacks schema-2 table semantics; rerender source")
            _require(contract.get("camera_names") == list(CAMERA_NAMES), "render camera contract mismatch")
            self.instance_ids = _mapping_ids(metadata["instance_mapping"])
            self.frame_count = integer(metadata["source"]["frames"], "frame count", 1, np.iinfo(np.int64).max)
            self.height = integer(metadata["settings"]["height"], "height", 1, 8192)
            self.width = integer(metadata["settings"]["width"], "width", 1, 8192)
            _require(metadata["settings"].get("camera_names") == list(CAMERA_NAMES), "render settings camera mismatch")
            _require(metadata["source"].get("schema_version") == 2, "render source must use trajectory schema 2")
            for name in ("source_sha256", "model_sha256", "source_metadata_sha256", "settings_sha256"):
                _require(_text(handle.attrs[name]) == metadata["source"][name], f"render identity mismatch: {name}")
            settings_json = json.dumps(metadata["settings"], sort_keys=True, separators=(",", ":"), ensure_ascii=False, allow_nan=False)
            _require(hashlib.sha256(settings_json.encode()).hexdigest() == metadata["source"]["settings_sha256"], "render settings checksum mismatch")
            _require(set(handle["frames"]) == set(_FRAME_TYPES), "render clock fields mismatch")
            _require(set(handle["cameras"]) == set(CAMERA_NAMES), "render camera names mismatch")
            for name, dtype in _FRAME_TYPES.items():
                _dataset(handle, f"frames/{name}", (self.frame_count,), dtype)
            for camera in CAMERA_NAMES:
                prefix = f"cameras/{camera}/"
                _require(set(handle[f"cameras/{camera}"]) == {"jpeg", "instance_id", "position", "rotation"}, "render camera fields mismatch")
                _dataset(handle, prefix + "jpeg", (self.frame_count,), "uint8", jpeg=True)
                masks = _dataset(handle, prefix + "instance_id", (self.frame_count, self.height, self.width), "<i4")
                _require(masks.compression == "lzf", "instance masks must be losslessly stored with LZF")
                _dataset(handle, prefix + "position", (self.frame_count, 3), "<f8")
                _dataset(handle, prefix + "rotation", (self.frame_count, 3, 3), "<f8")
        source_candidate = Path(source_path) if source_path is not None else Path(self.metadata["source_path"])
        self.source_path = source_candidate.expanduser().resolve(strict=True)
        _require(self.source_path != self.path and not self.source_path.name.endswith(".partial.h5"), "source must be a separate completed trajectory")
        source_signature = _signature(self.source_path)
        _require(_hash_file(self.source_path) == self.metadata["source"]["source_sha256"], "trajectory SHA256 does not match render source")
        with h5py.File(self.source_path, "r") as source:
            self._check_source_header(source)
            self.source_metadata = _json_dataset(source, "model/metadata", self.metadata["source"]["source_metadata_sha256"])
            _require(_text(source["model"].attrs["metadata_sha256"]) == self.metadata["source"]["source_metadata_sha256"], "source metadata identity mismatch")
            _require(_text(source["model"].attrs["model_sha256"]) == self.metadata["source"]["model_sha256"], "source model identity mismatch")
            self.task = _text(source.attrs["task"])
            _require(isinstance(source.attrs.get("success"), (bool, np.bool_)), "trajectory success must be bool")
            self.success = bool(source.attrs["success"])
            fields = self.source_metadata["fields"]
            _require(isinstance(fields, dict) and set(source["trajectory"]) == set(fields), "trajectory fields differ from source metadata")
            for name, spec in fields.items():
                _dataset(source, f"trajectory/{name}", (self.frame_count, *spec["shape"]), spec["dtype"])
            self._rewinds = np.empty(0, dtype=_REWIND_DTYPE)
            if "collection_events" in source:
                events = source["collection_events"]
                _require(set(events).issubset({"rewind", "recovery_transition"}) and not events.attrs, "unsupported collection events")
                if "recovery_transition" in events:
                    _dataset(events, "recovery_transition", (self.frame_count,), "uint8")
                if "rewind" in events:
                    rewinds = events["rewind"]
                    _require(rewinds.ndim == 1 and rewinds.dtype == _REWIND_DTYPE and not rewinds.attrs, "invalid rewind labels")
                    self._rewinds = rewinds[:]
                    counts, times = self._rewinds["frame_count"], self._rewinds["monotonic_ns"]
                    _require(bool(np.all((counts >= 0) & (counts <= self.frame_count)))
                             and bool(np.all(np.diff(counts) >= 0)) and bool(np.all(times >= 0))
                             and bool(np.all(np.diff(times) > 0)), "invalid rewind boundaries/timestamps")
        _require(_signature(self.path) == render_signature and _signature(self.source_path) == source_signature, "render/source changed during reader construction")
        self._render_signature, self._source_signature = render_signature, source_signature

    @staticmethod
    def _check_render_header(handle: h5py.File) -> None:
        schema = handle.attrs.get("render_schema_version")
        _require(isinstance(schema, (int, np.integer)) and not isinstance(schema, (bool, np.bool_))
                 and schema == RENDER_SCHEMA_VERSION,
                 "unsupported render schema: expected 2 with separate table (-3) masks; rerender the original trajectory")
        complete = handle.attrs.get("complete")
        _require(isinstance(complete, (bool, np.bool_)) and bool(complete), "render is incomplete")
        _require(set(handle) == {"metadata", "frames", "cameras"}, "invalid render root groups")

    @staticmethod
    def _check_source_header(handle: h5py.File) -> None:
        schema = handle.attrs.get("schema_version")
        _require(isinstance(schema, (int, np.integer)) and not isinstance(schema, (bool, np.bool_)) and schema == 2,
                 "unsupported trajectory schema: expected 2")
        complete = handle.attrs.get("complete")
        _require(isinstance(complete, (bool, np.bool_)) and bool(complete) and "abort_reason" not in handle.attrs,
                 "trajectory is incomplete or aborted")

    def _unchanged(self) -> None:
        _require(not self._closed, "sequence reader is closed")
        _require(_signature(self.path) == self._render_signature and _signature(self.source_path) == self._source_signature,
                 "render/source changed after reader construction")

    def __enter__(self) -> RenderedSequence:
        self._unchanged()
        return self

    def __exit__(self, exc_type: Any, exc: Any, traceback: Any) -> None:
        self.close()

    def close(self) -> None:
        self._closed = True

    def __len__(self) -> int:
        return self.frame_count

    def read(self, frame_indices: Iterable[int], *, cameras: Iterable[str] = CAMERA_NAMES,
             augmentation: VisualAugmenter | None = None, seed: int | None = None) -> SequenceSample:
        """Load strictly increasing source rows without crossing a rewind boundary.

        Explicit seed is required with augmentation and rejected without it.
        Non-contiguous rows keep their original clocks; they are not resampled.
        Missing recovery annotations remain missing, never invented as normal.
        """
        self._unchanged()
        indices = np.asarray([integer(value, "frame index", 0, self.frame_count - 1) for value in frame_indices], dtype=np.int64)
        _require(indices.size > 0 and bool(np.all(np.diff(indices) > 0)), "frame indices must be nonempty and strictly increasing")
        camera_names = tuple(cameras)
        _require(bool(camera_names) and all(isinstance(name, str) and name in CAMERA_NAMES for name in camera_names)
                 and len(set(camera_names)) == len(camera_names), "cameras must be unique known camera names")
        if augmentation is not None:
            _require(isinstance(augmentation, VisualAugmenter), "augmentation must be VisualAugmenter")
            seed = integer(seed, "seed")
        else:
            _require(seed is None, "seed requires an augmentation transform")
        boundaries = self._rewinds["frame_count"]
        _require(not np.any((boundaries > indices[0]) & (boundaries <= indices[-1])), "sequence crosses a recorded rewind boundary; choose rows within one retained segment")
        shape = (indices.size, len(camera_names), self.height, self.width)
        rgb = np.empty((*shape, 3), dtype=np.uint8)
        masks = np.empty(shape, dtype=np.int32)
        positions = np.empty((indices.size, len(camera_names), 3), dtype=np.float64)
        rotations = np.empty((indices.size, len(camera_names), 3, 3), dtype=np.float64)
        with ExitStack() as stack:
            rendered = stack.enter_context(h5py.File(self.path, "r"))
            source = stack.enter_context(h5py.File(self.source_path, "r"))
            self._check_render_header(rendered)
            self._check_source_header(source)
            frames = {name: rendered[f"frames/{name}"][indices] for name in _FRAME_TYPES}
            _require(np.array_equal(frames["source_index"], indices), "render/source frame indices mismatch")
            state = {name: source[f"trajectory/{name}"][indices] for name in self.source_metadata["fields"]}
            for name, values in state.items():
                _require(bool(np.all(np.isfinite(values))), f"nonfinite trajectory state: {name}")
            for name in ("tick", "sim_time", "monotonic_ns"):
                _require(np.array_equal(frames[name], state[name]), f"render/source clock mismatch: {name}")
                _require(bool(np.all(frames[name] >= 0)) and bool(np.all(np.diff(frames[name]) > 0)), f"invalid source clock: {name}")
            _require(np.array_equal(np.diff(frames["tick"]), np.diff(indices) * 8), "source tick cadence mismatch")
            _require(bool(np.allclose(np.diff(frames["sim_time"]), np.diff(indices) / 60, rtol=0, atol=1e-7)), "source simulation cadence mismatch")
            events = {}
            if "collection_events" in source:
                group = source["collection_events"]
                if "recovery_transition" in group:
                    events["recovery_transition"] = group["recovery_transition"][indices]
                    _require(bool(np.all(events["recovery_transition"] <= 3)), "invalid recovery transition labels")
                if "rewind" in group:
                    events["rewind"] = self._rewinds.copy()
            for camera_index, name in enumerate(camera_names):
                group = rendered[f"cameras/{name}"]
                masks[:, camera_index] = group["instance_id"][indices]
                positions[:, camera_index] = group["position"][indices]
                rotations[:, camera_index] = group["rotation"][indices]
                for time_index, frame_index in enumerate(indices):
                    encoded = group["jpeg"][int(frame_index)]
                    _require(encoded.ndim == 1 and encoded.size > 0, "invalid JPEG row")
                    with Image.open(io.BytesIO(encoded.tobytes())) as image:
                        _require(image.format == "JPEG" and image.mode == "RGB" and image.size == (self.width, self.height), "invalid JPEG format, colors, or dimensions")
                        rgb[time_index, camera_index] = np.asarray(image)
        self._unchanged()
        _require(bool(np.all(np.isfinite(positions))) and bool(np.all(np.isfinite(rotations))), "nonfinite camera transforms")
        _require(bool(np.allclose(np.swapaxes(rotations, -1, -2) @ rotations, np.eye(3), rtol=0, atol=1e-6))
                 and bool(np.allclose(np.linalg.det(rotations), 1, rtol=0, atol=1e-6)), "invalid camera rotations")
        validate_images(rgb, masks, self.instance_ids)
        plan = None
        if augmentation is not None:
            transformed = augmentation(rgb, masks, seed=seed, known_instance_ids=self.instance_ids)
            rgb, masks, plan = transformed.rgb, transformed.instance_id, transformed.plan
        return SequenceSample(rgb, masks, camera_names, frames, state, events, positions, rotations, self.task, self.success, plan)
