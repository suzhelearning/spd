"""Server scheduling and image settings; camera poses belong to the source model."""
from __future__ import annotations

from dataclasses import asdict, dataclass, field, replace
import math
from pathlib import Path
from typing import Any

import yaml

CAMERA_NAMES = ("top", "left_wrist", "right_wrist")
RENDER_SCHEMA_VERSION = 2
INSTANCE_POLICY = "task_manifest_ids_else_table_minus3_else_group1_robot_else_environment"


def _integer(value: Any, name: str, minimum: int, maximum: int | None = None) -> None:
    if type(value) is not int or value < minimum or (maximum is not None and value > maximum):
        bound = f"{minimum}..{maximum}" if maximum is not None else f">= {minimum}"
        raise ValueError(f"{name} must be an integer {bound}")


@dataclass(frozen=True)
class RenderSettings:
    width: int = 224
    height: int = 168
    jpeg_quality: int = 90
    allow_provisional_cameras: bool = False

    def __post_init__(self) -> None:
        _integer(self.width, "width", 1, 8192)
        _integer(self.height, "height", 1, 8192)
        _integer(self.jpeg_quality, "jpeg_quality", 1, 100)
        if type(self.allow_provisional_cameras) is not bool:
            raise ValueError("allow_provisional_cameras must be bool")

    def as_dict(self) -> dict[str, Any]:
        return {**asdict(self), "camera_names": list(CAMERA_NAMES)}


@dataclass(frozen=True)
class BatchConfig:
    input_dir: Path
    output_dir: Path
    gpu_ids: tuple[int, ...] = tuple(range(8))
    workers_per_gpu: int = 1
    threads_per_worker: int = 1
    startup_timeout_s: float = 60.0
    expected_gpu_name: str | None = "RTX 5090"
    settings: RenderSettings = field(default_factory=RenderSettings)

    def __post_init__(self) -> None:
        if not isinstance(self.input_dir, Path) or not isinstance(self.output_dir, Path):
            raise ValueError("input_dir and output_dir must be Paths")
        if not isinstance(self.gpu_ids, tuple) or not self.gpu_ids:
            raise ValueError("gpu_ids must be a nonempty tuple of EGL device indices")
        for device in self.gpu_ids:
            _integer(device, "EGL device index", 0)
        if len(set(self.gpu_ids)) != len(self.gpu_ids):
            raise ValueError("gpu_ids must not contain duplicate EGL device indices")
        _integer(self.workers_per_gpu, "workers_per_gpu", 1, 16)
        _integer(self.threads_per_worker, "threads_per_worker", 1)
        if (type(self.startup_timeout_s) not in (int, float)
                or not math.isfinite(self.startup_timeout_s) or self.startup_timeout_s <= 0):
            raise ValueError("startup_timeout_s must be positive and finite")
        if self.expected_gpu_name is not None and (not isinstance(self.expected_gpu_name, str)
                                                   or not self.expected_gpu_name.strip()):
            raise ValueError("expected_gpu_name must be a nonempty string or null")
        if not isinstance(self.settings, RenderSettings):
            raise ValueError("settings must be RenderSettings")


def load_batch_config(path: str | Path, overrides: dict[str, Any] | None = None) -> BatchConfig:
    path = Path(path).expanduser().resolve()
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise ValueError(f"cannot load render configuration {path}: {exc}") from exc
    batch_fields = set(BatchConfig.__dataclass_fields__) - {"settings"}
    render_fields = set(RenderSettings.__dataclass_fields__)
    allowed = batch_fields | {"version", "render"}
    if not isinstance(document, dict) or set(document) - allowed:
        raise ValueError(f"render config fields must be from {sorted(allowed)}")
    if type(document.get("version")) is not int or document["version"] != 1:
        raise ValueError("render config version must be 1")
    values = {key: value for key, value in document.items() if key in batch_fields}
    for key in ("input_dir", "output_dir"):
        value = values.get(key)
        if not isinstance(value, str) or not value.strip():
            raise ValueError(f"render {key} must be a nonempty path")
        directory = Path(value).expanduser()
        values[key] = (directory if directory.is_absolute() else path.parent / directory).resolve()
    devices = values.get("gpu_ids", list(range(8)))
    if not isinstance(devices, list):
        raise ValueError("render gpu_ids must be a list of EGL device indices")
    values["gpu_ids"] = tuple(devices)
    render = document.get("render", {})
    if not isinstance(render, dict) or set(render) - render_fields:
        raise ValueError(f"render settings fields must be from {sorted(render_fields)}; no camera poses are accepted")
    settings = RenderSettings(**render)
    config = BatchConfig(**values, settings=settings)
    overrides = {key: value for key, value in (overrides or {}).items() if value is not None}
    if set(overrides) - (batch_fields | render_fields):
        raise ValueError(f"unknown render overrides: {sorted(set(overrides) - (batch_fields | render_fields))}")
    batch_overrides = {key: value for key, value in overrides.items() if key in batch_fields}
    for key in ("input_dir", "output_dir"):
        if key in batch_overrides:
            if not str(batch_overrides[key]).strip():
                raise ValueError(f"render {key} override must be nonempty")
            batch_overrides[key] = Path(batch_overrides[key]).expanduser().resolve()
    if "gpu_ids" in batch_overrides:
        batch_overrides["gpu_ids"] = tuple(batch_overrides["gpu_ids"])
    settings = replace(settings, **{key: value for key, value in overrides.items() if key in render_fields})
    return replace(config, **batch_overrides, settings=settings)
