"""Validated sampling and output settings for the simulation collector."""
from __future__ import annotations

from dataclasses import asdict, dataclass, replace
from pathlib import Path

import yaml


PHYSICS_HZ = 480


@dataclass(frozen=True)
class CollectionConfig:
    version: int
    data_dir: Path
    state_rate_hz: int
    writer_queue_size: int
    max_frames: int

    def __post_init__(self) -> None:
        for name in ("version", "state_rate_hz", "writer_queue_size", "max_frames"):
            if type(getattr(self, name)) is not int:
                raise ValueError(f"collection {name} must be an integer")
        if self.version != 2:
            raise ValueError("collection version must be 2 (whole-scene trajectories)")
        if self.state_rate_hz != 60:
            raise ValueError("collection state_rate_hz must be 60 for the trajectory contract")
        if self.writer_queue_size < 2:
            raise ValueError("collection writer_queue_size must be at least 2")
        if self.max_frames < 0:
            raise ValueError("collection max_frames must be nonnegative (0 means unlimited)")
        if not isinstance(self.data_dir, Path):
            raise ValueError("collection data_dir must be a Path")

    def as_dict(self) -> dict:
        values = asdict(self)
        values["data_dir"] = str(self.data_dir)
        return values


def load_collection_config(
    path: str | Path, *, output: str | Path | None = None, max_frames: int | None = None,
) -> CollectionConfig:
    path = Path(path).expanduser().resolve()
    try:
        with path.open(encoding="utf-8") as stream:
            document = yaml.safe_load(stream)
    except (OSError, yaml.YAMLError) as exc:
        raise ValueError(f"cannot load collection config {path}: {exc}") from exc
    fields = set(CollectionConfig.__dataclass_fields__)
    if not isinstance(document, dict) or set(document) != fields:
        raise ValueError(f"collection config must contain exactly: {', '.join(sorted(fields))}")
    directory = document["data_dir"]
    if not isinstance(directory, str) or not directory.strip():
        raise ValueError("collection data_dir must be a nonempty path string")
    directory = Path(directory).expanduser()
    if not directory.is_absolute():
        directory = path.parent / directory
    config = CollectionConfig(**{**document, "data_dir": directory.resolve()})
    overrides = {}
    if output is not None:
        if not str(output).strip():
            raise ValueError("collection output override must be a nonempty path")
        overrides["data_dir"] = Path(output).expanduser().resolve()
    if max_frames is not None:
        overrides["max_frames"] = max_frames
    return replace(config, **overrides)
