"""Canonical model resources and the authoritative URDF compiler entry point."""

from __future__ import annotations

import argparse
from pathlib import Path

from description.model_compiler.artifacts import compile_models


def workspace_root() -> Path:
    """Return the repository root that owns the canonical Pixi workspace."""
    for candidate in Path(__file__).resolve().parents:
        if (candidate / "pixi.toml").is_file():
            return candidate
    raise RuntimeError("cannot locate repository pixi.toml from description package")


def description_root() -> Path:
    """Return the model source and generated-artifact directory."""
    return workspace_root() / "src" / "tianji_wuji2" / "tianji_wuji2"


def config_root() -> Path:
    """Return the repository's simulation configuration directory."""
    return workspace_root() / "config"


def build_model(output_dir: str | Path | None = None) -> tuple[Path, Path, Path]:
    """Compile the unified plant and return full XML, manifest, calibration paths."""
    output = Path(output_dir) if output_dir is not None else description_root() / "generated"
    urdf = description_root() / "assets" / "tianji_wuji2.urdf"
    result = compile_models(
        urdf,
        output,
        output.parent / "collision_cache",
        raw_collisions=True,
    )
    return result.full_model, result.path, result.actuator_calibration


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--urdf", type=Path, default=None)
    parser.add_argument("--cache", type=Path, default=None)
    args = parser.parse_args(argv)
    output = args.output_dir or description_root() / "generated"
    urdf = args.urdf or description_root() / "assets" / "tianji_wuji2.urdf"
    result = compile_models(
        urdf, output, args.cache,
        raw_collisions=True,
    )
    print(result.output_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
