"""Compatibility entry point forwarding to the authoritative URDF compiler."""

from __future__ import annotations

import argparse
from pathlib import Path

from .model_compiler.artifacts import compile_models


def workspace_root() -> Path:
    return Path(__file__).resolve().parents[4]


def build_model(output_dir: str | Path | None = None) -> tuple[Path, Path, Path]:
    """Compile the unified plant and return full XML, manifest, calibration paths."""
    output = Path(output_dir) if output_dir is not None else Path(__file__).resolve().parents[1] / "generated"
    urdf = workspace_root() / "assets" / "tianji_wuji2" / "tianji_wuji2.urdf"
    result = compile_models(urdf, output, output.parent / "collision_cache")
    return result.full_model, result.path, result.actuator_calibration


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--urdf", type=Path, default=None)
    parser.add_argument("--cache", type=Path, default=None)
    args = parser.parse_args(argv)
    output = args.output_dir or Path(__file__).resolve().parents[1] / "generated"
    urdf = args.urdf or workspace_root() / "assets" / "tianji_wuji2" / "tianji_wuji2.urdf"
    result = compile_models(urdf, output, args.cache)
    print(result.output_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
