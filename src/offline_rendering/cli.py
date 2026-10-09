"""Command-line entry point for native MuJoCo EGL offline rendering."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from .config import load_batch_config
from .scheduler import BatchRenderError, run_batch


_DEFAULT_CONFIG = Path(__file__).resolve().parents[2] / "config" / "render_server.yaml"


def _gpu_ids(value: str) -> tuple[int, ...]:
    try:
        ids = tuple(int(part.strip()) for part in value.split(","))
    except ValueError as exc:
        raise argparse.ArgumentTypeError("use comma-separated EGL device indices, e.g. 0,1") from exc
    if not ids or any(device < 0 for device in ids):
        raise argparse.ArgumentTypeError("GPU indices must be nonnegative")
    if len(set(ids)) != len(ids):
        raise argparse.ArgumentTypeError("duplicate GPU indices; use --workers-per-gpu instead")
    return ids


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=_DEFAULT_CONFIG,
                        help=f"Batch YAML configuration (default: {_DEFAULT_CONFIG})")
    parser.add_argument("--camera-config", dest="camera_config_path", type=Path,
                        help="Camera YAML override; relative paths are resolved from the working directory")
    parser.add_argument("--input", dest="input_dir", type=Path,
                        help="Episode .h5 or directory searched recursively for episode_*.h5")
    parser.add_argument("--output", dest="output_dir", type=Path,
                        help="Separate destination root; relative episode paths are preserved")
    parser.add_argument("--gpus", dest="gpu_ids", type=_gpu_ids,
                        help="Comma-separated physical EGL device indices (not CUDA remapping)")
    parser.add_argument("--workers-per-gpu", type=int,
                        help="Spawned processes on each selected device (default from YAML: 1)")
    parser.add_argument("--threads-per-worker", type=int,
                        help="CPU numerical-library threads per spawned process")
    parser.add_argument("--expected-gpu-name",
                        help="Required substring of the actual GL renderer, e.g. 'RTX 5060 Ti'")
    parser.add_argument("--width", type=int, help="Rendered image width")
    parser.add_argument("--height", type=int, help="Rendered image height")
    parser.add_argument("--jpeg-quality", type=int, help="RGB JPEG quality (1-100)")
    parser.add_argument("--allow-provisional-cameras", action="store_true", default=None,
                        help="Diagnostic override only: permit absent/provisional camera calibration approval")
    parser.add_argument("--check-gpus", action="store_true",
                        help="Initialize every requested EGL worker and probe actual GPUs; do not touch datasets")
    args = parser.parse_args(argv)
    overrides = {
        key: value for key, value in vars(args).items()
        if key not in {"config", "check_gpus"} and value is not None
    }
    try:
        config = load_batch_config(args.config, overrides)
        summary = run_batch(config, check_only=args.check_gpus)
    except BatchRenderError as exc:
        print(json.dumps(exc.summary, sort_keys=True))
        print(f"offline-render: {exc}", file=sys.stderr)
        return 130 if exc.summary["status"] == "interrupted" else 1
    except KeyboardInterrupt:
        print("offline-render: interrupted", file=sys.stderr)
        return 130
    except Exception as exc:
        print(json.dumps({
            "status": "failed", "check_only": args.check_gpus,
            "rendered": 0, "skipped": 0, "failed": 0,
            "errors": [{"error": f"{type(exc).__name__}: {exc}"}],
        }, sort_keys=True))
        print(f"offline-render: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1
    print(json.dumps(summary, sort_keys=True))
    return 0


__all__ = ["main"]

if __name__ == "__main__":
    raise SystemExit(main())
