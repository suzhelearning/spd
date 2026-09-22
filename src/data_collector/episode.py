"""Validate completed simulation episodes without running a controller."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from data_collector.recorder import validate_episode_path


def validate_main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", type=Path)
    args = parser.parse_args(argv)
    report = validate_episode_path(args.path)
    print(json.dumps(report, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(validate_main())
