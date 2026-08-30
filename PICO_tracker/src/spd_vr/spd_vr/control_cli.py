"""Publish one canonical control frame to the bridge Zenoh peer."""

from __future__ import annotations

import argparse
import json
import os
import time
from pathlib import Path
from typing import Any, Callable

from .wire import CONTROL_KEY, ControlCommand, ControlFrame, encode_control
from .zenoh_transport import CONTROL_CONGESTION_CONTROL, ZenohNode, peer_config


DEFAULT_ENDPOINT = "tcp/127.0.0.1:7447"
DEFAULT_SEQUENCE_FILE = Path("~/.cache/spd-vr/control-sequence").expanduser()


def _command(value: ControlCommand | str) -> ControlCommand:
    if isinstance(value, ControlCommand):
        return value
    try:
        return ControlCommand[str(value).upper()]
    except KeyError as exc:
        raise ValueError(f"unknown control command: {value}") from exc
def _default_sequence_file() -> Path:
    return Path(os.environ.get("SPD_VR_CONTROL_SEQUENCE_FILE", str(DEFAULT_SEQUENCE_FILE))).expanduser()


def _next_sequence(path: Path | None = None) -> int:
    path = _default_sequence_file() if path is None else path
    try:
        previous = int(path.read_text(encoding="utf-8").strip())
    except (OSError, ValueError):
        previous = 0
    sequence = max(1, previous + 1)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(f"{sequence}\n", encoding="utf-8")
    return sequence


def build_frame(
    command: ControlCommand | str,
    *,
    sequence: int | None = None,
    clock_ns: Callable[[], int] = time.monotonic_ns,
    sequence_file: str | Path | None = None,
) -> ControlFrame:
    """Build a validated frame with a strictly positive timestamp and sequence."""
    if sequence is None:
        sequence = _next_sequence(_default_sequence_file() if sequence_file is None else Path(sequence_file))
    if int(sequence) <= 0:
        raise ValueError("sequence must be positive")
    if sequence_file is not None:
        path = Path(sequence_file)
        try:
            current = int(path.read_text(encoding="utf-8").strip())
        except (OSError, ValueError):
            current = 0
        if sequence > current:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(f"{sequence}\n", encoding="utf-8")
    return ControlFrame(int(sequence), max(1, int(clock_ns())), _command(command))


def publish_control(
    command: ControlCommand | str,
    *,
    endpoint: str = DEFAULT_ENDPOINT,
    sequence: int | None = None,
    clock_ns: Callable[[], int] = time.monotonic_ns,
    sequence_file: str | Path | None = None,
    node_factory: Callable[[Any], Any] | None = None,
    dry_run: bool = False,
) -> ControlFrame:
    """Publish through a connecting-only peer; never opens a listening endpoint."""
    frame = build_frame(command, sequence=sequence, clock_ns=clock_ns, sequence_file=sequence_file)
    if dry_run:
        return frame
    factory = ZenohNode if node_factory is None else node_factory
    node = factory(peer_config(listen=False, endpoint=endpoint))
    try:
        publisher = node.declare_publisher(CONTROL_KEY, congestion_control=CONTROL_CONGESTION_CONTROL)
        publisher.put(encode_control(frame))
    finally:
        node.close()
    return frame


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=[command.name.lower() for command in ControlCommand])
    parser.add_argument("--endpoint", default=os.environ.get("SPD_VR_ZENOH_ENDPOINT", DEFAULT_ENDPOINT))
    parser.add_argument("--sequence", type=int, default=None)
    parser.add_argument("--sequence-file", type=Path, default=None)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args(argv)
    frame = publish_control(
        args.command,
        endpoint=args.endpoint,
        sequence=args.sequence,
        sequence_file=args.sequence_file,
        dry_run=args.dry_run,
    )
    print(
        json.dumps(
            {
                "command": frame.command.name.lower(),
                "endpoint": args.endpoint,
                "key": CONTROL_KEY,
                "monotonic_timestamp_ns": frame.monotonic_timestamp_ns,
                "sequence": frame.sequence,
            },
            sort_keys=True,
        )
    )
    return 0


__all__ = ["DEFAULT_ENDPOINT", "build_frame", "main", "publish_control"]

if __name__ == "__main__":
    raise SystemExit(main())
