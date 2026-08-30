"""Reliably publish canonical control frames to the bridge Zenoh peer."""

from __future__ import annotations

import argparse
import json
import os
import threading
import time
from pathlib import Path
from typing import Any, Callable

import zenoh

from .control_sequence import ControlSequenceAllocator, default_path
from .wire import (
    CONTROL_KEY,
    STATUS_BRIDGE_KEY,
    STATUS_IK_KEY,
    STATUS_VIEWER_KEY,
    ControlCommand,
    ControlFrame,
    encode_control,
)
from .zenoh_transport import CONTROL_CONGESTION_CONTROL, LatestSample, ZenohNode, peer_config


DEFAULT_ENDPOINT = "tcp/127.0.0.1:7447"
DEFAULT_SEQUENCE_FILE = default_path()
STATUS_KEYS = {"bridge": STATUS_BRIDGE_KEY, "ik": STATUS_IK_KEY, "viewer": STATUS_VIEWER_KEY}


class ControlPublishError(RuntimeError):
    """Raised when a command cannot be matched or acknowledged."""


def _command(value: ControlCommand | str) -> ControlCommand:
    if isinstance(value, ControlCommand):
        return value
    try:
        return ControlCommand[str(value).upper()]
    except KeyError as exc:
        raise ValueError(f"unknown control command: {value}") from exc


def _next_sequence(path: str | Path | None = None, *, session: str = "spd-teleop") -> int:
    return ControlSequenceAllocator(path, session=session).allocate()


def build_frame(
    command: ControlCommand | str,
    *,
    sequence: int | None = None,
    clock_ns: Callable[[], int] = time.monotonic_ns,
    sequence_file: str | Path | None = None,
    session: str = "spd-teleop",
) -> ControlFrame:
    allocator = ControlSequenceAllocator(sequence_file, session=session)
    sequence = allocator.allocate(sequence)
    return ControlFrame(sequence, max(1, int(clock_ns())), _command(command))

def _wait_matching(publisher: Any, timeout_s: float) -> None:
    status = getattr(publisher, "matching_status", None)
    listener_factory = getattr(publisher, "declare_matching_listener", None)
    if status is None or listener_factory is None:
        raise ControlPublishError("control publisher does not expose matching status")
    if bool(getattr(status, "matching", False)):
        return
    matched = threading.Event()
    listener = listener_factory(lambda value: matched.set() if getattr(value, "matching", False) else None)
    try:
        if not bool(getattr(status, "matching", False)) and not matched.wait(max(0.0, timeout_s)):
            raise ControlPublishError("timed out waiting for control consumer")
    finally:
        undeclare = getattr(listener, "undeclare", None)
        if undeclare is not None:
            undeclare()


def _wait_ack(mailboxes: dict[str, LatestSample[dict[str, Any]]], generations: dict[str, int], sequence: int, timeout_s: float) -> dict[str, dict[str, Any]]:
    deadline = time.monotonic() + max(0.0, timeout_s)
    accepted: dict[str, dict[str, Any]] = {}
    while time.monotonic() < deadline:
        for name, mailbox in mailboxes.items():
            sample = mailbox.take_new(generations[name])
            if sample is not None:
                generations[name], value = sample
                try:
                    acknowledged_sequence = int(value.get("sequence", -1))
                except (TypeError, ValueError):
                    acknowledged_sequence = -1
                if acknowledged_sequence == sequence:
                    accepted[name] = value
        if len(accepted) == len(mailboxes):
            return accepted
        time.sleep(min(0.01, max(0.0, deadline - time.monotonic())))
    missing = ",".join(name for name in mailboxes if name not in accepted)
    raise ControlPublishError(f"control sequence {sequence} was not acknowledged by: {missing}")


def publish_control(
    command: ControlCommand | str,
    *,
    endpoint: str = DEFAULT_ENDPOINT,
    sequence: int | None = None,
    clock_ns: Callable[[], int] = time.monotonic_ns,
    sequence_file: str | Path | None = None,
    session: str = "spd-teleop",
    node_factory: Callable[[Any], Any] | None = None,
    dry_run: bool = False,
    timeout_s: float = 2.0,
) -> ControlFrame:
    """Publish through a connecting-only reliable peer and require all acks."""
    frame = build_frame(command, sequence=sequence, clock_ns=clock_ns, sequence_file=sequence_file, session=session)
    if dry_run:
        return frame
    factory = ZenohNode if node_factory is None else node_factory
    node = factory(peer_config(listen=False, endpoint=endpoint))
    mailboxes = {name: LatestSample[dict[str, Any]]() for name in STATUS_KEYS}
    generations = {name: 0 for name in STATUS_KEYS}
    try:
        for name, key in STATUS_KEYS.items():
            node.declare_latest_subscriber(key, _decode_status, mailboxes[name])
        publisher = node.declare_publisher(
            CONTROL_KEY,
            congestion_control=CONTROL_CONGESTION_CONTROL,
            reliability=zenoh.Reliability.RELIABLE,
        )
        _wait_matching(publisher, timeout_s)
        publisher.put(encode_control(frame))
        _wait_ack(mailboxes, generations, frame.sequence, timeout_s)
    finally:
        node.close()
    return frame


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=[command.name.lower() for command in ControlCommand])
    parser.add_argument("--endpoint", default=os.environ.get("SPD_VR_ZENOH_ENDPOINT", DEFAULT_ENDPOINT))
    parser.add_argument("--sequence", type=int, default=None)
    parser.add_argument("--sequence-file", type=Path, default=None)
    parser.add_argument("--session", default="spd-teleop")
    parser.add_argument("--timeout", type=float, default=2.0)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args(argv)
    try:
        frame = publish_control(
            args.command,
            endpoint=args.endpoint,
            sequence=args.sequence,
            sequence_file=args.sequence_file,
            session=args.session,
            timeout_s=args.timeout,
            dry_run=args.dry_run,
        )
    except (ControlPublishError, OSError, ValueError) as exc:
        print(json.dumps({"command": args.command, "error": str(exc)}, sort_keys=True))
        return 1
    print(json.dumps({"command": frame.command.name.lower(), "endpoint": args.endpoint, "key": CONTROL_KEY, "monotonic_timestamp_ns": frame.monotonic_timestamp_ns, "sequence": frame.sequence}, sort_keys=True))
    return 0


__all__ = ["ControlPublishError", "DEFAULT_ENDPOINT", "build_frame", "main", "publish_control"]

if __name__ == "__main__":
    raise SystemExit(main())
