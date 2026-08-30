"""Read the latest bridge, IK and viewer diagnostic status samples."""

from __future__ import annotations

import argparse
import json
import os
import time
from collections.abc import Mapping
from typing import Any, Callable

from .wire import STATUS_BRIDGE_KEY, STATUS_IK_KEY, STATUS_VIEWER_KEY
from .zenoh_transport import LatestSample, ZenohNode, peer_config


DEFAULT_ENDPOINT = "tcp/127.0.0.1:7447"
STATUS_KEYS = {
    "bridge": STATUS_BRIDGE_KEY,
    "ik": STATUS_IK_KEY,
    "viewer": STATUS_VIEWER_KEY,
}


def decode_status(payload: bytes | bytearray | memoryview) -> Mapping[str, Any]:
    """Decode a diagnostic JSON payload, rejecting non-object values."""
    value = json.loads(bytes(payload).decode("utf-8"))
    if not isinstance(value, Mapping):
        raise ValueError("status payload must be a JSON object")
    return dict(value)


def read_status(
    *,
    endpoint: str = DEFAULT_ENDPOINT,
    timeout_s: float = 1.0,
    node_factory: Callable[[Any], Any] | None = None,
    sleep: Callable[[float], None] = time.sleep,
) -> dict[str, Mapping[str, Any] | None]:
    """Subscribe through a connecting-only peer and return unknown values as null."""
    mailboxes = {name: LatestSample[Mapping[str, Any]]() for name in STATUS_KEYS}
    generations = {name: 0 for name in STATUS_KEYS}
    latest: dict[str, Mapping[str, Any] | None] = dict.fromkeys(STATUS_KEYS)
    factory = ZenohNode if node_factory is None else node_factory
    node = factory(peer_config(listen=False, endpoint=endpoint))
    try:
        for name, key in STATUS_KEYS.items():
            node.declare_latest_subscriber(key, decode_status, mailboxes[name])
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        while True:
            for name, mailbox in mailboxes.items():
                sample = mailbox.take_new(generations[name])
                if sample is not None:
                    generations[name], latest[name] = sample
            if all(value is not None for value in latest.values()) or time.monotonic() >= deadline:
                break
            sleep(min(0.01, max(0.0, deadline - time.monotonic())))
        return latest
    finally:
        node.close()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--endpoint", default=os.environ.get("SPD_VR_ZENOH_ENDPOINT", DEFAULT_ENDPOINT))
    parser.add_argument("--timeout", type=float, default=1.0)
    args = parser.parse_args(argv)
    values = read_status(endpoint=args.endpoint, timeout_s=args.timeout)
    output = {"endpoint": args.endpoint, "key": "spd/vr/v1/status", "status": values}
    print(json.dumps(output, ensure_ascii=False, sort_keys=True))
    return 0 if any(value is not None for value in values.values()) else 1


__all__ = ["DEFAULT_ENDPOINT", "STATUS_KEYS", "decode_status", "main", "read_status"]

if __name__ == "__main__":
    raise SystemExit(main())
