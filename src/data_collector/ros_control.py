"""Serialize collection status for the native rclcpp publisher; no control services."""
from __future__ import annotations

import json
import time
from typing import Any


SERVICE_NAMES = {
    name: f"/spd/collection/{name}"
    for name in ("start", "save", "discard", "checkpoint", "pause", "resume", "revert", "skip")
}
STATUS_TOPIC = "/spd/collection/status"


class CollectionRosControl:
    """Publish lifecycle changes and heartbeats through the native executor."""

    def __init__(self, executor: Any, session: Any) -> None:
        self.executor = executor
        self.session = session
        self._last_publish_ns = 0
        session.on_transition = self.publish
        self.publish()

    def publish(self) -> None:
        self.executor.publish_status(json.dumps(self.session.snapshot(), ensure_ascii=False))
        self._last_publish_ns = time.monotonic_ns()

    def heartbeat(self) -> None:
        if time.monotonic_ns() - self._last_publish_ns >= 250_000_000:
            self.publish()
