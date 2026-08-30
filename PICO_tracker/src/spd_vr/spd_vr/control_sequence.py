"""Process-safe, session-scoped control sequence allocation."""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any

import fcntl


DEFAULT_PATH = Path("~/.cache/spd-vr/control-sequence").expanduser()
DEFAULT_SESSION = "spd-teleop"


class ControlSequenceAllocator:
    """Allocate strictly increasing sequences shared by every control publisher."""

    def __init__(self, path: str | Path | None = None, *, session: str = DEFAULT_SESSION) -> None:
        self.path = Path(path).expanduser() if path is not None else default_path()
        self.session = str(session)

    def allocate(self, requested: int | None = None) -> int:
        if requested is not None and int(requested) <= 0:
            raise ValueError("sequence must be positive")
        self.path.parent.mkdir(parents=True, exist_ok=True)
        with self.path.open("a+", encoding="utf-8") as stream:
            fcntl.flock(stream.fileno(), fcntl.LOCK_EX)
            try:
                stream.seek(0)
                raw = stream.read().strip()
                values: dict[str, int]
                try:
                    parsed: Any = json.loads(raw) if raw else {}
                    values = parsed if isinstance(parsed, dict) else {self.session: int(parsed)}
                except (TypeError, ValueError, json.JSONDecodeError):
                    values = {}
                current = int(values.get(self.session, 0))
                sequence = max(current + 1, int(requested or 0), 1)
                values[self.session] = sequence
                stream.seek(0)
                stream.truncate()
                json.dump(values, stream, sort_keys=True, separators=(",", ":"))
                stream.write("\n")
                stream.flush()
                os.fsync(stream.fileno())
                return sequence
            finally:
                fcntl.flock(stream.fileno(), fcntl.LOCK_UN)


def default_path() -> Path:
    return Path(os.environ.get("SPD_VR_CONTROL_SEQUENCE_FILE", str(DEFAULT_PATH))).expanduser()


__all__ = ["ControlSequenceAllocator", "DEFAULT_PATH", "DEFAULT_SESSION", "default_path"]
