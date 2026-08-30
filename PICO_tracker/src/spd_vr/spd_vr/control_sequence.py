"""Process-safe, session-scoped control sequence allocation and publishing."""

from __future__ import annotations

import fcntl
import json
import os
import tempfile
from contextlib import contextmanager
from pathlib import Path
from typing import Any, Callable, Iterator

DEFAULT_PATH = Path("~/.cache/spd-vr/control-sequence").expanduser()
DEFAULT_SESSION = "spd-teleop"


class ControlSequenceAllocator:
    """Allocate and publish strictly ordered sequences under one session lock."""

    def __init__(self, path: str | Path | None = None, *, session: str = DEFAULT_SESSION) -> None:
        self.path = Path(path).expanduser() if path is not None else default_path()
        self.session = str(session)
        self.lock_path = self.path.with_name(self.path.name + ".lock")

    def _read(self) -> dict[str, int]:
        if not self.path.exists():
            return {}
        raw = self.path.read_text(encoding="utf-8").strip()
        if not raw:
            raise ValueError(f"invalid empty control sequence state: {self.path}")
        try:
            parsed: Any = json.loads(raw)
        except json.JSONDecodeError as exc:
            raise ValueError(f"invalid control sequence state: {self.path}") from exc
        if isinstance(parsed, int) and not isinstance(parsed, bool):
            if parsed < 0:
                raise ValueError(f"invalid control sequence state: {self.path}")
            return {self.session: parsed}
        if not isinstance(parsed, dict):
            raise ValueError(f"invalid control sequence state: {self.path}")
        values: dict[str, int] = {}
        for session, value in parsed.items():
            if not isinstance(session, str) or isinstance(value, bool) or not isinstance(value, int) or value < 0:
                raise ValueError(f"invalid control sequence state: {self.path}")
            values[session] = value
        return values

    def _write(self, values: dict[str, int]) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        fd, temporary = tempfile.mkstemp(prefix=f".{self.path.name}.", dir=self.path.parent)
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as stream:
                json.dump(values, stream, sort_keys=True, separators=(",", ":"))
                stream.write("\n")
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, self.path)
        finally:
            try:
                os.unlink(temporary)
            except FileNotFoundError:
                pass

    @contextmanager
    def transaction(self, requested: int | None = None) -> Iterator[int]:
        """Hold the session lock from sequence allocation through publish/ack."""
        if requested is not None and (isinstance(requested, bool) or int(requested) <= 0):
            raise ValueError("sequence must be positive")
        self.lock_path.parent.mkdir(parents=True, exist_ok=True)
        with self.lock_path.open("a+", encoding="utf-8") as lock:
            fcntl.flock(lock.fileno(), fcntl.LOCK_EX)
            values = self._read()
            sequence = max(values.get(self.session, 0) + 1, int(requested or 0), 1)
            values[self.session] = sequence
            self._write(values)
            try:
                yield sequence
            finally:
                fcntl.flock(lock.fileno(), fcntl.LOCK_UN)

    def allocate(self, requested: int | None = None) -> int:
        with self.transaction(requested) as sequence:
            return sequence

    def publish(self, publisher: Any, payload_factory: Callable[[int], Any], requested: int | None = None, wait: Callable[[int], Any] | None = None) -> tuple[int, Any]:
        """Publish, and optionally wait for acknowledgement, under one session lock."""
        with self.transaction(requested) as sequence:
            result = publisher.put(payload_factory(sequence))
            if wait is not None:
                result = wait(sequence)
            return sequence, result


def default_path() -> Path:
    return Path(os.environ.get("SPD_VR_CONTROL_SEQUENCE_FILE", str(DEFAULT_PATH))).expanduser()


__all__ = ["ControlSequenceAllocator", "DEFAULT_PATH", "DEFAULT_SESSION", "default_path"]
