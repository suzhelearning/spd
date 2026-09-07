"""Operator session lifecycle and ordered control handling."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Any

from .wire.control import (
    ControlCommand,
    ControlFrame,
    ControlProtocolError,
    ControlSequenceGate,
)


class SessionState(str, Enum):
    IDLE = "IDLE"
    RUNNING = "RUNNING"
    PAUSED = "PAUSED"
    SHUTDOWN = "SHUTDOWN"


@dataclass(frozen=True, slots=True)
class SessionSnapshot:
    state: SessionState
    last_sequence: int | None
    alignment_generation: int
    requires_fresh_alignment: bool

    @property
    def paused(self) -> bool:
        return self.state is SessionState.PAUSED

    @property
    def running(self) -> bool:
        return self.state is SessionState.RUNNING

    @property
    def shutdown(self) -> bool:
        return self.state is SessionState.SHUTDOWN


class SessionController:
    """Serialize control frames and invoke plant lifecycle boundaries."""

    def __init__(self, plant: Any | None = None) -> None:
        self.plant = plant
        self.state = SessionState.IDLE
        self._gate = ControlSequenceGate()
        self._alignment_generation = 0
        self._requires_fresh_alignment = True
        self.snapshot = self._make_snapshot()

    def _make_snapshot(self) -> SessionSnapshot:
        return SessionSnapshot(
            state=self.state,
            last_sequence=self._gate.last_sequence,
            alignment_generation=self._alignment_generation,
            requires_fresh_alignment=self._requires_fresh_alignment,
        )

    def _call(self, name: str, *args: Any) -> None:
        callback = getattr(self.plant, name, None)
        if callback is not None:
            try:
                callback(*args)
            except TypeError:
                callback()

    def _require_alignment(self, control_timestamp_ns: int) -> None:
        self._alignment_generation += 1
        self._requires_fresh_alignment = True
        callback = getattr(self.plant, "require_fresh_alignment", None)
        if callback is not None:
            try:
                callback(int(control_timestamp_ns))
            except TypeError:
                callback()

    def apply(self, frame: ControlFrame) -> SessionSnapshot:
        """Apply one ordered frame; exact duplicate frames are no-ops."""
        accepted = self._gate.accept(frame)
        if not accepted:
            return self.snapshot
        command = frame.command
        if command is ControlCommand.START:
            if self.state is SessionState.IDLE:
                self.state = SessionState.RUNNING
                self._require_alignment(frame.monotonic_timestamp_ns)
        elif command is ControlCommand.PAUSE:
            if self.state is SessionState.RUNNING:
                self._call("set_paused", True)
                self.state = SessionState.PAUSED
        elif command is ControlCommand.RESUME:
            if self.state is SessionState.PAUSED:
                self._call("set_paused", False)
                self.state = SessionState.RUNNING
                self._require_alignment(frame.monotonic_timestamp_ns)
        elif command is ControlCommand.REALIGN:
            if self.state in {SessionState.RUNNING, SessionState.PAUSED}:
                self._require_alignment(frame.monotonic_timestamp_ns)
        elif command is ControlCommand.RESET:
            if self.state is not SessionState.SHUTDOWN:
                if self.state is SessionState.PAUSED:
                    self._call("set_paused", False)
                self._call("reset_home", frame.monotonic_timestamp_ns)
                self.state = SessionState.IDLE
                self._alignment_generation += 1
                self._requires_fresh_alignment = True
        elif command is ControlCommand.SHUTDOWN:
            if self.state is not SessionState.SHUTDOWN:
                self._call("shutdown")
                self.state = SessionState.SHUTDOWN
        else:  # pragma: no cover - ControlFrame validates enum values
            raise ControlProtocolError("invalid_command")
        self.snapshot = self._make_snapshot()
        return self.snapshot

    def mark_aligned(self, generation: int | None = None) -> None:
        """Allow a plant to acknowledge a fresh alignment window."""
        if generation is not None and int(generation) != self._alignment_generation:
            return
        self._requires_fresh_alignment = False
        self.snapshot = self._make_snapshot()

    @property
    def alignment_generation(self) -> int:
        return self._alignment_generation


__all__ = [
    "ControlCommand",
    "ControlFrame",
    "ControlSequenceGate",
    "SessionController",
    "SessionSnapshot",
    "SessionState",
]
