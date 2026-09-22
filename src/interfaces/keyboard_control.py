"""Shared, tap-only keyboard controls for focused terminals and viewers."""
from __future__ import annotations

from contextlib import contextmanager
import os
import select
import sys
import termios
import tty
from typing import Iterator


KEY_COMMANDS = {
    "r": "checkpoint",
    "s": "pause_toggle",
    "d": "revert_skip",
    "g": "start",
    "f": "save",
}


@contextmanager
def terminal_input(enabled: bool, fd: int | None = None) -> Iterator[None]:
    if not enabled:
        yield
        return
    fd = sys.stdin.fileno() if fd is None else fd
    previous = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)  # Keep ISIG: Ctrl+C still raises KeyboardInterrupt.
        yield
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, previous)


def read_key(fd: int | None = None) -> str:
    fd = sys.stdin.fileno() if fd is None else fd
    if select.select([fd], [], [], 0)[0]:
        key = os.read(fd, 1)
        if not key:
            raise EOFError
        return key.decode("ascii", errors="ignore").lower()
    return ""
