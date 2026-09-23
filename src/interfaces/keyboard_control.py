"""Shared, tap-only keyboard controls for focused terminals and viewers."""
from __future__ import annotations

from contextlib import contextmanager
import os
import select
import sys
import termios
import threading
import tty
from typing import Any, Callable, Iterator


KEY_COMMANDS = {
    "r": "checkpoint",
    "s": "save",
    "d": "revert",
    " ": "pause_toggle",
    "x": "discard",
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


class ControlTerminal:
    """Immediate stdin controls with a bounded, explicitly joined reader."""

    def __init__(self, joint_control: Callable[[str], None],
                 recording_control: Callable[[str], None], *, fd: int | None = None) -> None:
        self._joint_control = joint_control
        self._recording_control = recording_control
        self._fd = fd
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        fd = sys.stdin.fileno() if self._fd is None else self._fd
        context = terminal_input(os.isatty(fd), fd)
        context.__enter__()
        self._stop.clear()
        try:
            self._thread = threading.Thread(
                target=self._run, args=(fd, context), name="spd-ros-control",
            )
            self._thread.start()
        except BaseException:
            self._thread = None
            self._stop.set()
            context.__exit__(*sys.exc_info())
            raise

    def _run(self, fd: int, context: Any) -> None:
        try:
            while not self._stop.is_set():
                try:
                    if not select.select([fd], [], [], 0.05)[0]:
                        continue
                    key = read_key(fd)
                except (EOFError, OSError):
                    return
                if self._stop.is_set():
                    return
                if key == "q":
                    self._joint_control(key)
                    return
                elif key in KEY_COMMANDS:
                    self._recording_control(KEY_COMMANDS[key])
        finally:
            self._stop.set()
            context.__exit__(*sys.exc_info())

    def close(self) -> None:
        self._stop.set()
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join()
            self._thread = None
