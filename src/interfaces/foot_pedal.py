"""Opt-in, exclusively grabbed Linux evdev pedals; callbacks only queue commands."""
from __future__ import annotations

import errno
import fcntl
import math
import os
from pathlib import Path
import select
import stat
import struct
import threading
import time
from typing import Callable, Iterable, Iterator


EV_SYN = 0
EV_KEY = 1
SYN_DROPPED = 3
KEY_MAX = 0x2FF
DEFAULT_KEYS = (37, 25, 48)  # k, p, b
INPUT_EVENT = struct.Struct('@llHHi')
_KEY_BYTES = (KEY_MAX + 8) // 8


def _ioc(direction: int, number: int, size: int) -> int:
    return (direction << 30) | (size << 16) | (ord('E') << 8) | number


_EVIOCGVERSION = _ioc(2, 0x01, 4)
_EVIOCGRAB = _ioc(1, 0x90, 4)
_EVIOCSCLOCKID = _ioc(1, 0xA0, 4)
_EVIOCGBIT_TYPES = _ioc(2, 0x20, 4)
_EVIOCGBIT_KEYS = _ioc(2, 0x20 + EV_KEY, _KEY_BYTES)
_EVIOCGKEY = _ioc(2, 0x18, _KEY_BYTES)


def _validate_keys(keys: Iterable[int]) -> tuple[int, int, int]:
    values = tuple(keys)
    if (len(values) != 3 or any(type(key) is not int or not 1 <= key <= KEY_MAX for key in values)
            or len(set(values)) != 3):
        raise ValueError('pedal keys must be three distinct EV_KEY codes between 1 and 767')
    return values


def parse_pedal_keys(value: str) -> tuple[int, int, int]:
    """Parse decimal checkpoint,pause-toggle,revert-or-skip Linux key codes."""
    try:
        keys = tuple(int(part.strip(), 10) for part in value.split(','))
    except ValueError as exc:
        raise ValueError('pedal keys must be three comma-separated decimal key codes') from exc
    return _validate_keys(keys)


class PedalGesture:
    """Release-only gestures, measured in kernel CLOCK_MONOTONIC nanoseconds."""

    def __init__(self, keys: tuple[int, int, int] = DEFAULT_KEYS,
                 long_press_s: float = 1.0, held: Iterable[int] = ()) -> None:
        self.keys = _validate_keys(keys)
        if not math.isfinite(long_press_s) or long_press_s <= 0:
            raise ValueError('long_press_s must be finite and positive')
        self._threshold_ns = math.ceil(long_press_s * 1_000_000_000)
        self._held = set(held).intersection(self.keys)
        self._pressed: dict[int, int] = {}
        self._cancelled = False

    def cancel(self) -> None:
        """Disarm permanently: a dropped/disconnected stream needs a new reader."""
        self._pressed.clear()
        self._held.clear()
        self._cancelled = True

    def feed(self, event_type: int, code: int, value: int, timestamp_ns: int) -> str | None:
        if self._cancelled:
            return None
        if event_type == EV_SYN and code == SYN_DROPPED:
            self.cancel()
            raise OSError(errno.EIO, 'foot pedal input lost events (SYN_DROPPED)')
        if event_type != EV_KEY or code not in self.keys:
            return None
        if code in self._held:
            if value == 0:
                self._held.remove(code)
            return None
        if value == 1:
            self._pressed.setdefault(code, timestamp_ns)
            return None
        if value != 0:  # Ignore autorepeat (2), including repeats without a down.
            return None
        started_ns = self._pressed.pop(code, None)
        if started_ns is None:
            return None
        if timestamp_ns < started_ns:
            self.cancel()
            raise OSError(errno.EIO, 'foot pedal event clock moved backwards')
        if code == self.keys[0]:
            return 'checkpoint'
        if code == self.keys[1]:
            return 'pause_toggle'
        return 'skip' if timestamp_ns - started_ns >= self._threshold_ns else 'revert'


def decode_events(data: bytes) -> Iterator[tuple[int, int, int, int]]:
    """Decode native Linux input_event records without hardware or wall-clock time."""
    if len(data) % INPUT_EVENT.size:
        raise OSError(errno.EIO, 'foot pedal returned an incomplete input_event')
    for seconds, microseconds, event_type, code, value in INPUT_EVENT.iter_unpack(data):
        if seconds < 0 or not 0 <= microseconds < 1_000_000:
            raise OSError(errno.EIO, 'foot pedal returned an invalid event timestamp')
        yield event_type, code, value, seconds * 1_000_000_000 + microseconds * 1_000


def _bit_is_set(bits: bytearray, code: int) -> bool:
    return bool(bits[code // 8] & (1 << (code % 8)))


def _read_bits(fd: int, request: int, size: int) -> bytearray:
    bits = bytearray(size)
    fcntl.ioctl(fd, request, bits, True)
    return bits


class FootPedal:
    """One explicit device, one reader lifetime, no reconnect or automatic resume.

    start() synchronously validates/grabs the device. close() releases it and
    waits at most half a second for the reader; callbacks must be nonblocking.
    """

    def __init__(self, device: Path, callback: Callable[[str], None],
                 on_error: Callable[[str], None], *,
                 keys: tuple[int, int, int] = DEFAULT_KEYS,
                 long_press_s: float = 1.0) -> None:
        self.device = Path(device)
        self._callback = callback
        self._on_error = on_error
        self._gesture = PedalGesture(keys, long_press_s)
        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._fd: int | None = None
        self._grabbed = False
        self._started = False
        self._thread: threading.Thread | None = None
        self._ready_ns = 0

    def start(self) -> None:
        with self._lock:
            if self._started or self._stop.is_set():
                raise ValueError('foot pedal reader cannot be started more than once')
            self._started = True
            try:
                fd = os.open(self.device, os.O_RDONLY | os.O_NONBLOCK | os.O_CLOEXEC)
                self._fd = fd
                if not stat.S_ISCHR(os.fstat(fd).st_mode):
                    raise ValueError(f'foot pedal device is not a character device: {self.device}')
                fcntl.ioctl(fd, _EVIOCGVERSION, bytearray(4), True)
                types = _read_bits(fd, _EVIOCGBIT_TYPES, 4)
                if not _bit_is_set(types, EV_KEY):
                    raise ValueError(f'foot pedal device does not support EV_KEY: {self.device}')
                supported = _read_bits(fd, _EVIOCGBIT_KEYS, _KEY_BYTES)
                missing = [key for key in self._gesture.keys if not _bit_is_set(supported, key)]
                if missing:
                    raise ValueError(f'foot pedal device does not support configured key codes: {missing}')
                fcntl.ioctl(fd, _EVIOCSCLOCKID, struct.pack('@i', time.CLOCK_MONOTONIC))
                fcntl.ioctl(fd, _EVIOCGRAB, 1)
                self._grabbed = True
                held = _read_bits(fd, _EVIOCGKEY, _KEY_BYTES)
                self._gesture._held = {key for key in self._gesture.keys if _bit_is_set(held, key)}
                # Ignore queued pre-start key events as well as the release of
                # any key already down. A startup race may discard a cycle,
                # but can never authorize an incomplete one.
                self._ready_ns = time.monotonic_ns()
                self._thread = threading.Thread(target=self._run, args=(fd,),
                                                name='foot-pedal', daemon=True)
                self._thread.start()
            except BaseException as exc:
                self._stop.set()
                self._gesture.cancel()
                self._release()
                if isinstance(exc, OSError):
                    raise OSError(exc.errno, f'cannot start foot pedal {self.device}: {exc}') from exc
                raise

    def _release(self) -> None:
        with self._lock:
            fd, self._fd = self._fd, None
            if fd is None:
                return
            try:
                if self._grabbed:
                    fcntl.ioctl(fd, _EVIOCGRAB, 0)
            except OSError:
                pass  # Closing also removes an evdev grab after unplug/error.
            finally:
                self._grabbed = False
                os.close(fd)

    def _run(self, fd: int) -> None:
        error: str | None = None
        try:
            while not self._stop.is_set():
                readable, _, _ = select.select([fd], [], [], 0.1)
                if not readable:
                    continue
                with self._lock:
                    if self._stop.is_set() or self._fd != fd:
                        break
                    try:
                        data = os.read(fd, INPUT_EVENT.size * 64)
                    except BlockingIOError:
                        continue
                if not data:
                    raise OSError(errno.ENODEV, 'foot pedal disconnected')
                for event_type, code, value, timestamp_ns in decode_events(data):
                    if self._stop.is_set():
                        break
                    if event_type == EV_KEY and timestamp_ns <= self._ready_ns:
                        continue
                    operation = self._gesture.feed(event_type, code, value, timestamp_ns)
                    if operation is not None and not self._stop.is_set():
                        self._callback(operation)
        except Exception as exc:
            if not self._stop.is_set():
                error = f'foot pedal {self.device}: {exc}'
        finally:
            self._stop.set()
            self._gesture.cancel()
            self._release()
        if error is not None:
            self._on_error(error)

    def close(self) -> None:
        self._stop.set()
        self._release()
        thread = self._thread
        if thread is not None and thread.ident is not None and thread is not threading.current_thread():
            thread.join(timeout=0.5)
