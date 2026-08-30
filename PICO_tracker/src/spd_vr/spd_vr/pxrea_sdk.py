"""Minimal ctypes adapter for the PXREA custom tracking callback."""

from __future__ import annotations

import ctypes
import threading
from collections import deque
from dataclasses import dataclass
from os import PathLike
from typing import Any


class PXREAError(RuntimeError):
    """Raised when the vendor SDK rejects setup or is unavailable."""


class PXREADevCustomMessage(ctypes.Structure):
    _fields_ = [
        ("devID", ctypes.c_char * 32),
        ("dataSize", ctypes.c_uint64),
        ("dataPtr", ctypes.POINTER(ctypes.c_char)),
    ]


CALLBACK = ctypes.CFUNCTYPE(
    None, ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_void_p
)


@dataclass(frozen=True)
class CallbackEvent:
    device_id: str
    data: bytes
    reconnect: bool = False

    @property
    def raw(self) -> bytes:
        return self.data


class BoundedCallbackQueue:
    """Thread-safe FIFO with at most 64 callback slots and 2 KiB per slot."""

    def __init__(self, max_items: int = 64, max_bytes: int = 2048) -> None:
        if max_items <= 0 or max_bytes <= 0:
            raise ValueError("queue bounds must be positive")
        self.max_items = int(max_items)
        self.max_bytes = int(max_bytes)
        self._items: deque[CallbackEvent] = deque()
        self._cv = threading.Condition()
        self.dropped_overflow = 0

    def put(self, event: CallbackEvent) -> bool:
        if not isinstance(event, CallbackEvent):
            raise TypeError("event must be CallbackEvent")
        if len(event.data) > self.max_bytes:
            return False
        with self._cv:
            if len(self._items) >= self.max_items:
                self._items.popleft()
                self.dropped_overflow += 1
            self._items.append(event)
            self._cv.notify()
        return True

    def get(self, timeout: float | None = None) -> CallbackEvent | None:
        with self._cv:
            if not self._items and timeout is not None:
                self._cv.wait(timeout)
            if not self._items:
                return None
            return self._items.popleft()

    def drain(self) -> list[CallbackEvent]:
        with self._cv:
            events = list(self._items)
            self._items.clear()
            return events

    def qsize(self) -> int:
        with self._cv:
            return len(self._items)


class PXREAClient:
    """Own a PXREA callback and its exact init/deinit lifecycle."""

    callback_type = CALLBACK

    def __init__(
        self,
        library: Any,
        *,
        queue: BoundedCallbackQueue | None = None,
        user_data: int | None = None,
        flags: int = 0,
    ) -> None:
        self.library = library
        self.queue = queue or BoundedCallbackQueue()
        self.user_data = user_data
        self.flags = flags
        self._initialized = False
        self._closed = False
        self._status_lock = threading.Lock()
        self._status_counts = {"dropped_oversize": 0, "dropped_queue": 0}
        self._callback = CALLBACK(self._on_callback)
        self._configure_symbols()

    @classmethod
    def load_library(cls, path: str | bytes | PathLike[str]) -> "PXREAClient":
        try:
            library = ctypes.CDLL(path)
        except OSError as exc:
            raise PXREAError(f"failed to load PXREA library: {path}") from exc
        return cls(library)

    def _configure_symbols(self) -> None:
        try:
            init = self.library.PXREAInit
            deinit = self.library.PXREADeinit
        except AttributeError as exc:
            raise PXREAError(f"missing PXREA symbol: {exc.args[0]}") from exc
        init.argtypes = [ctypes.c_void_p, CALLBACK, ctypes.c_uint]
        init.restype = ctypes.c_int
        deinit.argtypes = []
        deinit.restype = ctypes.c_int
        self._init_fn = init
        self._deinit_fn = deinit

    @property
    def callback(self) -> CALLBACK | None:
        return self._callback

    def __enter__(self) -> "PXREAClient":
        if self._closed:
            raise PXREAError("PXREA client is closed")
        if self._initialized:
            return self
        result = int(self._init_fn(self.user_data, self._callback, self.flags))
        if result != 0:
            raise PXREAError(f"PXREAInit failed: {result}")
        self._initialized = True
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        if self._initialized:
            result = int(self._deinit_fn())
            self._initialized = False
            if result != 0:
                raise PXREAError(f"PXREADeinit failed: {result}")
        self._callback = None

    def status(self) -> dict[str, int]:
        with self._status_lock:
            return dict(self._status_counts)

    def _on_callback(
        self,
        _user: ctypes.c_void_p,
        _message_type: int,
        _reserved: int,
        message_ptr: ctypes.c_void_p,
    ) -> None:
        if not message_ptr:
            return
        message = ctypes.cast(
            message_ptr, ctypes.POINTER(PXREADevCustomMessage)
        ).contents
        size = int(message.dataSize)
        if size > self.queue.max_bytes:
            with self._status_lock:
                self._status_counts["dropped_oversize"] += 1
            return
        if size and not message.dataPtr:
            with self._status_lock:
                self._status_counts["dropped_oversize"] += 1
            return
        raw = ctypes.string_at(message.dataPtr, size)
        device_id = bytes(message.devID).split(b"\0", 1)[0].decode(
            "utf-8", "replace"
        )
        if not self.queue.put(CallbackEvent(device_id, raw)):
            with self._status_lock:
                self._status_counts["dropped_queue"] += 1


__all__ = [
    "BoundedCallbackQueue",
    "CALLBACK",
    "CallbackEvent",
    "PXREAClient",
    "PXREADevCustomMessage",
    "PXREAError",
]
