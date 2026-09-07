"""Reliable ADB-forwarded TCP subscription for PICO_2 hand frames."""

from __future__ import annotations

from collections.abc import Callable
import socket
import subprocess
import threading
import time

from .protocol import HEADER, MAGIC, MAX_PAYLOAD_BYTES, TYPE_HAND_FRAME, HandFrame, parse_hand_frame


class Pico2Receiver:
    """Reconnectable subscriber for the PICO_2 type-0x40 TCP stream.

    The callback runs on the receiver thread. It must either process the frame
    quickly or hand it to a bounded/latest-value mailbox. The receiver owns the
    ADB forward and recreates it before every connection attempt by default.
    """

    def __init__(
        self,
        on_frame: Callable[[HandFrame], None],
        *,
        host: str = "127.0.0.1",
        port: int = 10002,
        device_port: int | None = None,
        adb_path: str = "adb",
        adb_serial: str | None = None,
        reconnect_seconds: float = 2.0,
        auto_adb_forward: bool = True,
        on_connect: Callable[[], None] | None = None,
        on_disconnect: Callable[[], None] | None = None,
    ) -> None:
        if not callable(on_frame):
            raise TypeError("on_frame must be callable")
        if port <= 0 or port > 65535:
            raise ValueError("port must be in 1..65535")
        resolved_device_port = port if device_port is None else int(device_port)
        if resolved_device_port <= 0 or resolved_device_port > 65535:
            raise ValueError("device_port must be in 1..65535")
        if reconnect_seconds < 0.0:
            raise ValueError("reconnect_seconds must be non-negative")
        self.on_frame = on_frame
        self.host = host
        self.port = int(port)
        self.device_port = resolved_device_port
        self.adb_path = adb_path
        self.adb_serial = adb_serial
        self.reconnect_seconds = float(reconnect_seconds)
        self.auto_adb_forward = bool(auto_adb_forward)
        self.on_connect = on_connect
        self.on_disconnect = on_disconnect
        self.stop_event = threading.Event()
        self._socket_lock = threading.Lock()
        self._socket: socket.socket | None = None
        self._thread: threading.Thread | None = None
        self._connected = False
        self._state_lock = threading.Lock()
        self.frames_received = 0
        self.last_error = ""

    @property
    def connected(self) -> bool:
        with self._state_lock:
            return self._connected

    @property
    def running(self) -> bool:
        thread = self._thread
        return bool(thread is not None and thread.is_alive())

    def _set_connected(self, value: bool) -> None:
        with self._state_lock:
            self._connected = value

    def start(self) -> threading.Thread:
        if self._thread is not None and self._thread.is_alive():
            return self._thread
        self.stop_event.clear()
        self._thread = threading.Thread(target=self.run, name="pico2-hand-receiver", daemon=True)
        self._thread.start()
        return self._thread

    def stop(self) -> None:
        self.stop_event.set()
        with self._socket_lock:
            sock = self._socket
            self._socket = None
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                sock.close()
            except OSError:
                pass

    def join(self, timeout: float | None = None) -> None:
        thread = self._thread
        if thread is not None:
            thread.join(timeout)

    def ensure_adb_forward(self) -> None:
        if not self.auto_adb_forward:
            return
        command = [self.adb_path]
        if self.adb_serial:
            command.extend(("-s", self.adb_serial))
        command.extend((
            "forward",
            f"tcp:{self.port}",
            f"tcp:{self.device_port}",
        ))
        try:
            result = subprocess.run(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=5.0,
                check=False,
            )
        except FileNotFoundError as exc:
            raise RuntimeError(
                f"ADB executable not found: {self.adb_path!r}; install Android platform-tools"
            ) from exc
        except subprocess.TimeoutExpired as exc:
            raise RuntimeError("adb forward timed out; check the USB connection") from exc
        if result.returncode != 0:
            detail = (result.stderr or result.stdout).strip()
            raise RuntimeError(
                f"adb forward failed (exit {result.returncode})"
                + (f": {detail}" if detail else "")
            )

    @staticmethod
    def _recv_exact(sock: socket.socket, size: int, stop_event: threading.Event) -> bytes | None:
        chunks = bytearray()
        while len(chunks) < size:
            if stop_event.is_set():
                return None
            try:
                chunk = sock.recv(size - len(chunks))
            except socket.timeout:
                continue
            if not chunk:
                raise ConnectionError("PICO_2 closed the TCP connection")
            chunks.extend(chunk)
        return bytes(chunks)

    def receive_socket(self, sock: socket.socket) -> None:
        while not self.stop_event.is_set():
            first = self._recv_exact(sock, 1, self.stop_event)
            if first is None:
                return
            if first[0] != MAGIC:
                continue
            tail = self._recv_exact(sock, HEADER.size - 1, self.stop_event)
            if tail is None:
                return
            _magic, frame_type, timestamp_ms, payload_len = HEADER.unpack(first + tail)
            if payload_len > MAX_PAYLOAD_BYTES:
                raise ValueError(f"payload too large: {payload_len}")
            payload = self._recv_exact(sock, payload_len, self.stop_event)
            if payload is None:
                return
            if frame_type != TYPE_HAND_FRAME:
                continue
            frame = parse_hand_frame(timestamp_ms, payload)
            self.frames_received += 1
            self.on_frame(frame)

    def run(self) -> None:
        while not self.stop_event.is_set():
            sock: socket.socket | None = None
            try:
                self.ensure_adb_forward()
                sock = socket.create_connection((self.host, self.port), timeout=3.0)
                sock.settimeout(0.5)
                with self._socket_lock:
                    self._socket = sock
                self._set_connected(True)
                if self.on_connect is not None:
                    self.on_connect()
                self.receive_socket(sock)
            except (OSError, ConnectionError, RuntimeError, ValueError) as exc:
                self.last_error = str(exc)
                if not self.stop_event.is_set():
                    self.stop_event.wait(self.reconnect_seconds)
            finally:
                self._set_connected(False)
                if self.on_disconnect is not None:
                    self.on_disconnect()
                with self._socket_lock:
                    if self._socket is sock:
                        self._socket = None
                if sock is not None:
                    try:
                        sock.close()
                    except OSError:
                        pass


__all__ = ["Pico2Receiver"]
