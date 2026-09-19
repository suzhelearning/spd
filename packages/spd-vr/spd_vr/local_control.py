"""Private, simulation-local control requests serviced by the physics owner.

The client blocks and belongs on a dedicated control worker, never a UI or
solver thread. Datagram requests expire on the host's shared monotonic clock.
"""
from __future__ import annotations

import heapq
import json
import math
import os
from pathlib import Path
import socket
import stat
import tempfile
import time
from typing import Any
import uuid

from .ros_joint_command import VALID_READY_MASK

_MAX_PACKET = 4096
_MAX_REQUESTS_PER_POLL = 8
_MAX_INFLIGHT_IDS = 2048
_MAX_TIMEOUT = 60.0
_OPERATIONS = {"status", "enable", "hold"}


def _private_path(path: str | Path) -> Path:
    value = Path(path).absolute()
    parent = value.parent.stat()
    if not stat.S_ISDIR(parent.st_mode) or parent.st_uid != os.getuid() or stat.S_IMODE(parent.st_mode) != 0o700:
        raise RuntimeError("control socket requires an owned private 0700 parent directory")
    return value


def _decode(packet: bytes) -> dict[str, Any]:
    if len(packet) > _MAX_PACKET:
        raise ValueError("control packet too large")
    value = json.loads(packet)
    if not isinstance(value, dict):
        raise ValueError("control packet must be an object")
    return value


class LocalControlClient:
    def __init__(self, path: str | Path, timeout: float = 0.5) -> None:
        if not math.isfinite(timeout) or not 0 < timeout <= _MAX_TIMEOUT:
            raise ValueError("control timeout must be positive and at most 60 seconds")
        self.path = Path(path)
        self.timeout = timeout

    def request(self, op: str, session_id: str = "") -> dict[str, Any]:
        """Perform one request; refusals return ok=False, transport errors raise."""
        if op not in _OPERATIONS or not isinstance(session_id, str):
            raise RuntimeError("invalid local control operation or session")
        request_id = uuid.uuid4().hex
        deadline = time.monotonic_ns() + int(self.timeout * 1_000_000_000)
        packet = json.dumps({"request_id": request_id, "op": op, "session_id": session_id,
                             "deadline_ns": deadline}, separators=(",", ":")).encode()
        if len(packet) > _MAX_PACKET:
            raise RuntimeError("control request too large")
        try:
            path = _private_path(self.path)
            with tempfile.TemporaryDirectory(prefix="c-", dir=path.parent) as reply_dir:
                with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as sock:
                    sock.bind(str(Path(reply_dir) / "r"))
                    sock.settimeout(self.timeout)
                    sock.connect(str(path))
                    sock.send(packet)
                    remaining = (deadline - time.monotonic_ns()) * 1e-9
                    if remaining <= 0:
                        raise TimeoutError("local control request expired")
                    sock.settimeout(remaining)
                    response = _decode(sock.recv(_MAX_PACKET + 1))
                    if time.monotonic_ns() >= deadline:
                        raise TimeoutError("local control request expired")
            if response.get("request_id") != request_id or response.get("session_id") != session_id:
                raise ValueError("local control response request/session mismatch")
            for key, kind in (("ok", bool), ("enabled", bool), ("error", str), ("state", str),
                              ("ready_mask", int), ("hold_mask", int)):
                if type(response.get(key)) is not kind:
                    raise ValueError(f"invalid local control response {key}")
            for key in ("authorized_session", "candidate_session"):
                if key not in response or (response[key] is not None and not isinstance(response[key], str)):
                    raise ValueError(f"invalid local control response {key}")
            if any(not 0 <= response[key] <= VALID_READY_MASK for key in ("ready_mask", "hold_mask")):
                raise ValueError("invalid local control response masks")
            if response["ok"] and op == "enable" and not (
                response["enabled"] and response["authorized_session"] == session_id
                and response["candidate_session"] == session_id and response["ready_mask"] == VALID_READY_MASK
            ):
                raise ValueError("local control enable response did not authorize requested session")
            if response["ok"] and op == "hold" and response["enabled"]:
                raise ValueError("local control hold response remains enabled")
            return response
        except (OSError, ValueError, TypeError, RecursionError) as exc:
            raise RuntimeError(f"local control unavailable: {exc}") from exc


class LocalControlServer:
    """Bounded nonblocking endpoint; poll only on the executor's physics thread."""

    def __init__(self, path: str | Path, executor: Any) -> None:
        self.path = _private_path(path)
        self.executor = executor
        self._seen: dict[str, int] = {}
        self._expiry: list[tuple[int, str]] = []
        self._socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self._identity: tuple[int, int] | None = None
        try:
            # Never unlink an existing endpoint: another viewer may own it.
            self._socket.bind(str(self.path))
            info = self.path.stat()
            self._identity = (info.st_dev, info.st_ino)
            self.path.chmod(0o600)
            self._socket.setblocking(False)
        except BaseException:
            self.close()
            raise

    def close(self) -> None:
        self._socket.close()
        if self._identity is not None:
            try:
                info = self.path.lstat()
                if (info.st_dev, info.st_ino) == self._identity:
                    self.path.unlink()
            except FileNotFoundError:
                pass
            self._identity = None

    def _status(self, request: dict[str, Any], error: str = "") -> dict[str, Any]:
        mailbox = self.executor.mailbox
        candidate = mailbox.latest
        enabled = mailbox.enabled
        if enabled:
            reason = "Control following" if not self.executor.hold_mask else "Tracking timeout; explicit authorization required to resume held groups"
        elif candidate is None:
            reason = "No legal command candidate"
        else:
            reason = "Explicit authorization required"
            try:
                candidate.validate(now_ns=time.time_ns())
                if candidate.ready_mask != VALID_READY_MASK:
                    reason = "Alignment incomplete; all three groups must be ready"
            except (TypeError, ValueError) as exc:
                reason = str(exc)
        return {
            "request_id": request["request_id"], "session_id": request["session_id"],
            "ok": not error, "error": error, "reason": reason,
            "state": self.executor.state, "enabled": enabled,
            "authorized_session": mailbox.authorized_session,
            "candidate_session": candidate.session_id if candidate is not None else None,
            "ready_mask": candidate.ready_mask if candidate is not None else 0,
            "hold_mask": self.executor.hold_mask,
        }

    def _handle(self, request: dict[str, Any]) -> dict[str, Any]:
        # Hold the same lock used by receive/authorize so even optional stdin
        # control cannot change a session between comparison and authorization.
        with self.executor.mailbox._lock:
            now = time.monotonic_ns()
            deadline = request["deadline_ns"]
            while self._expiry and self._expiry[0][0] <= now:
                _, expired_id = heapq.heappop(self._expiry)
                self._seen.pop(expired_id, None)
            if deadline <= now:
                return self._status(request, "control request expired")
            if deadline - now > int(_MAX_TIMEOUT * 1_000_000_000):
                return self._status(request, "control request deadline is too far in the future")
            request_id = request["request_id"]
            if request_id in self._seen:
                return self._status(request, "control request already processed")
            if len(self._seen) >= _MAX_INFLIGHT_IDS:
                return self._status(request, "local control busy; retry with a new request")
            self._seen[request_id] = deadline
            heapq.heappush(self._expiry, (deadline, request_id))
            op, session = request["op"], request["session_id"]
            mailbox = self.executor.mailbox
            candidate = mailbox.latest
            if op == "status":
                return self._status(request)
            if not session:
                return self._status(request, "control operation requires a session")
            if op == "hold":
                if session != mailbox.authorized_session and (candidate is None or session != candidate.session_id):
                    return self._status(request, "hold session does not match candidate or authorization")
                self.executor.authorize(False)
                return self._status(request)
            if candidate is None:
                return self._status(request, "no legal command candidate")
            if session != candidate.session_id:
                return self._status(request, "candidate session does not match authorization")
            if candidate.ready_mask != VALID_READY_MASK:
                return self._status(request, "alignment incomplete; all three groups must be ready")
            if time.monotonic_ns() >= deadline:
                return self._status(request, "control request expired")
            if not self.executor.authorize(True):
                return self._status(request, mailbox.last_reject_reason or "executor refused authorization")
            # An authorization gate may itself consume the remaining lifetime.
            # No pending target is applied until poll returns to the owner loop.
            if time.monotonic_ns() >= deadline:
                self.executor.authorize(False)
                return self._status(request, "control request expired during authorization")
            return self._status(request)

    def poll(self) -> None:
        for _ in range(_MAX_REQUESTS_PER_POLL):
            try:
                packet, peer = self._socket.recvfrom(_MAX_PACKET + 1)
            except BlockingIOError:
                return
            try:
                request = _decode(packet)
                if (not isinstance(request.get("request_id"), str) or not 1 <= len(request["request_id"]) <= 64
                    or request.get("op") not in _OPERATIONS or not isinstance(request.get("session_id"), str)
                    or type(request.get("deadline_ns")) is not int or not peer):
                    continue
            except (ValueError, TypeError, RecursionError):
                continue
            response = json.dumps(self._handle(request), separators=(",", ":")).encode()
            if len(response) > _MAX_PACKET:
                continue
            try:
                self._socket.sendto(response, peer)
            except OSError:
                # Closed/timed-out clients never stall the simulation owner.
                pass
