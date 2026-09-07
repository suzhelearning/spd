"""Subscribe to PICO_2 type-0x40 frames and publish canonical SPD tracking."""

from __future__ import annotations

import argparse
from collections.abc import Callable
import json
import math
import os
import signal
import sys
import threading
import time
from typing import Any

import numpy as np

from .defaults import DEFAULT_ZENOH_ENDPOINT
from .wire import (
    CONTROL_KEY,
    STATUS_BRIDGE_KEY,
    TRACKING_KEY,
    ControlCommand,
    decode_control,
    encode_tracking,
    TrackingFrame,
)
from .zenoh_transport import LatestSample, ZenohNode, peer_config
from .wire.control import ControlSequenceGate


_IDENTITY_POSE = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)


def _identity_hand() -> tuple[tuple[float, ...], ...]:
    return (_IDENTITY_POSE,) * 26


def _pose_values(pose: Any, name: str) -> tuple[float, ...]:
    position = tuple(float(value) for value in getattr(pose, "position"))
    quaternion = tuple(float(value) for value in getattr(pose, "quaternion_xyzw"))
    values = position + quaternion
    if len(values) != 7 or not all(math.isfinite(value) for value in values):
        raise ValueError(f"{name} must contain seven finite values")
    return values


def _canonical_hand(hand: Any, side: str) -> tuple[bool, float, tuple[tuple[float, ...], ...]]:
    active = bool(getattr(hand, "valid"))
    joints = tuple(getattr(hand, "joints"))
    if len(joints) != 26:
        raise ValueError(f"{side} hand must contain 26 joints")
    if not active:
        return False, 1.0, _identity_hand()

    decoded: list[tuple[float, ...]] = []
    for index, joint in enumerate(joints):
        if not bool(getattr(joint, "valid")):
            return False, 1.0, _identity_hand()
        values = _pose_values(joint, f"{side}.joint[{index}]")
        quaternion = np.asarray(values[3:7], dtype=np.float64)
        norm = float(np.linalg.norm(quaternion))
        if not math.isfinite(norm) or norm <= 0.0:
            return False, 1.0, _identity_hand()
        normalized = tuple(float(value) for value in (quaternion / norm))
        decoded.append(values[:3] + normalized)
    return True, 1.0, tuple(decoded)


class Pico2BridgeCore:
    """Convert one PICO_2 frame into the shared ``TrackingFrame`` packet."""

    def __init__(self, *, clock_ns: Callable[[], int] = time.monotonic_ns) -> None:
        self._clock_ns = clock_ns
        self._epoch = 0
        self._stream_started = False
        self._sequence = 0
        self._last_timestamp_ms: int | None = None
        self._last_bridge_monotonic_ns = 0
        self._published = 0
        self._invalid_frames = 0
        self._shutdown = False
        self._ready = False
        self._control_gate = ControlSequenceGate()

    @property
    def epoch(self) -> int:
        return max(1, self._epoch)

    @property
    def published(self) -> int:
        return self._published

    @property
    def invalid_frames(self) -> int:
        return self._invalid_frames

    def reset_stream(self) -> None:
        """Start a new epoch after TCP reconnect or an application restart."""
        self._epoch = 1 if not self._stream_started else self._epoch + 1
        self._stream_started = True
        self._last_timestamp_ms = None

    def set_ready(self, ready: bool = True) -> None:
        self._ready = bool(ready)

    def shutdown(self) -> None:
        self._shutdown = True
        self._ready = False

    def accept_frame(self, frame: Any) -> bytes | None:
        """Return one canonical packet, or ``None`` for a duplicate/invalid frame."""
        try:
            timestamp_ms = int(getattr(frame, "timestamp_ms"))
            if timestamp_ms < 0:
                raise ValueError("timestamp_ms must be non-negative")
            if self._epoch == 0:
                self.reset_stream()
            if self._last_timestamp_ms is not None:
                if timestamp_ms == self._last_timestamp_ms:
                    return None
                if timestamp_ms < self._last_timestamp_ms:
                    self.reset_stream()
            head_pose = _pose_values(getattr(frame, "head"), "head")
            head_valid = bool(getattr(frame.head, "valid"))
            left_active, left_scale, left_hand = _canonical_hand(getattr(frame, "left"), "left")
            right_active, right_scale, right_hand = _canonical_hand(getattr(frame, "right"), "right")
            bridge_monotonic_ns = max(1, int(self._clock_ns()))
            if bridge_monotonic_ns <= self._last_bridge_monotonic_ns:
                bridge_monotonic_ns = self._last_bridge_monotonic_ns + 1
            tracking = TrackingFrame(
                sequence=self._sequence + 1,
                tracking_epoch=self.epoch,
                source_timestamp_ns=max(1, timestamp_ms * 1_000_000),
                bridge_monotonic_ns=bridge_monotonic_ns,
                left_active=left_active,
                right_active=right_active,
                head_valid=head_valid,
                left_scale=left_scale,
                right_scale=right_scale,
                head_pose=head_pose if head_valid else _IDENTITY_POSE,
                left_hand=left_hand,
                right_hand=right_hand,
            )
            packet = encode_tracking(tracking)
        except (AttributeError, TypeError, ValueError, OverflowError):
            self._invalid_frames += 1
            return None
        self._sequence += 1
        self._last_timestamp_ms = timestamp_ms
        self._last_bridge_monotonic_ns = bridge_monotonic_ns
        self._published += 1
        return packet

    def accept_control(self, frame: Any) -> bool:
        try:
            accepted = self._control_gate.accept(frame)
        except ValueError:
            return False
        if not accepted:
            return False
        if frame.command is ControlCommand.SHUTDOWN:
            self.shutdown()
        return True

    @property
    def shutdown_requested(self) -> bool:
        return self._shutdown

    def status(self, *, connected: bool, received: int, dropped: int, last_error: str) -> dict[str, Any]:
        return {
            "status": "shutdown" if self._shutdown else ("ready" if self._ready else "starting"),
            "ready": self._ready and not self._shutdown,
            "source": "pico2",
            "connected": bool(connected),
            "tracking_epoch": self.epoch,
            "sequence": self._control_gate.last_sequence,
            "tracking_sequence": self._sequence,
            "published": self._published,
            "invalid_frames": self._invalid_frames,
            "received": int(received),
            "dropped": int(dropped),
            "last_error": last_error,
        }


def _install_signal_handlers(stop: threading.Event) -> dict[int, Any]:
    old_handlers: dict[int, Any] = {}
    try:
        for sig in (signal.SIGINT, signal.SIGTERM):
            old_handlers[sig] = signal.signal(sig, lambda *_: stop.set())
    except Exception:
        for sig, handler in old_handlers.items():
            signal.signal(sig, handler)
        raise
    return old_handlers


def _restore_signal_handlers(old_handlers: dict[int, Any]) -> None:
    for sig, handler in old_handlers.items():
        signal.signal(sig, handler)


def _run(args: argparse.Namespace) -> int:
    from pico_hand_tracking import Pico2Receiver

    node: ZenohNode | None = None
    publisher: Any | None = None
    status_publisher: Any | None = None
    receiver: Pico2Receiver | None = None
    old_handlers: dict[int, Any] = {}
    stop = threading.Event()
    core = Pico2BridgeCore()
    frame_mailbox: LatestSample[Any] = LatestSample()
    control_mailbox: LatestSample[Any] = LatestSample()
    try:
        node = ZenohNode(peer_config(listen=args.listen, endpoint=args.endpoint))
        publisher = node.declare_publisher(args.key)
        status_publisher = node.declare_publisher(STATUS_BRIDGE_KEY)
        node.declare_latest_subscriber(CONTROL_KEY, decode_control, control_mailbox)
        def on_connect() -> None:
            frame_mailbox.invalidate()
            core.reset_stream()

        receiver = Pico2Receiver(
            frame_mailbox.put,
            host=args.host,
            port=args.port,
            device_port=args.device_port,
            adb_path=args.adb_path,
            adb_serial=args.adb_serial,
            reconnect_seconds=max(args.reconnect, 0.1),
            auto_adb_forward=not args.no_adb_forward,
            on_connect=on_connect,
        )
        old_handlers = _install_signal_handlers(stop)
        core.set_ready()
        receiver.start()
        generation = 0
        control_generation = 0
        status_publisher.put(
            _status_bytes(core, receiver, frame_mailbox)
        )
        while not stop.is_set() and not core.shutdown_requested:
            control = control_mailbox.take_new(control_generation)
            if control is not None:
                control_generation, control_frame = control
                core.accept_control(control_frame)
                status_publisher.put(_status_bytes(core, receiver, frame_mailbox))
            sample = frame_mailbox.take_new(generation)
            if sample is not None:
                generation, frame = sample
                packet = core.accept_frame(frame)
                if packet is not None:
                    publisher.put(packet)
                status_publisher.put(
                    _status_bytes(core, receiver, frame_mailbox)
                )
            stop.wait(0.01)
    except (ImportError, OSError, RuntimeError, ValueError) as exc:
        print(f"PICO_2 bridge failed: {exc}", file=sys.stderr)
        return 2
    finally:
        core.shutdown()
        if receiver is not None:
            receiver.stop()
            receiver.join(timeout=1.0)
        if node is not None:
            try:
                if status_publisher is not None:
                    status_publisher.put(_status_bytes(core, receiver, frame_mailbox))
            except Exception:
                pass
            node.close()
        _restore_signal_handlers(old_handlers)
    return 0


def _status_bytes(
    core: Pico2BridgeCore,
    receiver: Any,
    mailbox: LatestSample[Any],
) -> bytes:
    status = core.status(
        connected=bool(receiver is not None and receiver.connected),
        received=0 if receiver is None else receiver.frames_received,
        dropped=mailbox.dropped_count,
        last_error="" if receiver is None else receiver.last_error,
    )
    return json.dumps(status, sort_keys=True, separators=(",", ":")).encode()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=10002)
    parser.add_argument("--device-port", type=int)
    parser.add_argument("--adb-path", default="adb")
    parser.add_argument("--adb-serial", default=os.environ.get("PICO_ADB_SERIAL"))
    parser.add_argument("--no-adb-forward", action="store_true")
    parser.add_argument("--reconnect", type=float, default=2.0)
    parser.add_argument("--key", default=TRACKING_KEY)
    parser.add_argument("--endpoint", default=DEFAULT_ZENOH_ENDPOINT)
    parser.add_argument("--listen", action="store_true")
    return _run(parser.parse_args(argv))


__all__ = ["Pico2BridgeCore", "main"]

if __name__ == "__main__":
    raise SystemExit(main())
