"""PXREA callback bridge: queue first, decode and publish off the callback thread."""

from __future__ import annotations

import argparse
import json
import signal
import sys
import threading
import time
from collections.abc import Callable, Iterable, Mapping
from dataclasses import dataclass
from numbers import Integral
from pathlib import Path
from typing import Any

from .pico_frames import (
    FRAME_TYPE_WORLD_RESET,
    HandPairer,
    PicoFrameError,
    PicoStreamDecoder,
)
from .pxrea_sdk import (
    BoundedCallbackQueue,
    CallbackEvent,
    PXREAClient,
    PXREA_DEVICE_CUSTOM,
    PXREA_DEVICE_CONNECT,
    PXREA_DEVICE_FIND,
    PXREA_DEVICE_MISSING,
    PXREA_SERVER_CONNECT,
    PXREA_SERVER_DISCONNECT,
)
from .wire import (
    CONTROL_KEY,
    STATUS_BRIDGE_KEY,
    TRACKING_KEY,
    ControlCommand,
    ControlSequenceGate,
    TrackingFrame,
    decode_control,
    encode_tracking,
)

_LIFECYCLE_TYPES = {
    PXREA_SERVER_CONNECT,
    PXREA_SERVER_DISCONNECT,
    PXREA_DEVICE_FIND,
    PXREA_DEVICE_MISSING,
    PXREA_DEVICE_CONNECT,
}


@dataclass(frozen=True)
class BridgeStatus:
    device_id: str | None
    tracking_epoch: int
    published: int
    invalid_payloads: int
    device_selection_ambiguous: bool


class BridgeCore:
    """Decode one selected device and turn complete hand pairs into tracking bytes."""

    def __init__(
        self,
        selected_device_id: str | None = None,
        *,
        clock_ns: Callable[[], int] = time.monotonic_ns,
    ) -> None:
        self.selected_device_id = selected_device_id
        self._auto_select = selected_device_id is None
        self._seen_devices: set[str] = set()
        self._decoder = PicoStreamDecoder()
        self._pairer = HandPairer()
        self._clock_ns = clock_ns
        self._gate = ControlSequenceGate()
        self._shutdown = False
        self._ready = False
        self._epoch = 1
        self._sequence = 0
        self._published = 0
        self._invalid_payloads = 0
        self._ambiguous = False

    @property
    def epoch(self) -> int:
        return self._epoch

    def reset_device(self, device_id: str | None = None) -> None:
        if device_id is not None and self.selected_device_id not in (None, device_id):
            return
        self._epoch += 1
        self._decoder.reset()
        self._pairer.reset()

    def set_ready(self, ready: bool = True) -> None:
        self._ready = bool(ready)

    def status(self, dropped: int = 0) -> dict[str, Any]:
        return {
            "status": "shutdown" if self._shutdown else ("ready" if self._ready else "starting"),
            "ready": self._ready and not self._shutdown,
            "device_id": self.selected_device_id,
            "tracking_epoch": self._epoch,
            "published": self._published,
            "invalid_payloads": self._invalid_payloads,
            "dropped": int(dropped),
            "device_selection_ambiguous": self._ambiguous,
            "sequence": self._gate.last_sequence,
        }

    def status_json(self, dropped: int = 0) -> str:
        return json.dumps(self.status(dropped), sort_keys=True, separators=(",", ":"))

    def accept_control(self, frame: Any) -> bool:
        accepted = self._gate.accept(frame)
        if accepted and frame.command is ControlCommand.SHUTDOWN:
            self._shutdown = True
            self._ready = False
        return accepted
    def shutdown(self) -> None:
        self._shutdown = True
        self._ready = False


    def accept_event(self, event: CallbackEvent | tuple[str, bytes] | Any) -> list[bytes]:
        device_id, raw, event_type = self._event_parts(event)
        if event_type not in _LIFECYCLE_TYPES and not device_id:
            self._invalid_payloads += 1
            return []
        if event_type in {
            PXREA_SERVER_CONNECT,
            PXREA_SERVER_DISCONNECT,
            PXREA_DEVICE_FIND,
            PXREA_DEVICE_MISSING,
            PXREA_DEVICE_CONNECT,
        }:
            self.reset_device()
            return []
        self._seen_devices.add(device_id)
        if self._auto_select:
            if len(self._seen_devices) == 1:
                self.selected_device_id = device_id
            else:
                self._ambiguous = True
        if self._ambiguous or device_id != self.selected_device_id:
            return []
        if not raw or (not self._decoder._buffer and raw[0] != 0xAB):
            self._invalid_payloads += 1
            self._decoder.reset()
            return []
        try:
            frames = self._decoder.feed(raw)
        except PicoFrameError:
            self._invalid_payloads += 1
            return []
        output: list[bytes] = []
        for frame in frames:
            try:
                pair = self._pairer.accept(frame, self._epoch)
            except PicoFrameError:
                self._invalid_payloads += 1
                continue
            if frame.frame_type == FRAME_TYPE_WORLD_RESET:
                self.reset_device(device_id)
            if pair is None:
                continue
            self._sequence += 1
            tracking = TrackingFrame(
                sequence=self._sequence,
                tracking_epoch=self._epoch,
                source_timestamp_ns=max(1, pair.timestamp_ms * 1_000_000),
                bridge_monotonic_ns=max(1, int(self._clock_ns())),
                left_active=pair.left.active,
                right_active=pair.right.active,
                head_valid=False,
                left_scale=pair.left.scale,
                right_scale=pair.right.scale,
                head_pose=_identity_head(),
                left_hand=pair.left.joints,
                right_hand=pair.right.joints,
            )
            output.append(encode_tracking(tracking))
            self._published += 1
        return output

    @staticmethod
    def _event_parts(
        event: CallbackEvent | tuple[str, bytes] | Any,
    ) -> tuple[str, bytes, int]:
        if isinstance(event, CallbackEvent):
            device_id, raw, event_type = event.device_id, event.data, event.event_type
        elif isinstance(event, tuple) and len(event) == 2:
            device_id, raw, event_type = (*event, PXREA_DEVICE_CUSTOM)
        elif isinstance(event, Mapping):
            device_id = event.get("device_id", "")
            raw = event.get("data", b"")
            event_type = int(event.get("event_type", PXREA_DEVICE_CUSTOM))
        else:
            device_id, raw = event.device_id, event.data
            event_type = int(getattr(event, "event_type", PXREA_DEVICE_CUSTOM))
        if not isinstance(device_id, str):
            raise ValueError("invalid device_id")
        if not isinstance(raw, bytes):
            raw = bytes(raw)
        return device_id, raw, int(event_type)


def _identity_head() -> tuple[float, ...]:
    return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)


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


class BridgeWorker:
    """Run BridgeCore on a worker thread and keep publication out of callbacks."""

    def __init__(
        self,
        queue: BoundedCallbackQueue,
        core: BridgeCore,
        publisher: Callable[[bytes], None] | None = None,
        status_publisher: Callable[[bytes], None] | None = None,
    ) -> None:
        self.queue = queue
        self.core = core
        self.publisher = publisher
        self.status_publisher = status_publisher
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(
            target=self._run, name="pxrea-worker", daemon=True
        )
        self._thread.start()
        self._publish_status()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join()
            self._thread = None
        self._publish_status()

    def _publish_status(self) -> None:
        if self.status_publisher is not None:
            self.status_publisher(
                self.core.status_json(self.queue.dropped_overflow).encode()
            )

    def _run(self) -> None:
        while not self._stop.is_set() or self.queue.qsize():
            event = self.queue.get(timeout=0.05)
            if event is None:
                continue
            for payload in self.core.accept_event(event):
                if self.publisher is not None:
                    self.publisher(payload)
            self._publish_status()


def _read_fake_events(path: Path) -> Iterable[tuple[CallbackEvent, int]]:
    required = {"device_id", "data_hex", "delay_ms"}
    with path.open(encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            try:
                value = json.loads(line)
                if not isinstance(value, dict) or set(value) != required:
                    raise ValueError("expected device_id/data_hex/delay_ms")
                device_id = value["device_id"]
                data_hex = value["data_hex"]
                delay_ms = value["delay_ms"]
                if (
                    not isinstance(device_id, str)
                    or not device_id
                    or not isinstance(data_hex, str)
                    or isinstance(delay_ms, bool)
                    or not isinstance(delay_ms, Integral)
                    or delay_ms < 0
                ):
                    raise ValueError("invalid JSONL field")
                raw = bytes.fromhex(data_hex)
            except (json.JSONDecodeError, TypeError, ValueError) as exc:
                raise ValueError(f"malformed JSONL line {line_number}: {exc}") from exc
            yield CallbackEvent(device_id, raw), int(delay_ms)


def _run_fake_source(
    path: Path,
    publisher: Callable[[bytes], None] | None = None,
    status_publisher: Callable[[bytes], None] | None = None,
    *,
    listen: bool = False,
    endpoint: str = "tcp/127.0.0.1:7447",
    key: str = TRACKING_KEY,
) -> int:
    queue = BoundedCallbackQueue()
    core = BridgeCore()
    node = None
    worker = None
    stop = threading.Event()
    try:
        if publisher is None or status_publisher is None:
            from .zenoh_transport import LatestSample, ZenohNode, peer_config
            node = ZenohNode(peer_config(listen=listen, endpoint=endpoint))
            if publisher is None:
                publisher = node.declare_publisher(key).put
            if status_publisher is None:
                status_publisher = node.declare_publisher(STATUS_BRIDGE_KEY).put
        from .zenoh_transport import LatestSample
        control_mailbox = LatestSample()
        if node is not None:
            node.declare_latest_subscriber(CONTROL_KEY, decode_control, control_mailbox)
        old_handlers = _install_signal_handlers(stop)
        core.set_ready()
        worker = BridgeWorker(queue, core, publisher, status_publisher)
        worker.start()
        generation = 0
        for event, delay_ms in _read_fake_events(path):
            sample = control_mailbox.take_new(generation)
            if sample is not None:
                generation, control = sample
                try:
                    core.accept_control(control)
                except ValueError:
                    pass
                worker._publish_status()
                if core._shutdown:
                    stop.set()
            if stop.is_set():
                break
            queue.put(event)
            if delay_ms:
                stop.wait(delay_ms / 1000.0)
    except (OSError, ValueError) as exc:
        print(str(exc), file=sys.stderr)
        return 2
    finally:
        core.shutdown()
        if worker is not None:
            worker.stop()
        if node is not None:
            node.close()
        _restore_signal_handlers(old_handlers)
    print(core.status_json(queue.dropped_overflow))
    return 0


def _run_sdk(args: argparse.Namespace) -> int:
    if not args.sdk_library:
        print("--sdk-library is required without --fake-source-jsonl", file=sys.stderr)
        return 2
    queue = BoundedCallbackQueue()
    core = BridgeCore(selected_device_id=args.device_id)
    stop = threading.Event()
    node = None
    client = None
    worker = None
    old_handlers: dict[int, Any] = {}
    try:
        from .zenoh_transport import LatestSample, ZenohNode, peer_config

        node = ZenohNode(peer_config(listen=args.listen, endpoint=args.endpoint))
        publisher = node.declare_publisher(args.key)
        status_publisher = node.declare_publisher(STATUS_BRIDGE_KEY)
        control_mailbox = LatestSample()
        node.declare_latest_subscriber(CONTROL_KEY, decode_control, control_mailbox)
        worker = BridgeWorker(queue, core, publisher.put, status_publisher.put)
        client = PXREAClient.load_library(args.sdk_library)
        client.queue = queue
        old_handlers = _install_signal_handlers(stop)
        worker.start()
        with client:
            core.set_ready()
            worker._publish_status()
            generation = 0
            while not stop.wait(0.05):
                sample = control_mailbox.take_new(generation)
                if sample is None:
                    continue
                generation, control = sample
                try:
                    core.accept_control(control)
                except ValueError:
                    continue
                worker._publish_status()
                if core._shutdown:
                    stop.set()
    finally:
        if client is not None:
            client.close()
        core.shutdown()
        if worker is not None:
            worker.stop()
        if node is not None:
            node.close()
        _restore_signal_handlers(old_handlers)
    return 0

def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fake-source-jsonl", type=Path)
    parser.add_argument("--sdk-library")
    parser.add_argument("--device-id")
    parser.add_argument("--key", default=TRACKING_KEY)
    parser.add_argument("--endpoint", default="tcp/127.0.0.1:7447")
    parser.add_argument("--listen", action="store_true")
    args = parser.parse_args(argv)
    if args.fake_source_jsonl is not None:
        return _run_fake_source(
            args.fake_source_jsonl,
            listen=args.listen,
            endpoint=args.endpoint,
            key=args.key,
        )
    return _run_sdk(args)


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = ["BridgeCore", "BridgeStatus", "BridgeWorker", "main"]
