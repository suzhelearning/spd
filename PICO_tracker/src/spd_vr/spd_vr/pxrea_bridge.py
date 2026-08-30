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
from .pxrea_sdk import BoundedCallbackQueue, CallbackEvent, PXREAClient
from .wire import TrackingFrame, encode_tracking


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

    def status(self) -> dict[str, Any]:
        return {
            "device_id": self.selected_device_id,
            "tracking_epoch": self._epoch,
            "published": self._published,
            "invalid_payloads": self._invalid_payloads,
            "device_selection_ambiguous": self._ambiguous,
        }

    def status_json(self) -> str:
        return json.dumps(self.status(), sort_keys=True, separators=(",", ":"))

    def accept_event(self, event: CallbackEvent | tuple[str, bytes] | Any) -> list[bytes]:
        device_id, raw = self._event_parts(event)
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
    def _event_parts(event: CallbackEvent | tuple[str, bytes] | Any) -> tuple[str, bytes]:
        if isinstance(event, CallbackEvent):
            device_id, raw = event.device_id, event.data
        elif isinstance(event, tuple) and len(event) == 2:
            device_id, raw = event
        else:
            device_id, raw = event.device_id, event.data
        if not isinstance(device_id, str) or not device_id:
            raise ValueError("invalid device_id")
        if not isinstance(raw, bytes):
            raw = bytes(raw)
        return device_id, raw


def _identity_head() -> tuple[float, ...]:
    return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)


class BridgeWorker:
    """Run BridgeCore on a worker thread and keep publication out of callbacks."""

    def __init__(
        self,
        queue: BoundedCallbackQueue,
        core: BridgeCore,
        publisher: Callable[[bytes], None] | None = None,
    ) -> None:
        self.queue = queue
        self.core = core
        self.publisher = publisher
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(target=self._run, name="pxrea-worker", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join()
            self._thread = None

    def _run(self) -> None:
        while not self._stop.is_set() or self.queue.qsize():
            event = self.queue.get(timeout=0.05)
            if event is None:
                continue
            for payload in self.core.accept_event(event):
                if self.publisher is not None:
                    self.publisher(payload)


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


def _run_fake_source(path: Path) -> int:
    queue = BoundedCallbackQueue()
    core = BridgeCore()
    worker = BridgeWorker(queue, core)
    worker.start()
    try:
        for event, delay_ms in _read_fake_events(path):
            queue.put(event)
            if delay_ms:
                time.sleep(delay_ms / 1000.0)
    except (OSError, ValueError) as exc:
        worker.stop()
        print(str(exc), file=sys.stderr)
        return 2
    worker.stop()
    print(core.status_json())
    return 0


def _run_sdk(args: argparse.Namespace) -> int:
    if not args.sdk_library:
        print("--sdk-library is required without --fake-source-jsonl", file=sys.stderr)
        return 2
    queue = BoundedCallbackQueue()
    core = BridgeCore(selected_device_id=args.device_id)
    stop = threading.Event()
    node = None
    publisher = None
    client = None
    old_handlers: dict[int, Any] = {}
    try:
        from .zenoh_transport import ZenohNode, peer_config

        node = ZenohNode(peer_config(listen=args.listen, endpoint=args.endpoint))
        publisher = node.declare_publisher(args.key)
        worker = BridgeWorker(queue, core, publisher.put)
        client = PXREAClient.load_library(args.sdk_library)
        client.queue = queue
        for sig in (signal.SIGINT, signal.SIGTERM):
            old_handlers[sig] = signal.signal(sig, lambda *_: stop.set())
        worker.start()
        with client:
            while not stop.wait(0.2):
                pass
        worker.stop()
        return 0
    finally:
        if client is not None:
            client.close()
        if node is not None:
            node.close()
        for sig, handler in old_handlers.items():
            signal.signal(sig, handler)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fake-source-jsonl", type=Path)
    parser.add_argument("--sdk-library")
    parser.add_argument("--device-id")
    parser.add_argument("--key", default="spd/pico/tracking")
    parser.add_argument("--endpoint", default="tcp/127.0.0.1:7447")
    parser.add_argument("--listen", action="store_true")
    args = parser.parse_args(argv)
    if args.fake_source_jsonl is not None:
        return _run_fake_source(args.fake_source_jsonl)
    return _run_sdk(args)


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = ["BridgeCore", "BridgeStatus", "BridgeWorker", "main"]
