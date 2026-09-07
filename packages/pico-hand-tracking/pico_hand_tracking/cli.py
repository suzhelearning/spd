"""Command-line subscriber for the PICO_2 hand stream."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import time

from .protocol import HandFrame
from .receiver import Pico2Receiver


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=10002)
    parser.add_argument("--device-port", type=int)
    parser.add_argument("--adb-path", default="adb")
    parser.add_argument("--adb-serial", default=os.environ.get("PICO_ADB_SERIAL"))
    parser.add_argument("--no-adb-forward", action="store_true")
    parser.add_argument("--reconnect", type=float, default=2.0)
    parser.add_argument("--print", dest="print_frames", action="store_true")
    parser.add_argument("--save-jsonl", type=Path)
    parser.add_argument("--duration", type=float, default=None)
    args = parser.parse_args(argv)
    if args.duration is not None and args.duration < 0.0:
        parser.error("--duration must be non-negative")

    output = None
    if args.save_jsonl is not None:
        args.save_jsonl.parent.mkdir(parents=True, exist_ok=True)
        output = args.save_jsonl.open("a", encoding="utf-8")
    last_print = 0.0

    def on_frame(frame: HandFrame) -> None:
        nonlocal last_print
        if output is not None:
            output.write(json.dumps(frame.as_dict(), separators=(",", ":")) + "\n")
            output.flush()
        now = time.monotonic()
        if not args.print_frames or now - last_print < 0.25:
            return
        last_print = now
        left = frame.left.wrist.position
        right = frame.right.wrist.position
        print(
            f"[{frame.timestamp_ms:8d} ms] "
            f"head=({frame.head.position[0]:+.3f},{frame.head.position[1]:+.3f},{frame.head.position[2]:+.3f}) "
            f"left_wrist=({left[0]:+.3f},{left[1]:+.3f},{left[2]:+.3f}) "
            f"right_wrist=({right[0]:+.3f},{right[1]:+.3f},{right[2]:+.3f}) "
            f"hands=L{int(frame.left.valid)}/R{int(frame.right.valid)}",
            flush=True,
        )

    receiver = Pico2Receiver(
        on_frame,
        host=args.host,
        port=args.port,
        device_port=args.device_port,
        adb_path=args.adb_path,
        adb_serial=args.adb_serial,
        reconnect_seconds=max(args.reconnect, 0.1),
        auto_adb_forward=not args.no_adb_forward,
    )
    receiver.start()
    deadline = None if args.duration is None else time.monotonic() + args.duration
    try:
        while receiver.running:
            if deadline is not None and time.monotonic() >= deadline:
                break
            receiver.join(timeout=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        receiver.stop()
        receiver.join(timeout=1.0)
        if output is not None:
            output.close()
    return 0


__all__ = ["main"]
