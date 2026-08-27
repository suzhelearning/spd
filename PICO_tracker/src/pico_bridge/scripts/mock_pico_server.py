#!/usr/bin/env python3
"""Mock PicoStreamingServer, including the atomic OpenXR hand stream.

Frame layout: [0xAB][type:1B][ts_ms:8B LE i64][payload_len:4B LE u32][payload].
The default stream emits body poses and two legal 733-byte hand payloads with
one timestamp.  ``--hand-case`` deliberately exercises host-side rejection and
reconnection paths.
"""

from __future__ import annotations

import argparse
import math
import socket
import struct
import time


XR_POSE_TYPES = ((0x03, 0.0), (0x04, 1.0), (0x05, 2.0))
BODY_POSE_TYPES = tuple((0x20 + index, 3.0 + index) for index in range(24))
POSE_TYPES = XR_POSE_TYPES + BODY_POSE_TYPES
HAND_JOINT_COUNT = 26
HAND_PAYLOAD_BYTES = 733


def make_header(frame_type: int, ts_ms: int, payload_len: int) -> bytes:
    return struct.pack("<BBqI", 0xAB, frame_type, ts_ms, payload_len)


def packet(frame_type: int, ts_ms: int, payload: bytes) -> bytes:
    return make_header(frame_type, ts_ms, len(payload)) + payload


def tiny_jpeg() -> bytes:
    # Minimal SOI+EOI payload; the bridge transports it without decoding it.
    return b"\xFF\xD8\xFF\xD9" + b"FAKE_JPEG"


def pose_payload(t: float, offset: float) -> bytes:
    return struct.pack(
        "<7f",
        math.sin(t), math.cos(t), offset,
        0.0, 0.0, 0.0, 1.0,
    )


def hand_payload(t: float, side_offset: float, active: bool = True) -> bytes:
    """Return the fixed active:u8 + scale:f32 + 26*(xyz+quat) payload."""
    payload = bytearray(struct.pack("<Bf", int(active), 1.0))
    for joint in range(HAND_JOINT_COUNT):
        if active:
            xyz = (
                side_offset + joint * 0.001 + math.sin(t) * 0.01,
                joint * 0.002 + math.cos(t) * 0.01,
                0.02 + joint * 0.001,
            )
        else:
            xyz = (0.0, 0.0, 0.0)
        payload.extend(struct.pack("<7f", *xyz, 0.0, 0.0, 0.0, 1.0))
    assert len(payload) == HAND_PAYLOAD_BYTES
    return bytes(payload)


def ble_payload(ts_ms: int, payload_byte: int) -> bytes:
    return struct.pack("<I", ts_ms) + bytes([payload_byte, payload_byte ^ 0xFF])


def hand_packets(ts_ms: int, t: float, case: str, connection_index: int) -> tuple[bytes, bytes]:
    """Build left/right packets for a named bridge test case."""
    left_active = case != "inactive"
    right_active = case != "inactive"
    left_ts = ts_ms
    right_ts = ts_ms
    left_payload = hand_payload(t, -0.1, left_active)
    right_payload = hand_payload(t, 0.1, right_active)

    if case == "truncated" and connection_index == 0:
        left_payload = left_payload[:-1]
    elif case == "timestamp-mismatch" and connection_index == 0:
        right_ts += 1
    return packet(0x38, left_ts, left_payload), packet(0x39, right_ts, right_payload)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=9999)
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--duration", type=float, default=0.0,
                    help="Run each accepted connection for this many seconds (0 = forever)")
    ap.add_argument("--record-cycle", type=float, default=0.0,
                    help="Toggle the 0x09 record flag every N seconds (0 = never send)")
    ap.add_argument(
        "--hand-case", "--hand-mode",
        choices=("valid", "truncated", "timestamp-mismatch", "inactive", "reconnect"),
        default="valid",
        help="Hand packet behavior used by protocol/runtime tests",
    )
    ap.add_argument("--no-hands", action="store_true", help="Do not emit 0x38/0x39 frames")
    args = ap.parse_args()
    if args.fps <= 0.0:
        ap.error("--fps must be positive")

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", args.port))
    srv.listen(1)
    print(f"[mock] listening on 127.0.0.1:{args.port}", flush=True)

    connection_index = 0
    reconnect_pending = args.hand_case == "reconnect"
    try:
        while True:
            conn, addr = srv.accept()
            print(f"[mock] client connected: {addr}", flush=True)
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

            start = time.time()
            frame_idx = 0
            record_flag = False
            last_toggle = start
            force_reconnect = False
            try:
                while args.duration == 0 or (time.time() - start) < args.duration:
                    ts_ms = int((time.time() - start) * 1000)
                    t = ts_ms / 1000.0

                    for ftype in (0x01, 0x02):
                        jpeg = tiny_jpeg()
                        conn.sendall(packet(ftype, ts_ms, jpeg))

                    for ftype, offset in POSE_TYPES:
                        pose = pose_payload(t, offset)
                        conn.sendall(packet(ftype, ts_ms, pose))

                    if not args.no_hands:
                        left, right = hand_packets(ts_ms, t, args.hand_case, connection_index)
                        conn.sendall(left)
                        conn.sendall(right)

                    if args.record_cycle > 0 and time.time() - last_toggle >= args.record_cycle:
                        record_flag = not record_flag
                        last_toggle = time.time()
                        value = bytes([1 if record_flag else 0])
                        conn.sendall(packet(0x09, ts_ms, value))
                        print(f"[mock] record_flag -> {int(record_flag)}", flush=True)

                    for ftype, value in ((0x10, 0xA0), (0x11, 0xB0)):
                        ble = ble_payload(ts_ms, value + (frame_idx & 0x0F))
                        conn.sendall(packet(ftype, ts_ms, ble))

                    frame_idx += 1
                    if reconnect_pending and frame_idx >= 2:
                        print("[mock] forcing reconnect", flush=True)
                        reconnect_pending = False
                        force_reconnect = True
                        break
                    time.sleep(1.0 / args.fps)
            except (BrokenPipeError, ConnectionResetError):
                print("[mock] client disconnected", flush=True)
            finally:
                conn.close()
                connection_index += 1
            if force_reconnect:
                continue
            if args.duration > 0:
                break
    finally:
        srv.close()


if __name__ == "__main__":
    main()
