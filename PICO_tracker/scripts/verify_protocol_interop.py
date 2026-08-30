#!/usr/bin/env python3
"""Verify C++/Python semantic and byte-level protocol interoperability."""

from __future__ import annotations

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import subprocess
from typing import Any

from spd_vr.arm_target_protocol import (
    ArmTargetFrame,
    ArmTargetHoldReason,
    decode_arm_target_packet,
    encode_arm_target_packet,
)
from spd_vr.control_protocol import (
    ControlCommand,
    ControlFrame,
    decode_control_packet,
    encode_control_packet,
)
from spd_vr.tracking_protocol import (
    HEAD_VALID,
    LEFT_ACTIVE,
    RIGHT_ACTIVE,
    TrackingFrame,
    decode_tracking_packet,
    encode_tracking_packet,
)

_FIXTURES = Path(__file__).parents[1] / "src" / "spd_vr" / "test" / "fixtures"


def tracking_frame() -> TrackingFrame:
    return TrackingFrame(
        sequence=101,
        tracking_epoch=7,
        source_timestamp_ns=1_000_000_000,
        bridge_monotonic_ns=1_000_000_500,
        flags=LEFT_ACTIVE | RIGHT_ACTIVE | HEAD_VALID,
        left_scale=1.25,
        right_scale=0.75,
        head_pose=(1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0),
        left_hand=tuple(
            (index / 8, -index / 16, index / 32, 0.0, 0.0, 0.0, 1.0)
            for index in range(26)
        ),
        right_hand=tuple(
            (-index / 8, index / 16, -index / 32, 0.0, 0.0, 0.0, 1.0)
            for index in range(26)
        ),
    )


def control_frame() -> ControlFrame:
    return ControlFrame(202, 2_000_000_000, ControlCommand.REALIGN)


def arm_target_frame() -> ArmTargetFrame:
    return ArmTargetFrame(
        sequence=17,
        tracking_epoch=9,
        source_timestamp_ns=1_000_000_000,
        control_timestamp_ns=1_000_001_000,
        valid_mask=1,
        left_hold_reason=ArmTargetHoldReason.NONE,
        right_hold_reason=ArmTargetHoldReason.INACTIVE,
        left_q=tuple(0.1 * index for index in range(7)),
        right_q=tuple(-0.2 * index for index in range(7)),
        left_qdot=tuple(0.3 * index for index in range(7)),
        right_qdot=tuple(-0.4 * index for index in range(7)),
    )


def semantic_dict(frame: object) -> dict[str, Any]:
    return json.loads(json.dumps(asdict(frame)))


def tool_bytes(tool: Path, command: str, packet: bytes | None = None) -> bytes:
    completed = subprocess.run(
        [str(tool), command],
        input=packet,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=True,
    )
    return completed.stdout


def verify_one(
    *,
    name: str,
    tool: Path,
    frame: object,
    encode: Any,
    decode: Any,
) -> None:
    fixture = bytes.fromhex((_FIXTURES / f"{name}.hex").read_text().strip())
    python_bytes = encode(frame)
    if python_bytes != fixture:
        raise AssertionError(f"{name}: Python encoding differs from golden vector")

    cpp_bytes = tool_bytes(tool, f"encode-{name.replace('_v1', '').replace('_v2', '').replace('_', '-')}")
    if cpp_bytes != fixture:
        raise AssertionError(f"{name}: C++ encoding differs from golden vector")
    if decode(cpp_bytes) != frame:
        raise AssertionError(f"{name}: Python did not preserve every C++ semantic field")

    decode_command = f"decode-{name.replace('_v1', '').replace('_v2', '').replace('_', '-')}"
    cpp_semantics = json.loads(tool_bytes(tool, decode_command, python_bytes))
    expected_semantics = semantic_dict(decode(python_bytes))
    if cpp_semantics != expected_semantics:
        raise AssertionError(
            f"{name}: C++ semantic mismatch\nexpected={expected_semantics!r}\nactual={cpp_semantics!r}"
        )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--tool", type=Path, required=True)
    args = parser.parse_args()
    tool = args.tool.resolve()
    if not tool.is_file():
        parser.error(f"fixture tool not found: {tool}")

    verify_one(
        name="tracking_v1",
        tool=tool,
        frame=tracking_frame(),
        encode=encode_tracking_packet,
        decode=decode_tracking_packet,
    )
    verify_one(
        name="control_v1",
        tool=tool,
        frame=control_frame(),
        encode=encode_control_packet,
        decode=decode_control_packet,
    )
    verify_one(
        name="arm_target_v2",
        tool=tool,
        frame=arm_target_frame(),
        encode=encode_arm_target_packet,
        decode=decode_arm_target_packet,
    )
    print("tracking=ok control=ok arm_target=ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
