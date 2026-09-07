"""PICO_2 type-0x40 hand-tracking frame protocol."""

from __future__ import annotations

from dataclasses import dataclass
import math
import struct
from typing import Any

MAGIC = 0xAB
TYPE_HAND_FRAME = 0x40
PROTOCOL_VERSION = 1
JOINT_COUNT = 26
HEADER = struct.Struct("<BBqI")
POSE = struct.Struct("<7f")
JOINT_RECORD_BYTES = 4 + POSE.size + 4
HAND_BLOCK_BYTES = 4 + POSE.size + JOINT_COUNT * JOINT_RECORD_BYTES
PAYLOAD_BYTES = 4 + POSE.size + 2 * HAND_BLOCK_BYTES
MAX_PAYLOAD_BYTES = 4 * 1024 * 1024

JOINT_NAMES = (
    "palm",
    "wrist",
    "thumb_metacarpal",
    "thumb_proximal",
    "thumb_distal",
    "thumb_tip",
    "index_metacarpal",
    "index_proximal",
    "index_intermediate",
    "index_distal",
    "index_tip",
    "middle_metacarpal",
    "middle_proximal",
    "middle_intermediate",
    "middle_distal",
    "middle_tip",
    "ring_metacarpal",
    "ring_proximal",
    "ring_intermediate",
    "ring_distal",
    "ring_tip",
    "little_metacarpal",
    "little_proximal",
    "little_intermediate",
    "little_distal",
    "little_tip",
)


class PicoHandTrackingError(ValueError):
    """Raised when a PICO_2 frame violates the type-0x40 contract."""


@dataclass(frozen=True, slots=True)
class Pose:
    position: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float]
    valid: bool = True

    @property
    def values(self) -> tuple[float, ...]:
        return self.position + self.quaternion_xyzw

    def as_dict(self) -> dict[str, Any]:
        return {
            "pos": list(self.position),
            "rot": list(self.quaternion_xyzw),
            "valid": self.valid,
        }


@dataclass(frozen=True, slots=True)
class Joint(Pose):
    index: int = 0
    name: str = ""
    radius: float = 0.0

    def as_dict(self) -> dict[str, Any]:
        value = super().as_dict()
        value.update(index=self.index, name=self.name, radius=self.radius)
        return value


@dataclass(frozen=True, slots=True)
class Hand:
    valid: bool
    wrist: Pose
    joints: tuple[Joint, ...]

    def __post_init__(self) -> None:
        if len(self.joints) != JOINT_COUNT:
            raise PicoHandTrackingError(
                f"hand has {len(self.joints)} joints, expected {JOINT_COUNT}"
            )

    def as_dict(self) -> dict[str, Any]:
        return {
            "valid": self.valid,
            "wrist": self.wrist.as_dict(),
            "joints": [joint.as_dict() for joint in self.joints],
        }


@dataclass(frozen=True, slots=True)
class HandFrame:
    timestamp_ms: int
    flags: int
    head: Pose
    left: Hand
    right: Hand
    extra_bytes: int = 0

    @property
    def head_valid(self) -> bool:
        return self.head.valid

    def as_dict(self) -> dict[str, Any]:
        return {
            "timestamp_ms": self.timestamp_ms,
            "flags": self.flags,
            "head": self.head.as_dict(),
            "hands": {"left": self.left.as_dict(), "right": self.right.as_dict()},
            "extra_bytes": self.extra_bytes,
        }


def _finite(values: tuple[float, ...], label: str) -> tuple[float, ...]:
    if not all(math.isfinite(value) for value in values):
        raise PicoHandTrackingError(f"{label} contains non-finite values")
    return values


def _pose(view: memoryview, offset: int, label: str) -> tuple[Pose, int]:
    try:
        values = tuple(float(value) for value in POSE.unpack_from(view, offset))
    except struct.error as exc:
        raise PicoHandTrackingError(f"truncated {label}") from exc
    _finite(values, label)
    return Pose(values[:3], values[3:]), offset + POSE.size


def parse_hand_frame(timestamp_ms: int, payload: bytes | bytearray | memoryview) -> HandFrame:
    """Decode one PICO_2 payload, retaining optional trailing fields."""
    if isinstance(timestamp_ms, bool) or not isinstance(timestamp_ms, int) or timestamp_ms < 0:
        raise PicoHandTrackingError("timestamp_ms must be a non-negative integer")
    raw = bytes(payload)
    if len(raw) < PAYLOAD_BYTES:
        raise PicoHandTrackingError(
            f"hand frame is {len(raw)} bytes, expected at least {PAYLOAD_BYTES}"
        )

    view = memoryview(raw)
    version, flags, joint_count, _reserved = struct.unpack_from("<BBBB", view, 0)
    if version != PROTOCOL_VERSION:
        raise PicoHandTrackingError(f"unsupported hand frame version {version}")
    if joint_count != JOINT_COUNT:
        raise PicoHandTrackingError(f"unsupported joint count {joint_count}")

    offset = 4
    head, offset = _pose(view, offset, "head")
    head = Pose(head.position, head.quaternion_xyzw, bool(flags & 0x01))

    hands: dict[str, Hand] = {}
    for side, bit in (("left", 0x02), ("right", 0x04)):
        hand_valid = bool(view[offset])
        offset += 4
        wrist, offset = _pose(view, offset, f"{side}.wrist")
        joints: list[Joint] = []
        for index, name in enumerate(JOINT_NAMES):
            joint_valid = bool(view[offset])
            offset += 4
            joint_pose, offset = _pose(view, offset, f"{side}.{name}")
            try:
                radius = float(struct.unpack_from("<f", view, offset)[0])
            except struct.error as exc:
                raise PicoHandTrackingError(f"truncated {side}.{name}.radius") from exc
            offset += 4
            if not math.isfinite(radius):
                raise PicoHandTrackingError(f"{side}.{name}.radius is non-finite")
            joints.append(
                Joint(
                    position=joint_pose.position,
                    quaternion_xyzw=joint_pose.quaternion_xyzw,
                    valid=joint_valid,
                    index=index,
                    name=name,
                    radius=radius,
                )
            )
        active = hand_valid and bool(flags & bit)
        hands[side] = Hand(
            valid=active,
            wrist=Pose(wrist.position, wrist.quaternion_xyzw, active),
            joints=tuple(joints),
        )

    return HandFrame(
        timestamp_ms=timestamp_ms,
        flags=int(flags),
        head=head,
        left=hands["left"],
        right=hands["right"],
        extra_bytes=len(raw) - offset,
    )


__all__ = [
    "HEADER",
    "Hand",
    "HandFrame",
    "JOINT_COUNT",
    "JOINT_NAMES",
    "Joint",
    "MAGIC",
    "MAX_PAYLOAD_BYTES",
    "PAYLOAD_BYTES",
    "POSE",
    "PROTOCOL_VERSION",
    "PicoHandTrackingError",
    "Pose",
    "TYPE_HAND_FRAME",
    "parse_hand_frame",
]
