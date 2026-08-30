"""Canonical NumPy-free codec for the 1,540-byte SPD tracking v1 frame."""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass, replace

from .arm_target_protocol import crc32

TRACKING_PACKET_SIZE = 1540
LEFT_ACTIVE = 1 << 0
RIGHT_ACTIVE = 1 << 1
HEAD_VALID = 1 << 2
_KNOWN_FLAGS = LEFT_ACTIVE | RIGHT_ACTIVE | HEAD_VALID
_HEAD_OFFSET = 56
_LEFT_HAND_OFFSET = 84
_RIGHT_HAND_OFFSET = 812
_POSE_SIZE = 7 * 4
_QUATERNION_NORM_TOLERANCE = 1.0e-3
_UINT64_MAX = (1 << 64) - 1
_INT64_MAX = (1 << 63) - 1


class TrackingProtocolError(ValueError):
    """Raised when a tracking frame violates the canonical wire contract."""

    def __init__(self, code: str, detail: str | None = None) -> None:
        self.code = code
        super().__init__(f"{code}: {detail}" if detail else code)


def _pose_tuple(value: object, name: str) -> tuple[float, ...]:
    try:
        pose = tuple(float(item) for item in value)  # type: ignore[arg-type]
    except (TypeError, ValueError) as exc:
        raise TrackingProtocolError("invalid_pose", name) from exc
    if len(pose) != 7:
        raise TrackingProtocolError("wrong_pose_size", name)
    return pose


def _hand_tuple(value: object, name: str) -> tuple[tuple[float, ...], ...]:
    try:
        hand = tuple(_pose_tuple(pose, name) for pose in value)  # type: ignore[arg-type]
    except TypeError as exc:
        raise TrackingProtocolError("invalid_hand", name) from exc
    if len(hand) != 26:
        raise TrackingProtocolError("wrong_hand_size", name)
    return hand


@dataclass(frozen=True)
class TrackingFrame:
    sequence: int
    tracking_epoch: int
    source_timestamp_ns: int
    bridge_monotonic_ns: int
    flags: int
    left_scale: float
    right_scale: float
    head_pose: tuple[float, ...]
    left_hand: tuple[tuple[float, ...], ...]
    right_hand: tuple[tuple[float, ...], ...]

    def __post_init__(self) -> None:
        object.__setattr__(self, "sequence", int(self.sequence))
        object.__setattr__(self, "tracking_epoch", int(self.tracking_epoch))
        object.__setattr__(self, "source_timestamp_ns", int(self.source_timestamp_ns))
        object.__setattr__(self, "bridge_monotonic_ns", int(self.bridge_monotonic_ns))
        object.__setattr__(self, "flags", int(self.flags))
        object.__setattr__(self, "left_scale", float(self.left_scale))
        object.__setattr__(self, "right_scale", float(self.right_scale))
        object.__setattr__(self, "head_pose", _pose_tuple(self.head_pose, "head_pose"))
        object.__setattr__(self, "left_hand", _hand_tuple(self.left_hand, "left_hand"))
        object.__setattr__(self, "right_hand", _hand_tuple(self.right_hand, "right_hand"))


def _quaternion_norm(pose: tuple[float, ...]) -> float:
    return math.sqrt(sum(value * value for value in pose[3:]))


def _validated_quaternion(
    pose: tuple[float, ...], *, normalize: bool
) -> tuple[float, ...]:
    norm = _quaternion_norm(pose)
    if not math.isfinite(norm) or abs(norm - 1.0) > _QUATERNION_NORM_TOLERANCE:
        raise TrackingProtocolError("invalid_quaternion")
    if not normalize:
        return pose
    return (*pose[:3], *(value / norm for value in pose[3:]))


def _validate_and_maybe_normalize(
    frame: TrackingFrame, *, normalize: bool
) -> TrackingFrame:
    if frame.flags & ~_KNOWN_FLAGS:
        raise TrackingProtocolError("unknown_flags")
    if not 0 <= frame.sequence <= _UINT64_MAX:
        raise TrackingProtocolError("invalid_metadata", "sequence")
    if not 0 < frame.tracking_epoch <= _UINT64_MAX:
        raise TrackingProtocolError("invalid_metadata", "tracking_epoch")
    if not 0 < frame.source_timestamp_ns <= _INT64_MAX:
        raise TrackingProtocolError("invalid_metadata", "source_timestamp_ns")
    if not 0 < frame.bridge_monotonic_ns <= _INT64_MAX:
        raise TrackingProtocolError("invalid_metadata", "bridge_monotonic_ns")
    if (
        not math.isfinite(frame.left_scale)
        or frame.left_scale <= 0.0
        or not math.isfinite(frame.right_scale)
        or frame.right_scale <= 0.0
    ):
        raise TrackingProtocolError("invalid_scale")
    poses = (frame.head_pose, *frame.left_hand, *frame.right_hand)
    if not all(math.isfinite(value) for pose in poses for value in pose):
        raise TrackingProtocolError("non_finite_value")
    if not frame.flags & HEAD_VALID and any(value != 0.0 for value in frame.head_pose):
        raise TrackingProtocolError("invalid_head_pose")

    head_pose = frame.head_pose
    left_hand = frame.left_hand
    right_hand = frame.right_hand
    if frame.flags & HEAD_VALID:
        head_pose = _validated_quaternion(head_pose, normalize=normalize)
    if frame.flags & LEFT_ACTIVE:
        left_hand = tuple(
            _validated_quaternion(pose, normalize=normalize) for pose in left_hand
        )
    if frame.flags & RIGHT_ACTIVE:
        right_hand = tuple(
            _validated_quaternion(pose, normalize=normalize) for pose in right_hand
        )
    if not normalize:
        return frame
    return replace(
        frame, head_pose=head_pose, left_hand=left_hand, right_hand=right_hand
    )


def encode_tracking_packet(frame: TrackingFrame) -> bytes:
    if not isinstance(frame, TrackingFrame):
        raise TrackingProtocolError("invalid_frame")
    frame = _validate_and_maybe_normalize(frame, normalize=False)
    packet = bytearray(TRACKING_PACKET_SIZE)
    packet[:4] = b"SVT1"
    try:
        struct.pack_into(
            "<HHIIQQqqff",
            packet,
            4,
            1,
            frame.flags,
            TRACKING_PACKET_SIZE,
            0,
            frame.sequence,
            frame.tracking_epoch,
            frame.source_timestamp_ns,
            frame.bridge_monotonic_ns,
            frame.left_scale,
            frame.right_scale,
        )
        struct.pack_into("<7f", packet, _HEAD_OFFSET, *frame.head_pose)
        for index, pose in enumerate(frame.left_hand):
            struct.pack_into("<7f", packet, _LEFT_HAND_OFFSET + index * _POSE_SIZE, *pose)
        for index, pose in enumerate(frame.right_hand):
            struct.pack_into("<7f", packet, _RIGHT_HAND_OFFSET + index * _POSE_SIZE, *pose)
    except (OverflowError, struct.error) as exc:
        raise TrackingProtocolError("unrepresentable_value", str(exc)) from exc
    struct.pack_into("<I", packet, 12, crc32(packet[16:]))
    return bytes(packet)


def decode_tracking_packet(packet: bytes | bytearray | memoryview) -> TrackingFrame:
    packet = bytes(packet)
    if len(packet) != TRACKING_PACKET_SIZE:
        raise TrackingProtocolError("wrong_size", str(len(packet)))
    if packet[:4] != b"SVT1":
        raise TrackingProtocolError("wrong_magic")
    version, flags = struct.unpack_from("<HH", packet, 4)
    if version != 1:
        raise TrackingProtocolError("wrong_version", str(version))
    if flags & ~_KNOWN_FLAGS:
        raise TrackingProtocolError("unknown_flags", str(flags))
    declared_size, expected_crc = struct.unpack_from("<II", packet, 8)
    if declared_size != TRACKING_PACKET_SIZE:
        raise TrackingProtocolError("wrong_declared_size", str(declared_size))
    if expected_crc != crc32(packet[16:]):
        raise TrackingProtocolError("crc_mismatch")
    sequence, epoch, source_ns, bridge_ns, left_scale, right_scale = struct.unpack_from(
        "<QQqqff", packet, 16
    )
    head_pose = tuple(struct.unpack_from("<7f", packet, _HEAD_OFFSET))
    left_hand = tuple(
        tuple(struct.unpack_from("<7f", packet, _LEFT_HAND_OFFSET + index * _POSE_SIZE))
        for index in range(26)
    )
    right_hand = tuple(
        tuple(struct.unpack_from("<7f", packet, _RIGHT_HAND_OFFSET + index * _POSE_SIZE))
        for index in range(26)
    )
    frame = TrackingFrame(
        sequence=sequence,
        tracking_epoch=epoch,
        source_timestamp_ns=source_ns,
        bridge_monotonic_ns=bridge_ns,
        flags=flags,
        left_scale=left_scale,
        right_scale=right_scale,
        head_pose=head_pose,
        left_hand=left_hand,
        right_hand=right_hand,
    )
    return _validate_and_maybe_normalize(frame, normalize=True)


class TrackingStreamDecoder:
    """Decode tracking bytes and enforce epoch, sequence, and timestamp order."""

    def __init__(self) -> None:
        self.last_epoch: int | None = None
        self.last_sequence: int | None = None
        self.last_source_timestamp_ns: int | None = None

    def reset(self) -> None:
        self.last_epoch = None
        self.last_sequence = None
        self.last_source_timestamp_ns = None

    def decode(self, packet: bytes | bytearray | memoryview) -> TrackingFrame:
        frame = decode_tracking_packet(packet)
        if self.last_epoch is not None:
            if frame.tracking_epoch < self.last_epoch:
                raise TrackingProtocolError("epoch_rollback")
            if frame.tracking_epoch == self.last_epoch:
                if self.last_sequence is not None and frame.sequence <= self.last_sequence:
                    raise TrackingProtocolError("out_of_order")
                if (
                    self.last_source_timestamp_ns is not None
                    and frame.source_timestamp_ns <= self.last_source_timestamp_ns
                ):
                    raise TrackingProtocolError("timestamp_rollback")
        self.last_epoch = frame.tracking_epoch
        self.last_sequence = frame.sequence
        self.last_source_timestamp_ns = frame.source_timestamp_ns
        return frame


__all__ = [
    "HEAD_VALID",
    "LEFT_ACTIVE",
    "RIGHT_ACTIVE",
    "TRACKING_PACKET_SIZE",
    "TrackingFrame",
    "TrackingProtocolError",
    "TrackingStreamDecoder",
    "decode_tracking_packet",
    "encode_tracking_packet",
]
