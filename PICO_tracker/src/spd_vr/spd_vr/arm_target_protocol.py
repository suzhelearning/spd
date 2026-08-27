"""Cross-language decoder for the 272-byte SPD arm target UDP packet."""

from __future__ import annotations

import struct
from dataclasses import dataclass, replace
from enum import IntEnum
from math import isfinite
from typing import Iterable

PACKET_SIZE = 272
CRC_OFFSET = PACKET_SIZE - 4
LEFT_Q_OFFSET = 44
RIGHT_Q_OFFSET = LEFT_Q_OFFSET + 7 * 8
LEFT_QDOT_OFFSET = RIGHT_Q_OFFSET + 7 * 8
RIGHT_QDOT_OFFSET = LEFT_QDOT_OFFSET + 7 * 8
LEFT_VALID = 1
RIGHT_VALID = 2


class ArmTargetHoldReason(IntEnum):
    NONE = 0
    INPUT_STALE = 1
    SOLVER_FAILURE = 2
    PAUSED = 3


class ArmTargetProtocolError(ValueError):
    """Raised when a packet fails the structural or semantic wire contract."""

    def __init__(self, code: str, detail: str | None = None) -> None:
        self.code = code
        super().__init__(f"{code}: {detail}" if detail else code)


@dataclass(frozen=True)
class ArmTargetFrame:
    sequence: int
    tracking_epoch: int
    source_timestamp_ns: int
    control_timestamp_ns: int
    valid_mask: int
    hold_reason: ArmTargetHoldReason
    left_q: tuple[float, ...]
    right_q: tuple[float, ...]
    left_qdot: tuple[float, ...]
    right_qdot: tuple[float, ...]

    def __post_init__(self) -> None:
        for name in ("left_q", "right_q", "left_qdot", "right_qdot"):
            value = tuple(float(item) for item in getattr(self, name))
            if len(value) != 7:
                raise ArmTargetProtocolError("wrong_vector_size", name)
            object.__setattr__(self, name, value)


def crc32(data: bytes) -> int:
    crc = 0xFFFFFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            mask = -(crc & 1) & 0xFFFFFFFF
            crc = ((crc >> 1) ^ (0xEDB88320 & mask)) & 0xFFFFFFFF
    return crc ^ 0xFFFFFFFF


def _valid_mask_and_reason(valid_mask: int, reason: ArmTargetHoldReason) -> bool:
    if valid_mask & ~0x03:
        return False
    if reason is ArmTargetHoldReason.NONE:
        return valid_mask == (LEFT_VALID | RIGHT_VALID)
    return valid_mask != (LEFT_VALID | RIGHT_VALID)


def _finite(values: Iterable[float]) -> bool:
    return all(isfinite(float(value)) for value in values)


def _read_vector(packet: bytes, offset: int) -> tuple[float, ...]:
    return tuple(struct.unpack_from("<7d", packet, offset))


def _coerce_frame(frame: ArmTargetFrame | dict) -> ArmTargetFrame:
    if isinstance(frame, ArmTargetFrame):
        return frame
    try:
        reason = frame["hold_reason"]
        if not isinstance(reason, ArmTargetHoldReason):
            reason = ArmTargetHoldReason(int(reason))
        return ArmTargetFrame(
            sequence=int(frame["sequence"]),
            tracking_epoch=int(frame["tracking_epoch"]),
            source_timestamp_ns=int(frame["source_timestamp_ns"]),
            control_timestamp_ns=int(frame["control_timestamp_ns"]),
            valid_mask=int(frame["valid_mask"]),
            hold_reason=reason,
            left_q=tuple(frame["left_q"]),
            right_q=tuple(frame["right_q"]),
            left_qdot=tuple(frame["left_qdot"]),
            right_qdot=tuple(frame["right_qdot"]),
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise ArmTargetProtocolError("invalid_frame", str(exc)) from exc


def encode_packet(frame: ArmTargetFrame | dict) -> bytes:
    frame = _coerce_frame(frame)
    if (
        frame.sequence <= 0
        or frame.tracking_epoch <= 0
        or frame.source_timestamp_ns <= 0
        or frame.control_timestamp_ns <= 0
    ):
        raise ArmTargetProtocolError("invalid_metadata")
    if not _valid_mask_and_reason(frame.valid_mask, frame.hold_reason):
        raise ArmTargetProtocolError("invalid_valid_mask_or_hold_reason")
    if not all(_finite(getattr(frame, name)) for name in ("left_q", "right_q", "left_qdot", "right_qdot")):
        raise ArmTargetProtocolError("non_finite_value")

    packet = bytearray(PACKET_SIZE)
    packet[0:4] = b"SPDA"
    struct.pack_into(
        "<HHQQQQBBH",
        packet,
        4,
        1,
        PACKET_SIZE,
        frame.sequence,
        frame.tracking_epoch,
        frame.source_timestamp_ns,
        frame.control_timestamp_ns,
        frame.valid_mask,
        int(frame.hold_reason),
        0,
    )
    for offset, values in (
        (LEFT_Q_OFFSET, frame.left_q),
        (RIGHT_Q_OFFSET, frame.right_q),
        (LEFT_QDOT_OFFSET, frame.left_qdot),
        (RIGHT_QDOT_OFFSET, frame.right_qdot),
    ):
        struct.pack_into("<7d", packet, offset, *values)
    struct.pack_into("<I", packet, CRC_OFFSET, crc32(packet[:CRC_OFFSET]))
    return bytes(packet)


def decode_packet(
    packet: bytes | bytearray | memoryview,
    *,
    now_ns: int | None = None,
    max_age_ns: int | None = None,
    last_sequence: int | None = None,
    last_epoch: int | None = None,
) -> ArmTargetFrame:
    packet = bytes(packet)
    if len(packet) != PACKET_SIZE:
        raise ArmTargetProtocolError("wrong_size", str(len(packet)))
    if packet[:4] != b"SPDA":
        raise ArmTargetProtocolError("wrong_magic")
    version, declared_size = struct.unpack_from("<HH", packet, 4)
    if version != 1:
        raise ArmTargetProtocolError("wrong_version", str(version))
    if declared_size != PACKET_SIZE:
        raise ArmTargetProtocolError("wrong_declared_size", str(declared_size))
    sequence, epoch, source_ns, control_ns = struct.unpack_from("<QQQQ", packet, 8)
    valid_mask, reason_value, reserved = struct.unpack_from("<BBH", packet, 40)
    try:
        reason = ArmTargetHoldReason(reason_value)
    except ValueError as exc:
        raise ArmTargetProtocolError("invalid_hold_reason", str(reason_value)) from exc
    if valid_mask & ~0x03:
        raise ArmTargetProtocolError("invalid_valid_mask", str(valid_mask))
    if not _valid_mask_and_reason(valid_mask, reason):
        raise ArmTargetProtocolError("invalid_valid_mask_or_hold_reason")
    if reserved != 0:
        raise ArmTargetProtocolError("non_zero_reserved")
    expected_crc = struct.unpack_from("<I", packet, CRC_OFFSET)[0]
    if expected_crc != crc32(packet[:CRC_OFFSET]):
        raise ArmTargetProtocolError("crc_mismatch")
    if not all((sequence, epoch, source_ns, control_ns)):
        raise ArmTargetProtocolError("invalid_metadata")
    frame = ArmTargetFrame(
        sequence=sequence,
        tracking_epoch=epoch,
        source_timestamp_ns=source_ns,
        control_timestamp_ns=control_ns,
        valid_mask=valid_mask,
        hold_reason=reason,
        left_q=_read_vector(packet, LEFT_Q_OFFSET),
        right_q=_read_vector(packet, RIGHT_Q_OFFSET),
        left_qdot=_read_vector(packet, LEFT_QDOT_OFFSET),
        right_qdot=_read_vector(packet, RIGHT_QDOT_OFFSET),
    )
    if not all(_finite(getattr(frame, name)) for name in ("left_q", "right_q", "left_qdot", "right_qdot")):
        raise ArmTargetProtocolError("non_finite_value")
    if last_epoch is not None:
        if epoch < last_epoch:
            raise ArmTargetProtocolError("epoch_rollback")
        if epoch == last_epoch and last_sequence is not None and sequence <= last_sequence:
            raise ArmTargetProtocolError("out_of_order")
    if now_ns is not None and max_age_ns is not None:
        if max_age_ns < 0:
            raise ArmTargetProtocolError("invalid_max_age")
        age = int(now_ns) - int(control_ns)
        if age > max_age_ns:
            frame = replace(
                frame,
                valid_mask=0,
                hold_reason=ArmTargetHoldReason.INPUT_STALE,
            )
    return frame


class ArmTargetStreamDecoder:
    """Stateful epoch/sequence/freshness gate for simulator snapshots."""

    def __init__(self, max_age_ns: int = 50_000_000) -> None:
        if max_age_ns < 0:
            raise ValueError("max_age_ns must be non-negative")
        self.max_age_ns = int(max_age_ns)
        self.last_epoch: int | None = None
        self.last_sequence: int | None = None

    def reset(self) -> None:
        self.last_epoch = None
        self.last_sequence = None

    def decode(self, packet: bytes, *, now_ns: int | None = None) -> ArmTargetFrame:
        frame = decode_packet(
            packet,
            now_ns=now_ns,
            max_age_ns=self.max_age_ns if now_ns is not None else None,
            last_sequence=self.last_sequence,
            last_epoch=self.last_epoch,
        )
        if self.last_epoch != frame.tracking_epoch:
            self.last_sequence = None
        self.last_epoch = frame.tracking_epoch
        self.last_sequence = frame.sequence
        return frame


# Names used by the C++/Python fixture tests and by downstream callers.
decode_arm_target_packet = decode_packet
encode_arm_target_packet = encode_packet

__all__ = [
    "ArmTargetFrame",
    "ArmTargetHoldReason",
    "ArmTargetProtocolError",
    "ArmTargetStreamDecoder",
    "PACKET_SIZE",
    "crc32",
    "decode_arm_target_packet",
    "decode_packet",
    "encode_arm_target_packet",
    "encode_packet",
]
