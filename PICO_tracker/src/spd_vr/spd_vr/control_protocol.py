"""Canonical codec and idempotence gate for SPD control v1 frames."""

from __future__ import annotations

import struct
from dataclasses import dataclass
from enum import IntEnum

from .arm_target_protocol import crc32

CONTROL_PACKET_SIZE = 40
_UINT64_MAX = (1 << 64) - 1
_INT64_MAX = (1 << 63) - 1


class ControlCommand(IntEnum):
    START = 1
    PAUSE = 2
    RESUME = 3
    REALIGN = 4
    RESET = 5
    SHUTDOWN = 6


class ControlProtocolError(ValueError):
    """Raised when a control frame violates the canonical contract."""

    def __init__(self, code: str, detail: str | None = None) -> None:
        self.code = code
        super().__init__(f"{code}: {detail}" if detail else code)


@dataclass(frozen=True)
class ControlFrame:
    sequence: int
    monotonic_timestamp_ns: int
    command: ControlCommand

    def __post_init__(self) -> None:
        object.__setattr__(self, "sequence", int(self.sequence))
        object.__setattr__(
            self, "monotonic_timestamp_ns", int(self.monotonic_timestamp_ns)
        )
        try:
            command = ControlCommand(self.command)
        except (TypeError, ValueError) as exc:
            raise ControlProtocolError("invalid_command", str(self.command)) from exc
        object.__setattr__(self, "command", command)


def _validate_frame(frame: ControlFrame) -> None:
    if not 0 <= frame.sequence <= _UINT64_MAX:
        raise ControlProtocolError("invalid_metadata", "sequence")
    if not 0 < frame.monotonic_timestamp_ns <= _INT64_MAX:
        raise ControlProtocolError("invalid_metadata", "monotonic_timestamp_ns")


def encode_control_packet(frame: ControlFrame) -> bytes:
    if not isinstance(frame, ControlFrame):
        raise ControlProtocolError("invalid_frame")
    _validate_frame(frame)
    packet = bytearray(CONTROL_PACKET_SIZE)
    packet[:4] = b"SVTC"
    struct.pack_into(
        "<HHIIQqQ",
        packet,
        4,
        1,
        int(frame.command),
        CONTROL_PACKET_SIZE,
        0,
        frame.sequence,
        frame.monotonic_timestamp_ns,
        0,
    )
    struct.pack_into("<I", packet, 12, crc32(packet[16:]))
    return bytes(packet)


def decode_control_packet(packet: bytes | bytearray | memoryview) -> ControlFrame:
    packet = bytes(packet)
    if len(packet) != CONTROL_PACKET_SIZE:
        raise ControlProtocolError("wrong_size", str(len(packet)))
    if packet[:4] != b"SVTC":
        raise ControlProtocolError("wrong_magic")
    version, command_value, declared_size, expected_crc = struct.unpack_from(
        "<HHII", packet, 4
    )
    if version != 1:
        raise ControlProtocolError("wrong_version", str(version))
    try:
        command = ControlCommand(command_value)
    except ValueError as exc:
        raise ControlProtocolError("invalid_command", str(command_value)) from exc
    if declared_size != CONTROL_PACKET_SIZE:
        raise ControlProtocolError("wrong_declared_size", str(declared_size))
    if expected_crc != crc32(packet[16:]):
        raise ControlProtocolError("crc_mismatch")
    sequence, timestamp_ns, reserved = struct.unpack_from("<QqQ", packet, 16)
    if reserved != 0:
        raise ControlProtocolError("non_zero_reserved")
    frame = ControlFrame(sequence, timestamp_ns, command)
    _validate_frame(frame)
    return frame


class ControlSequenceGate:
    """Ensure duplicate command IDs are idempotent and rollbacks fail closed."""

    def __init__(self) -> None:
        self.last_sequence: int | None = None

    def reset(self) -> None:
        self.last_sequence = None

    def accept(self, frame: ControlFrame) -> bool:
        if not isinstance(frame, ControlFrame):
            raise ControlProtocolError("invalid_frame")
        if self.last_sequence is None:
            self.last_sequence = frame.sequence
            return True
        if frame.sequence == self.last_sequence:
            return False
        if frame.sequence < self.last_sequence:
            raise ControlProtocolError("out_of_order")
        self.last_sequence = frame.sequence
        return True


__all__ = [
    "CONTROL_PACKET_SIZE",
    "ControlCommand",
    "ControlFrame",
    "ControlProtocolError",
    "ControlSequenceGate",
    "decode_control_packet",
    "encode_control_packet",
]
