from pathlib import Path
import struct

import pytest

from spd_vr.arm_target_protocol import crc32
from spd_vr.control_protocol import (
    CONTROL_PACKET_SIZE,
    ControlCommand,
    ControlFrame,
    ControlProtocolError,
    ControlSequenceGate,
    decode_control_packet,
    encode_control_packet,
)

FIXTURE = Path(__file__).parent / "fixtures" / "control_v1.hex"


def fixture_frame() -> ControlFrame:
    return ControlFrame(
        sequence=202,
        monotonic_timestamp_ns=2_000_000_000,
        command=ControlCommand.REALIGN,
    )


def refresh_crc(packet: bytearray) -> None:
    struct.pack_into("<I", packet, 12, crc32(packet[16:]))


def test_matches_golden_vector_and_roundtrips_every_command():
    packet = encode_control_packet(fixture_frame())
    assert CONTROL_PACKET_SIZE == 40
    assert len(packet) == 40
    assert packet[:4] == b"SVTC"
    assert packet == bytes.fromhex(FIXTURE.read_text().strip())
    assert decode_control_packet(packet) == fixture_frame()

    for command in ControlCommand:
        frame = ControlFrame(202, 2_000_000_000, command)
        assert decode_control_packet(encode_control_packet(frame)) == frame


def test_rejects_structural_corruption_reserved_enum_and_timestamp():
    packet = bytearray(encode_control_packet(fixture_frame()))
    packet[0] = ord("X")
    with pytest.raises(ControlProtocolError, match="wrong_magic"):
        decode_control_packet(packet)

    packet = bytearray(encode_control_packet(fixture_frame()))
    struct.pack_into("<H", packet, 4, 2)
    with pytest.raises(ControlProtocolError, match="wrong_version"):
        decode_control_packet(packet)

    packet = bytearray(encode_control_packet(fixture_frame()))
    struct.pack_into("<I", packet, 8, 1)
    with pytest.raises(ControlProtocolError, match="wrong_declared_size"):
        decode_control_packet(packet)

    packet = bytearray(encode_control_packet(fixture_frame()))
    packet[20] ^= 1
    with pytest.raises(ControlProtocolError, match="crc_mismatch"):
        decode_control_packet(packet)

    packet = bytearray(encode_control_packet(fixture_frame()))
    struct.pack_into("<H", packet, 6, 7)
    refresh_crc(packet)
    with pytest.raises(ControlProtocolError, match="invalid_command"):
        decode_control_packet(packet)

    packet = bytearray(encode_control_packet(fixture_frame()))
    struct.pack_into("<Q", packet, 32, 1)
    refresh_crc(packet)
    with pytest.raises(ControlProtocolError, match="non_zero_reserved"):
        decode_control_packet(packet)

    packet = bytearray(encode_control_packet(fixture_frame()))
    struct.pack_into("<q", packet, 24, 0)
    refresh_crc(packet)
    with pytest.raises(ControlProtocolError, match="invalid_metadata"):
        decode_control_packet(packet)

    with pytest.raises(ControlProtocolError, match="wrong_size"):
        decode_control_packet(packet[:-1])


def test_duplicate_control_sequence_is_idempotent_and_rollback_rejected():
    gate = ControlSequenceGate()
    assert gate.accept(fixture_frame()) is True
    assert gate.accept(fixture_frame()) is False
    with pytest.raises(ControlProtocolError, match="out_of_order"):
        gate.accept(ControlFrame(201, 2_000_000_001, ControlCommand.PAUSE))
    assert gate.accept(ControlFrame(203, 2_000_000_002, ControlCommand.RESUME)) is True
