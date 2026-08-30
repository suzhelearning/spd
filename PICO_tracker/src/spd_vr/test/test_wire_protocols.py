from dataclasses import replace
import struct

import numpy as np
import pytest

from spd_vr.wire import (
    ARM_TARGET_PACKET_SIZE,
    ARM_TARGETS_KEY,
    CONTROL_KEY,
    CONTROL_PACKET_SIZE,
    STATUS_BRIDGE_KEY,
    STATUS_IK_KEY,
    STATUS_VIEWER_KEY,
    TRACKING_KEY,
    TRACKING_PACKET_SIZE,
    ControlCommand,
    ControlFrame,
    ControlProtocolError,
    ControlSequenceGate,
    TrackingFrame,
    TrackingProtocolError,
    TrackingStreamGate,
    crc32,
    decode_control,
    decode_tracking,
    encode_control,
    encode_tracking,
)


def identity_tracking_frame(*, sequence: int = 1, epoch: int = 1) -> TrackingFrame:
    hand = np.zeros((26, 7), dtype=np.float32)
    hand[:, 6] = 1.0
    head = np.array([0.0, 0.0, 1.6, 0.0, 0.0, 0.0, 1.0], dtype=np.float32)
    return TrackingFrame(
        sequence=sequence,
        tracking_epoch=epoch,
        source_timestamp_ns=1_000_000_000 + sequence,
        bridge_monotonic_ns=2_000_000_000 + sequence,
        left_active=True,
        right_active=True,
        head_valid=True,
        left_scale=1.0,
        right_scale=1.0,
        head_pose=head,
        left_hand=hand,
        right_hand=hand.copy(),
    )


def with_tracking_crc(packet: bytearray) -> bytes:
    struct.pack_into("<I", packet, 12, crc32(memoryview(packet)[16:]))
    return bytes(packet)


def with_control_crc(packet: bytearray) -> bytes:
    struct.pack_into("<I", packet, 12, crc32(memoryview(packet)[16:]))
    return bytes(packet)


def test_fixed_packet_sizes_magics_and_keys():
    tracking_packet = encode_tracking(identity_tracking_frame())
    control_packet = encode_control(ControlFrame(1, 10, ControlCommand.START))

    assert len(tracking_packet) == TRACKING_PACKET_SIZE == 1540
    assert tracking_packet[:4] == b"SVT1"
    assert decode_tracking(tracking_packet).left_hand.shape == (26, 7)
    assert len(control_packet) == CONTROL_PACKET_SIZE == 40
    assert control_packet[:4] == b"SVC1"
    assert ARM_TARGET_PACKET_SIZE == 272
    assert (
        TRACKING_KEY,
        ARM_TARGETS_KEY,
        CONTROL_KEY,
        STATUS_BRIDGE_KEY,
        STATUS_IK_KEY,
        STATUS_VIEWER_KEY,
    ) == (
        "spd/vr/v1/tracking",
        "spd/vr/v1/arm_targets",
        "spd/vr/v1/control",
        "spd/vr/v1/status/bridge",
        "spd/vr/v1/status/ik",
        "spd/vr/v1/status/viewer",
    )


@pytest.mark.parametrize(
    ("offset", "replacement", "error"),
    [
        (0, b"BAD!", "wrong_magic"),
        (4, struct.pack("<H", 2), "wrong_version"),
        (6, struct.pack("<H", 0x8000), "non_zero_reserved"),
        (8, struct.pack("<I", 1539), "wrong_declared_size"),
    ],
)
def test_tracking_rejects_bad_header(offset, replacement, error):
    packet = bytearray(encode_tracking(identity_tracking_frame()))
    packet[offset : offset + len(replacement)] = replacement
    with pytest.raises(TrackingProtocolError, match=error):
        decode_tracking(packet)


def test_tracking_rejects_wrong_size_and_crc():
    packet = encode_tracking(identity_tracking_frame())
    with pytest.raises(TrackingProtocolError, match="wrong_size"):
        decode_tracking(packet[:-1])

    corrupt = bytearray(packet)
    corrupt[-1] ^= 1
    with pytest.raises(TrackingProtocolError, match="crc_mismatch"):
        decode_tracking(corrupt)


@pytest.mark.parametrize(
    ("mutate", "error"),
    [
        (lambda packet: struct.pack_into("<f", packet, 84, float("nan")), "non_finite_value"),
        (lambda packet: struct.pack_into("<f", packet, 48, float("inf")), "non_finite_value"),
        (
            lambda packet: struct.pack_into("<4f", packet, 96, 0.0, 0.0, 0.0, 0.0),
            "invalid_quaternion",
        ),
    ],
)
def test_tracking_decoder_rejects_non_finite_and_active_bad_quaternion(mutate, error):
    packet = bytearray(encode_tracking(identity_tracking_frame()))
    mutate(packet)
    with pytest.raises(TrackingProtocolError, match=error):
        decode_tracking(with_tracking_crc(packet))


def test_tracking_encoder_rejects_non_finite_and_active_bad_quaternion():
    frame = identity_tracking_frame()
    bad_hand = frame.left_hand.copy()
    bad_hand[0, 0] = np.nan
    with pytest.raises(TrackingProtocolError, match="non_finite_value"):
        encode_tracking(replace(frame, left_hand=bad_hand))

    bad_hand = frame.left_hand.copy()
    bad_hand[4, 3:7] = 0.0
    with pytest.raises(TrackingProtocolError, match="invalid_quaternion"):
        encode_tracking(replace(frame, left_hand=bad_hand))

    with pytest.raises(TrackingProtocolError, match="invalid_scale"):
        encode_tracking(replace(frame, right_scale=0.0))
    with pytest.raises(TrackingProtocolError, match="invalid_metadata"):
        encode_tracking(replace(frame, sequence=1.5))
    with pytest.raises(TrackingProtocolError, match="invalid_flag"):
        encode_tracking(replace(frame, left_active=1))


def test_tracking_stream_gate_rejects_epoch_sequence_and_timestamp_rollback():
    gate = TrackingStreamGate()
    first = identity_tracking_frame(sequence=4, epoch=2)
    assert gate.accept(first) is True

    with pytest.raises(TrackingProtocolError, match="out_of_order"):
        gate.accept(replace(first, sequence=4))
    with pytest.raises(TrackingProtocolError, match="timestamp_rollback"):
        gate.accept(replace(first, sequence=5, source_timestamp_ns=first.source_timestamp_ns - 1))
    with pytest.raises(TrackingProtocolError, match="epoch_rollback"):
        gate.accept(identity_tracking_frame(sequence=6, epoch=1))

    assert gate.accept(identity_tracking_frame(sequence=1, epoch=3)) is True


@pytest.mark.parametrize(
    ("offset", "replacement", "error"),
    [
        (0, b"BAD!", "wrong_magic"),
        (4, struct.pack("<H", 2), "wrong_version"),
        (6, struct.pack("<H", 99), "invalid_command"),
        (8, struct.pack("<I", 39), "wrong_declared_size"),
    ],
)
def test_control_rejects_bad_header(offset, replacement, error):
    packet = bytearray(encode_control(ControlFrame(1, 10, ControlCommand.START)))
    packet[offset : offset + len(replacement)] = replacement
    with pytest.raises(ControlProtocolError, match=error):
        decode_control(packet)


def test_control_rejects_size_crc_reserved_and_invalid_metadata():
    packet = encode_control(ControlFrame(1, 10, ControlCommand.START))
    with pytest.raises(ControlProtocolError, match="wrong_size"):
        decode_control(packet[:-1])

    corrupt = bytearray(packet)
    corrupt[20] ^= 1
    with pytest.raises(ControlProtocolError, match="crc_mismatch"):
        decode_control(corrupt)

    reserved = bytearray(packet)
    reserved[32] = 1
    with pytest.raises(ControlProtocolError, match="non_zero_reserved"):
        decode_control(with_control_crc(reserved))

    with pytest.raises(ControlProtocolError, match="invalid_metadata"):
        encode_control(ControlFrame(0, 10, ControlCommand.START))
    with pytest.raises(ControlProtocolError, match="invalid_metadata"):
        encode_control(ControlFrame(1, 0, ControlCommand.START))
    with pytest.raises(ControlProtocolError, match="invalid_metadata"):
        encode_control(ControlFrame(1.5, 10, ControlCommand.START))


def test_control_sequence_gate_makes_identical_duplicate_idempotent():
    gate = ControlSequenceGate()
    first = ControlFrame(7, 100, ControlCommand.PAUSE)
    assert gate.accept(first) is True
    assert gate.accept(first) is False

    with pytest.raises(ControlProtocolError, match="duplicate_conflict"):
        gate.accept(ControlFrame(7, 100, ControlCommand.RESUME))
    with pytest.raises(ControlProtocolError, match="sequence_rollback"):
        gate.accept(ControlFrame(6, 101, ControlCommand.RESUME))
    assert gate.accept(ControlFrame(8, 102, ControlCommand.RESUME)) is True
