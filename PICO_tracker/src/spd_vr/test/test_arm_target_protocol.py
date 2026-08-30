from pathlib import Path

import pytest

from spd_vr.arm_target_protocol import (
    ArmTargetFrame,
    ArmTargetHoldReason,
    ArmTargetProtocolError,
    ArmTargetStreamDecoder,
    decode_packet,
    encode_packet,
)

FIXTURE = Path(__file__).parent / "fixtures" / "arm_target_v2.hex"


def fixture_frame() -> ArmTargetFrame:
    return ArmTargetFrame(
        sequence=17,
        tracking_epoch=9,
        source_timestamp_ns=1_000_000_000,
        control_timestamp_ns=1_000_001_000,
        valid_mask=3,
        left_hold_reason=ArmTargetHoldReason.NONE,
        right_hold_reason=ArmTargetHoldReason.NONE,
        left_q=tuple(0.1 * index for index in range(7)),
        right_q=tuple(-0.2 * index for index in range(7)),
        left_qdot=tuple(0.3 * index for index in range(7)),
        right_qdot=tuple(-0.4 * index for index in range(7)),
    )


def test_python_matches_shared_fixture():
    packet = bytes.fromhex(FIXTURE.read_text().strip())
    assert len(packet) == 272
    assert encode_packet(fixture_frame()) == packet
    assert decode_packet(packet) == fixture_frame()


def test_corrupt_crc_nan_and_sequence_are_rejected_or_held():
    packet = bytearray(encode_packet(fixture_frame()))
    packet[100] ^= 1
    with pytest.raises(ArmTargetProtocolError, match="crc_mismatch"):
        decode_packet(packet)

    nan_packet = bytearray(encode_packet(fixture_frame()))
    nan_packet[44:52] = bytes.fromhex("000000000000f87f")
    # Keep the CRC intentionally stale; the packet is rejected before values.
    with pytest.raises(ArmTargetProtocolError):
        decode_packet(nan_packet)

    decoder = ArmTargetStreamDecoder(max_age_ns=10)
    fresh = decoder.decode(encode_packet(fixture_frame()), now_ns=1_000_001_005)
    assert fresh.valid_mask == 3
    stale = decoder.decode(
        encode_packet(
            ArmTargetFrame(
                **{
                    **fixture_frame().__dict__,
                    "sequence": 18,
                    "control_timestamp_ns": 1_000_000_000,
                }
            )
        ),
        now_ns=1_000_001_100,
    )
    assert stale.valid_mask == 0
    assert stale.left_hold_reason is ArmTargetHoldReason.INPUT_STALE
    assert stale.right_hold_reason is ArmTargetHoldReason.INPUT_STALE

    with pytest.raises(ArmTargetProtocolError, match="out_of_order"):
        decoder.decode(encode_packet(fixture_frame()), now_ns=1_000_001_005)


def test_partial_hold_roundtrip():
    frame = ArmTargetFrame(
        **{
            **fixture_frame().__dict__,
            "valid_mask": 1,
            "left_hold_reason": ArmTargetHoldReason.NONE,
            "right_hold_reason": ArmTargetHoldReason.INPUT_STALE,
        }
    )
    assert decode_packet(encode_packet(frame)) == frame


def test_left_solver_failure_right_valid_roundtrip():
    frame = ArmTargetFrame(
        **{
            **fixture_frame().__dict__,
            "valid_mask": 2,
            "left_hold_reason": ArmTargetHoldReason.SOLVER_FAILURE,
            "right_hold_reason": ArmTargetHoldReason.NONE,
        }
    )
    decoded = decode_packet(encode_packet(frame))
    assert decoded.left_hold_reason is ArmTargetHoldReason.SOLVER_FAILURE
    assert decoded.right_hold_reason is ArmTargetHoldReason.NONE


def test_rejects_wrong_version_and_reserved_byte():
    packet = bytearray(encode_packet(fixture_frame()))
    packet[4] = 1
    with pytest.raises(ArmTargetProtocolError, match="wrong_version"):
        decode_packet(packet)
    packet = bytearray(encode_packet(fixture_frame()))
    packet[43] = 1
    with pytest.raises(ArmTargetProtocolError, match="non_zero_reserved"):
        decode_packet(packet)

def test_paused_both_sides_roundtrip():
    frame = ArmTargetFrame(
        **{
            **fixture_frame().__dict__,
            "valid_mask": 0,
            "left_hold_reason": ArmTargetHoldReason.PAUSED,
            "right_hold_reason": ArmTargetHoldReason.PAUSED,
        }
    )
    assert decode_packet(encode_packet(frame)) == frame
