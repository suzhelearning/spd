from dataclasses import replace
from pathlib import Path
import math
import struct

import pytest

from spd_vr.arm_target_protocol import crc32
from spd_vr.tracking_protocol import (
    HEAD_VALID,
    LEFT_ACTIVE,
    RIGHT_ACTIVE,
    TRACKING_PACKET_SIZE,
    TrackingFrame,
    TrackingProtocolError,
    TrackingStreamDecoder,
    decode_tracking_packet,
    encode_tracking_packet,
)
from spd_vr.zenoh_keys import (
    ARM_TARGETS_KEY,
    BRIDGE_STATUS_KEY,
    CONTROL_KEY,
    IK_STATUS_KEY,
    TRACKING_KEY,
    VIEWER_STATUS_KEY,
)

FIXTURE = Path(__file__).parent / "fixtures" / "tracking_v1.hex"


def fixture_frame() -> TrackingFrame:
    left_hand = tuple(
        (index / 8, -index / 16, index / 32, 0.0, 0.0, 0.0, 1.0)
        for index in range(26)
    )
    right_hand = tuple(
        (-index / 8, index / 16, -index / 32, 0.0, 0.0, 0.0, 1.0)
        for index in range(26)
    )
    return TrackingFrame(
        sequence=101,
        tracking_epoch=7,
        source_timestamp_ns=1_000_000_000,
        bridge_monotonic_ns=1_000_000_500,
        flags=LEFT_ACTIVE | RIGHT_ACTIVE | HEAD_VALID,
        left_scale=1.25,
        right_scale=0.75,
        head_pose=(1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0),
        left_hand=left_hand,
        right_hand=right_hand,
    )


def refresh_crc(packet: bytearray) -> None:
    struct.pack_into("<I", packet, 12, crc32(packet[16:]))


def test_matches_golden_vector_and_canonical_keys():
    packet = encode_tracking_packet(fixture_frame())
    assert TRACKING_PACKET_SIZE == 1540
    assert len(packet) == 1540
    assert packet[:4] == b"SVT1"
    assert packet == bytes.fromhex(FIXTURE.read_text().strip())
    decoded = decode_tracking_packet(packet)
    assert decoded == fixture_frame()
    assert decoded.left_hand[1][0] == 0.125
    assert (
        TRACKING_KEY,
        ARM_TARGETS_KEY,
        CONTROL_KEY,
        BRIDGE_STATUS_KEY,
        IK_STATUS_KEY,
        VIEWER_STATUS_KEY,
    ) == (
        "spd/vr/v1/tracking",
        "spd/vr/v1/arm_targets",
        "spd/vr/v1/control",
        "spd/vr/v1/status/bridge",
        "spd/vr/v1/status/ik",
        "spd/vr/v1/status/viewer",
    )


def test_rejects_structural_corruption_fail_closed():
    packet = bytearray(encode_tracking_packet(fixture_frame()))
    packet[0] = ord("X")
    with pytest.raises(TrackingProtocolError, match="wrong_magic"):
        decode_tracking_packet(packet)

    packet = bytearray(encode_tracking_packet(fixture_frame()))
    struct.pack_into("<H", packet, 4, 2)
    with pytest.raises(TrackingProtocolError, match="wrong_version"):
        decode_tracking_packet(packet)

    packet = bytearray(encode_tracking_packet(fixture_frame()))
    struct.pack_into("<I", packet, 8, 1)
    with pytest.raises(TrackingProtocolError, match="wrong_declared_size"):
        decode_tracking_packet(packet)

    packet = bytearray(encode_tracking_packet(fixture_frame()))
    struct.pack_into("<H", packet, 6, 8)
    with pytest.raises(TrackingProtocolError, match="unknown_flags"):
        decode_tracking_packet(packet)

    packet = bytearray(encode_tracking_packet(fixture_frame()))
    packet[100] ^= 1
    with pytest.raises(TrackingProtocolError, match="crc_mismatch"):
        decode_tracking_packet(packet)

    with pytest.raises(TrackingProtocolError, match="wrong_size"):
        decode_tracking_packet(packet[:-1])


def test_rejects_nan_inf_nonpositive_metadata_scale_and_bad_quaternion():
    packet = bytearray(encode_tracking_packet(fixture_frame()))
    struct.pack_into("<f", packet, 84, math.nan)
    refresh_crc(packet)
    with pytest.raises(TrackingProtocolError, match="non_finite_value"):
        decode_tracking_packet(packet)

    packet = bytearray(encode_tracking_packet(fixture_frame()))
    struct.pack_into("<f", packet, 52, math.inf)
    refresh_crc(packet)
    with pytest.raises(TrackingProtocolError, match="invalid_scale"):
        decode_tracking_packet(packet)

    for offset in (24, 32, 40):
        packet = bytearray(encode_tracking_packet(fixture_frame()))
        struct.pack_into("<q", packet, offset, 0)
        refresh_crc(packet)
        with pytest.raises(TrackingProtocolError, match="invalid_metadata"):
            decode_tracking_packet(packet)

    packet = bytearray(encode_tracking_packet(fixture_frame()))
    struct.pack_into("<f", packet, 48, 0.0)
    refresh_crc(packet)
    with pytest.raises(TrackingProtocolError, match="invalid_scale"):
        decode_tracking_packet(packet)

    packet = bytearray(encode_tracking_packet(fixture_frame()))
    struct.pack_into("<f", packet, 84 + 6 * 4, 1.01)
    refresh_crc(packet)
    with pytest.raises(TrackingProtocolError, match="invalid_quaternion"):
        decode_tracking_packet(packet)


def test_invalid_head_is_zero_and_inactive_placeholders_are_permitted():
    frame = replace(
        fixture_frame(),
        flags=LEFT_ACTIVE | RIGHT_ACTIVE,
        head_pose=(0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    )
    with pytest.raises(TrackingProtocolError, match="invalid_head_pose"):
        encode_tracking_packet(frame)

    frame = replace(
        frame,
        flags=LEFT_ACTIVE,
        head_pose=(0.0,) * 7,
        right_hand=((0.0,) * 7,) * 26,
    )
    decoded = decode_tracking_packet(encode_tracking_packet(frame))
    assert decoded.head_pose == (0.0,) * 7
    assert decoded.right_hand == ((0.0,) * 7,) * 26


def test_normalizes_accepted_quaternion_once():
    hand = list(fixture_frame().left_hand)
    hand[1] = (*hand[1][:-1], 1.0005)
    frame = replace(fixture_frame(), left_hand=tuple(hand))
    decoded = decode_tracking_packet(encode_tracking_packet(frame))
    assert math.sqrt(sum(value * value for value in decoded.left_hand[1][3:])) == pytest.approx(1.0)


def test_stream_rejects_rollback_and_resets_on_epoch_increase():
    decoder = TrackingStreamDecoder()
    packet = encode_tracking_packet(fixture_frame())
    assert decoder.decode(packet) == fixture_frame()
    with pytest.raises(TrackingProtocolError, match="out_of_order"):
        decoder.decode(packet)

    timestamp_rollback = replace(
        fixture_frame(), sequence=102, source_timestamp_ns=999_999_999
    )
    with pytest.raises(TrackingProtocolError, match="timestamp_rollback"):
        decoder.decode(encode_tracking_packet(timestamp_rollback))

    new_epoch = replace(
        fixture_frame(), tracking_epoch=8, sequence=1, source_timestamp_ns=1
    )
    assert decoder.decode(encode_tracking_packet(new_epoch)) == new_epoch

    old_epoch = replace(
        fixture_frame(), tracking_epoch=7, sequence=1000, source_timestamp_ns=2_000_000_000
    )
    with pytest.raises(TrackingProtocolError, match="epoch_rollback"):
        decoder.decode(encode_tracking_packet(old_epoch))
