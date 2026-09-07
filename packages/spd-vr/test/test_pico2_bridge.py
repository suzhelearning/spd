import struct

import numpy as np
import pytest

from pico_hand_tracking import PAYLOAD_BYTES, POSE, parse_hand_frame
from spd_vr.pico2_bridge import Pico2BridgeCore
from spd_vr.wire import ControlCommand, ControlFrame, decode_tracking


def hand_payload(*, timestamp: int = 10, left_valid: bool = True, right_valid: bool = False) -> bytes:
    flags = 0x01 | (0x02 if left_valid else 0) | (0x04 if right_valid else 0)
    payload = bytearray(PAYLOAD_BYTES)
    struct.pack_into("<BBBB", payload, 0, 1, flags, 26, 0)
    offset = 4
    struct.pack_into("<7f", payload, offset, 0.1, 0.2, 1.6, 0.0, 0.0, 0.0, 1.0)
    offset += POSE.size
    for hand_index, active in enumerate((left_valid, right_valid)):
        struct.pack_into("<BBBB", payload, offset, int(active), 0, 0, 0)
        offset += 4
        struct.pack_into("<7f", payload, offset, 0.2 + hand_index, 0.3, 1.0, 0.0, 0.0, 0.0, 1.0)
        offset += POSE.size
        for joint_index in range(26):
            x = 0.01 * (joint_index + 1)
            struct.pack_into("<BBBB", payload, offset, int(active), 0, 0, 0)
            offset += 4
            struct.pack_into("<7f", payload, offset, x, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
            offset += POSE.size
            struct.pack_into("<f", payload, offset, 0.01)
            offset += 4
    assert offset == len(payload)
    return bytes(payload)


def test_pico2_parser_decodes_atomic_head_and_hands():
    frame = parse_hand_frame(10, hand_payload())

    assert frame.head_valid is True
    assert frame.left.valid is True
    assert frame.right.valid is False
    assert frame.left.joints[4].name == "thumb_distal"
    assert frame.left.joints[4].position == pytest.approx((0.05, 0.0, 0.0))


def test_pico2_bridge_publishes_canonical_tracking_packet():
    core = Pico2BridgeCore(clock_ns=lambda: 123)
    core.reset_stream()

    packet = core.accept_frame(parse_hand_frame(10, hand_payload()))
    assert packet is not None
    tracking = decode_tracking(packet)

    assert tracking.tracking_epoch == 1
    assert tracking.sequence == 1
    assert tracking.source_timestamp_ns == 10_000_000
    assert tracking.bridge_monotonic_ns == 123
    assert tracking.head_valid is True
    assert tracking.left_active is True
    assert tracking.right_active is False
    assert tracking.left_hand[4, 0] == pytest.approx(0.05)
    assert np.all(tracking.right_hand[:, 3:6] == 0.0)
    assert np.all(tracking.right_hand[:, 6] == 1.0)


def test_pico2_bridge_drops_duplicate_and_resets_epoch_on_timestamp_rollback():
    core = Pico2BridgeCore(clock_ns=lambda: 456)
    core.reset_stream()
    first = core.accept_frame(parse_hand_frame(20, hand_payload(timestamp=20)))
    assert first is not None
    assert core.accept_frame(parse_hand_frame(20, hand_payload(timestamp=20))) is None

    second = core.accept_frame(parse_hand_frame(10, hand_payload(timestamp=10)))
    assert second is not None
    tracking = decode_tracking(second)
    assert tracking.tracking_epoch == 2
    assert tracking.sequence == 2


def test_pico2_bridge_holds_hand_with_invalid_joint():
    payload = bytearray(hand_payload())
    joint_valid_offset = 4 + POSE.size + 4 + POSE.size
    payload[joint_valid_offset] = 0
    core = Pico2BridgeCore()
    core.reset_stream()

    packet = core.accept_frame(parse_hand_frame(10, payload))
    assert packet is not None
    assert decode_tracking(packet).left_active is False


def test_control_acknowledgement_survives_tracking_and_rejects_stale_shutdown():
    core = Pico2BridgeCore()
    start = ControlFrame(50, 100, ControlCommand.START)
    assert core.accept_control(start)
    core.accept_frame(parse_hand_frame(10, hand_payload()))
    status = core.status(connected=True, received=1, dropped=0, last_error="")
    assert status["sequence"] == 50
    assert status["tracking_sequence"] == 1
    assert not core.accept_control(start)
    assert not core.accept_control(ControlFrame(49, 101, ControlCommand.SHUTDOWN))
    assert not core.shutdown_requested
    assert core.accept_control(ControlFrame(51, 102, ControlCommand.SHUTDOWN))
    assert core.shutdown_requested
