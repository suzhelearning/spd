import struct

import numpy as np
import pytest

from spd_vr.pico_frames import (
    FRAME_TYPE_HAND_LEFT,
    FRAME_TYPE_HAND_RIGHT,
    FRAME_TYPE_HEAD,
    FRAME_TYPE_WORLD_RESET,
    HAND_PAYLOAD_BYTES,
    MAX_INNER_FRAME_BYTES,
    HandPairer,
    PicoFrame,
    PicoFrameError,
    PicoStreamDecoder,
    decode_hand,
    PicoHand,
    PicoPose,
    decode_head_pose,
    decode_world_reset_yaw,
)

HEADER = struct.Struct("<BBqI")


def frame_bytes(frame_type: int, timestamp_ms: int, payload: bytes) -> bytes:
    return HEADER.pack(0xAB, frame_type, timestamp_ms, len(payload)) + payload


def hand_payload(*, active: bool = True, scale: float = 1.0, x: float = 0.0) -> bytes:
    joints = np.zeros((26, 7), dtype="<f4")
    joints[:, 0] = x
    joints[:, 6] = 1.0
    return bytes([int(active)]) + struct.pack("<f", scale) + joints.tobytes()


def test_stream_decoder_preserves_half_frame_until_complete():
    encoded = frame_bytes(FRAME_TYPE_HAND_LEFT, 123, hand_payload())
    decoder = PicoStreamDecoder()

    assert decoder.feed(encoded[:9]) == []
    frames = decoder.feed(encoded[9:])

    assert frames == [PicoFrame(FRAME_TYPE_HAND_LEFT, 123, hand_payload())]
    assert len(encoded) == MAX_INNER_FRAME_BYTES == 747


def test_stream_decoder_returns_multiple_frames_from_one_feed():
    left = frame_bytes(FRAME_TYPE_HAND_LEFT, 10, hand_payload(x=1.0))
    right = frame_bytes(FRAME_TYPE_HAND_RIGHT, 10, hand_payload(x=2.0))

    frames = PicoStreamDecoder().feed(left + right)

    assert [frame.frame_type for frame in frames] == [
        FRAME_TYPE_HAND_LEFT,
        FRAME_TYPE_HAND_RIGHT,
    ]
    assert [frame.timestamp_ms for frame in frames] == [10, 10]


@pytest.mark.parametrize("payload_size", [HAND_PAYLOAD_BYTES - 1, HAND_PAYLOAD_BYTES + 1])
def test_hand_payload_length_is_exactly_733_bytes(payload_size):
    frame = PicoFrame(FRAME_TYPE_HAND_LEFT, 1, bytes(payload_size))
    with pytest.raises(PicoFrameError, match="wrong_hand_payload_size"):
        decode_hand(frame)


def test_bad_magic_and_oversize_clear_decoder_buffer():
    decoder = PicoStreamDecoder()
    valid = frame_bytes(FRAME_TYPE_HEAD, 2, struct.pack("<7f", 0, 0, 0, 0, 0, 0, 1))

    with pytest.raises(PicoFrameError, match="bad_magic"):
        decoder.feed(b"\x00" + valid[1:])
    assert decoder.feed(valid) == [
        PicoFrame(FRAME_TYPE_HEAD, 2, struct.pack("<7f", 0, 0, 0, 0, 0, 0, 1))
    ]

    with pytest.raises(PicoFrameError, match="payload_too_large"):
        decoder.feed(HEADER.pack(0xAB, FRAME_TYPE_HAND_LEFT, 3, HAND_PAYLOAD_BYTES + 1))
    assert decoder.feed(valid) == [
        PicoFrame(FRAME_TYPE_HEAD, 2, struct.pack("<7f", 0, 0, 0, 0, 0, 0, 1))
    ]


def test_typed_hand_head_and_world_reset_decoding():
    hand = decode_hand(PicoFrame(FRAME_TYPE_HAND_LEFT, 10, hand_payload(scale=1.25, x=0.5)))
    assert hand.active is True
    assert hand.scale == pytest.approx(1.25)
    assert hand.joints.shape == (26, 7)
    assert hand.joints[0, 0] == pytest.approx(0.5)

    pose = decode_head_pose(
        PicoFrame(
            FRAME_TYPE_HEAD,
            11,
            struct.pack("<7f", 1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0),
        )
    )
    np.testing.assert_array_equal(pose.position, [1.0, 2.0, 3.0])
    np.testing.assert_array_equal(pose.quaternion_xyzw, [0.0, 0.0, 0.0, 1.0])

    assert decode_world_reset_yaw(
        PicoFrame(FRAME_TYPE_WORLD_RESET, 12, struct.pack("<f", 0.75))
    ) == pytest.approx(0.75)


@pytest.mark.parametrize(
    ("frame", "error"),
    [
        (
            PicoFrame(FRAME_TYPE_HAND_LEFT, 1, bytes([2]) + bytes(HAND_PAYLOAD_BYTES - 1)),
            "invalid_active",
        ),
        (
            PicoFrame(
                FRAME_TYPE_HAND_LEFT,
                1,
                bytes([1]) + struct.pack("<f", float("inf")) + bytes(26 * 7 * 4),
            ),
            "invalid_scale",
        ),
        (
            PicoFrame(
                FRAME_TYPE_HEAD,
                1,
                struct.pack("<7f", 0, 0, 0, 0, 0, 0, float("nan")),
            ),
            "non_finite_value",
        ),
        (
            PicoFrame(FRAME_TYPE_WORLD_RESET, 1, struct.pack("<f", float("inf"))),
            "non_finite_value",
        ),
    ],
)
def test_typed_decoders_reject_invalid_semantics(frame, error):
    decoder = {
        FRAME_TYPE_HAND_LEFT: decode_hand,
        FRAME_TYPE_HEAD: decode_head_pose,
        FRAME_TYPE_WORLD_RESET: decode_world_reset_yaw,
    }[frame.frame_type]
    with pytest.raises(PicoFrameError, match=error):
        decoder(frame)


def test_active_hand_rejects_non_unit_quaternions_but_inactive_hand_allows_them():
    joints = np.zeros((26, 7), dtype="<f4")
    invalid_active = PicoFrame(
        FRAME_TYPE_HAND_LEFT,
        1,
        bytes([1]) + struct.pack("<f", 1.0) + joints.tobytes(),
    )
    with pytest.raises(PicoFrameError, match="invalid_quaternion"):
        decode_hand(invalid_active)

    inactive = PicoFrame(
        FRAME_TYPE_HAND_LEFT,
        1,
        bytes([0]) + struct.pack("<f", 1.0) + joints.tobytes(),
    )
    assert decode_hand(inactive).active is False


def test_pico_typed_dataclasses_own_read_only_array_copies():
    position = np.array([1.0, 2.0, 3.0], dtype=np.float32)
    quaternion = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)
    joints = np.zeros((26, 7), dtype=np.float32)
    joints[:, 6] = 1.0
    pose = PicoPose(position, quaternion)
    hand = PicoHand(True, 1.0, joints)

    position[0] = 99.0
    quaternion[3] = 0.0
    joints[0, 0] = 99.0
    assert pose.position[0] == pytest.approx(1.0)
    assert pose.quaternion_xyzw[3] == pytest.approx(1.0)
    assert hand.joints[0, 0] == pytest.approx(0.0)
    with pytest.raises(ValueError, match="read-only"):
        pose.position[0] = 5.0
    with pytest.raises(ValueError, match="read-only"):
        pose.quaternion_xyzw[0] = 5.0
    with pytest.raises(ValueError, match="read-only"):
        hand.joints[0, 0] = 5.0


def test_pairer_uses_same_timestamp_and_latest_replacement():
    pairer = HandPairer()
    old_left = PicoFrame(FRAME_TYPE_HAND_LEFT, 20, hand_payload(x=1.0))
    replacement_left = PicoFrame(FRAME_TYPE_HAND_LEFT, 20, hand_payload(x=2.0))
    right = PicoFrame(FRAME_TYPE_HAND_RIGHT, 20, hand_payload(x=3.0))

    assert pairer.accept(old_left, epoch=4) is None
    assert pairer.accept(replacement_left, epoch=4) is None
    pair = pairer.accept(right, epoch=4)

    assert pair is not None
    assert pair.timestamp_ms == 20
    assert pair.epoch == 4
    assert pair.left.joints[0, 0] == pytest.approx(2.0)
    assert pair.right.joints[0, 0] == pytest.approx(3.0)


def test_new_timestamp_replaces_old_incomplete_pair():
    pairer = HandPairer()
    assert pairer.accept(
        PicoFrame(FRAME_TYPE_HAND_LEFT, 20, hand_payload(x=1.0)), epoch=4
    ) is None
    assert pairer.accept(
        PicoFrame(FRAME_TYPE_HAND_RIGHT, 21, hand_payload(x=2.0)), epoch=4
    ) is None

    pair = pairer.accept(
        PicoFrame(FRAME_TYPE_HAND_LEFT, 21, hand_payload(x=3.0)), epoch=4
    )
    assert pair is not None
    assert pair.timestamp_ms == 21
    assert pair.left.joints[0, 0] == pytest.approx(3.0)


def test_epoch_change_and_world_reset_clear_incomplete_pair():
    pairer = HandPairer()
    assert pairer.accept(
        PicoFrame(FRAME_TYPE_HAND_LEFT, 30, hand_payload(x=1.0)), epoch=1
    ) is None
    assert pairer.accept(
        PicoFrame(FRAME_TYPE_HAND_RIGHT, 30, hand_payload(x=2.0)), epoch=2
    ) is None

    reset = PicoFrame(FRAME_TYPE_WORLD_RESET, 31, struct.pack("<f", 0.25))
    assert pairer.accept(reset, epoch=2) is None
    assert pairer.accept(
        PicoFrame(FRAME_TYPE_HAND_LEFT, 30, hand_payload(x=3.0)), epoch=3
    ) is None
    pair = pairer.accept(
        PicoFrame(FRAME_TYPE_HAND_RIGHT, 30, hand_payload(x=4.0)), epoch=3
    )
    assert pair is not None
    assert pair.epoch == 3


def test_pairer_rejects_epoch_rollback_and_non_integral_epoch_without_resetting():
    pairer = HandPairer()
    left = PicoFrame(FRAME_TYPE_HAND_LEFT, 40, hand_payload(x=1.0))
    right = PicoFrame(FRAME_TYPE_HAND_RIGHT, 40, hand_payload(x=2.0))
    assert pairer.accept(left, epoch=2) is None

    with pytest.raises(PicoFrameError, match="epoch_rollback"):
        pairer.accept(right, epoch=1)
    with pytest.raises(PicoFrameError, match="invalid_epoch"):
        pairer.accept(right, epoch=2.5)

    pair = pairer.accept(right, epoch=2)
    assert pair is not None
    assert pair.epoch == 2


def test_pairer_ignores_generic_frames():
    pairer = HandPairer()
    assert pairer.accept(PicoFrame(0x42, 1, b"opaque"), epoch=1) is None
