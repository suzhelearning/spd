"""Canonical Python codecs and key names for SPD VR teleoperation."""

from ..arm_target_protocol import (
    ArmTargetFrame,
    ArmTargetHoldReason,
    ArmTargetProtocolError,
    ArmTargetStreamDecoder,
    decode_packet as decode_arm_target,
    encode_packet as encode_arm_target,
)
from ..arm_target_protocol import PACKET_SIZE as ARM_TARGET_PACKET_SIZE
from .control import (
    CONTROL_PACKET_SIZE,
    ControlCommand,
    ControlFrame,
    ControlProtocolError,
    ControlSequenceGate,
    decode_control,
    encode_control,
)
from .crc import crc32
from .keys import (
    ARM_TARGETS_KEY,
    CONTROL_KEY,
    STATUS_BRIDGE_KEY,
    STATUS_IK_KEY,
    STATUS_VIEWER_KEY,
    TRACKING_KEY,
)
from .tracking import (
    TRACKING_PACKET_SIZE,
    TrackingFrame,
    TrackingProtocolError,
    TrackingStreamGate,
    decode_tracking,
    encode_tracking,
)

__all__ = [
    "ARM_TARGETS_KEY",
    "ARM_TARGET_PACKET_SIZE",
    "CONTROL_KEY",
    "CONTROL_PACKET_SIZE",
    "STATUS_BRIDGE_KEY",
    "STATUS_IK_KEY",
    "STATUS_VIEWER_KEY",
    "TRACKING_KEY",
    "TRACKING_PACKET_SIZE",
    "ArmTargetFrame",
    "ArmTargetHoldReason",
    "ArmTargetProtocolError",
    "ArmTargetStreamDecoder",
    "ControlCommand",
    "ControlFrame",
    "ControlProtocolError",
    "ControlSequenceGate",
    "TrackingFrame",
    "TrackingProtocolError",
    "TrackingStreamGate",
    "crc32",
    "decode_arm_target",
    "decode_control",
    "decode_tracking",
    "encode_arm_target",
    "encode_control",
    "encode_tracking",
]
