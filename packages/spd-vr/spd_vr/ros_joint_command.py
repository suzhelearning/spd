"""ROS 2 JointCommand contract and transport helpers.

The ROS message definition lives in ``tianji_spd_interfaces``.  This module
keeps all application-level validation independent from rclpy so it can be
unit-tested in the default Python environment.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Any, Iterable, Sequence

import numpy as np

SCHEMA_VERSION = 1
ROBOT_CONFIG = "tianji_wuji2_v1"
TOPIC = "/spd/tianji_wuji2/v1/joint_command"
QOS_DEPTH = 1
MAX_AGE_NS = 100_000_000
MAX_FUTURE_NS = 5_000_000

ARM_NAMES = tuple(
    [f"Joint{index}_L" for index in range(1, 8)]
    + [f"Joint{index}_R" for index in range(1, 8)]
)
_HAND_BASE_NAMES = (
    "thumb_cmc_flex", "thumb_cmc_abd", "thumb_mcp", "thumb_ip",
    "index_finger_mcp_flex", "index_finger_mcp_abd", "index_finger_pip", "index_finger_dip",
    "middle_finger_mcp_flex", "middle_finger_mcp_abd", "middle_finger_pip", "middle_finger_dip",
    "ring_finger_mcp_flex", "ring_finger_mcp_abd", "ring_finger_pip", "ring_finger_dip",
    "pinky_mcp_flex", "pinky_mcp_abd", "pinky_pip", "pinky_dip",
)
JOINT_NAMES = ARM_NAMES + tuple(
    f"{side}_{name}" for side in ("l", "r") for name in _HAND_BASE_NAMES
)
JOINT_NAME_TUPLE = tuple(JOINT_NAMES)
ARMS_READY = 1
RIGHT_HAND_READY = 2
LEFT_HAND_READY = 4
VALID_READY_MASK = ARMS_READY | RIGHT_HAND_READY | LEFT_HAND_READY


class JointCommandError(ValueError):
    """Raised when a ROS command violates the application contract."""


@dataclass(frozen=True, slots=True)
class JointCommandSnapshot:
    schema_version: int
    robot_config: str
    session_id: str
    sequence: int
    stamp_ns: int
    ready_mask: int
    joint_names: tuple[str, ...]
    position_rad: tuple[float, ...]

    def validate(self, *, now_ns: int | None = None, previous_sequence: int | None = None,
                 previous_stamp_ns: int | None = None) -> "JointCommandSnapshot":
        if int(self.schema_version) != SCHEMA_VERSION:
            raise JointCommandError(f"unsupported schema_version: {self.schema_version}")
        if str(self.robot_config) != ROBOT_CONFIG:
            raise JointCommandError(f"unsupported robot_config: {self.robot_config}")
        if not self.session_id:
            raise JointCommandError("session_id must be non-empty")
        if isinstance(self.sequence, bool) or int(self.sequence) <= 0:
            raise JointCommandError("sequence must be a positive integer")
        if isinstance(self.stamp_ns, bool) or int(self.stamp_ns) <= 0:
            raise JointCommandError("stamp_ns must be a positive integer")
        if int(self.ready_mask) & ~VALID_READY_MASK:
            raise JointCommandError("ready_mask contains reserved bits")
        names = tuple(self.joint_names)
        if names != JOINT_NAME_TUPLE:
            raise JointCommandError("joint_names do not match the canonical 54-DoF order")
        values = tuple(float(value) for value in self.position_rad)
        if len(values) != 54 or not all(math.isfinite(value) for value in values):
            raise JointCommandError("position_rad must contain 54 finite values")
        if previous_sequence is not None and int(self.sequence) <= int(previous_sequence):
            raise JointCommandError("sequence is not strictly increasing")
        if previous_stamp_ns is not None and int(self.stamp_ns) < int(previous_stamp_ns):
            raise JointCommandError("stamp_ns rolled back")
        if now_ns is not None:
            age = int(now_ns) - int(self.stamp_ns)
            if age > MAX_AGE_NS:
                raise JointCommandError("command is older than 100 ms")
            if age < -MAX_FUTURE_NS:
                raise JointCommandError("command is more than 5 ms in the future")
        return JointCommandSnapshot(
            int(self.schema_version), str(self.robot_config), str(self.session_id),
            int(self.sequence), int(self.stamp_ns), int(self.ready_mask),
            names, values,
        )

    @classmethod
    def from_values(cls, *, session_id: str, sequence: int, ready_mask: int,
                    position_rad: Iterable[float], stamp_ns: int | None = None,
                    robot_config: str = ROBOT_CONFIG) -> "JointCommandSnapshot":
        snapshot = cls(
            SCHEMA_VERSION, robot_config, str(session_id), int(sequence),
            int(time.time_ns() if stamp_ns is None else stamp_ns), int(ready_mask),
            JOINT_NAME_TUPLE, tuple(float(value) for value in position_rad),
        )
        return snapshot.validate()


def ros_time_to_ns(stamp: Any) -> int:
    """Convert builtin_interfaces/Time to a host UTC nanosecond integer."""
    try:
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    except (AttributeError, TypeError, ValueError) as exc:
        raise JointCommandError("invalid ROS timestamp") from exc


def ns_to_ros_time(stamp_ns: int) -> Any:
    from builtin_interfaces.msg import Time
    value = int(stamp_ns)
    if value <= 0:
        raise JointCommandError("stamp_ns must be positive")
    return Time(sec=value // 1_000_000_000, nanosec=value % 1_000_000_000)


def snapshot_from_ros(message: Any, *, now_ns: int | None = None,
                      previous_sequence: int | None = None,
                      previous_stamp_ns: int | None = None) -> JointCommandSnapshot:
    snapshot = JointCommandSnapshot(
        schema_version=int(message.schema_version),
        robot_config=str(message.robot_config),
        session_id=str(message.session_id),
        sequence=int(message.sequence),
        stamp_ns=ros_time_to_ns(message.stamp),
        ready_mask=int(message.ready_mask),
        joint_names=tuple(str(value) for value in message.joint_names),
        position_rad=tuple(float(value) for value in message.position_rad),
    )
    return snapshot.validate(
        now_ns=int(time.time_ns() if now_ns is None else now_ns),
        previous_sequence=previous_sequence,
        previous_stamp_ns=previous_stamp_ns,
    )


def message_from_snapshot(snapshot: JointCommandSnapshot) -> Any:
    from tianji_spd_interfaces.msg import JointCommand
    value = snapshot.validate()
    message = JointCommand()
    message.schema_version = value.schema_version
    message.robot_config = value.robot_config
    message.session_id = value.session_id
    message.sequence = value.sequence
    message.stamp = ns_to_ros_time(value.stamp_ns)
    message.ready_mask = value.ready_mask
    message.joint_names = list(value.joint_names)
    message.position_rad = list(value.position_rad)
    return message


def best_effort_qos() -> Any:
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=QOS_DEPTH,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


__all__ = [
    "ARM_NAMES", "ARMS_READY", "JOINT_NAMES", "JOINT_NAME_TUPLE", "LEFT_HAND_READY",
    "JointCommandError", "JointCommandSnapshot", "MAX_AGE_NS", "MAX_FUTURE_NS",
    "QOS_DEPTH", "RIGHT_HAND_READY", "ROBOT_CONFIG", "SCHEMA_VERSION", "TOPIC",
    "VALID_READY_MASK", "best_effort_qos", "message_from_snapshot", "ns_to_ros_time",
    "ros_time_to_ns", "snapshot_from_ros",
]
