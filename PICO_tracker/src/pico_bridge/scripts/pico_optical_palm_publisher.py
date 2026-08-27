#!/usr/bin/env python3
"""Publish OpenXR palm joint 0 without controller calibration artifacts."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import PoseStamped
from pico_bridge.msg import PicoHands
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class OpticalPalmPublisher(Node):
    """Project one atomic ``PicoHands`` message into two palm poses."""

    def __init__(self) -> None:
        super().__init__("pico_optical_palm_publisher")
        self._input_topic = self.declare_parameter("input_topic", "/pico/hands").value
        self._left_topic = self.declare_parameter("left_topic", "/pico/palm_left").value
        self._right_topic = self.declare_parameter("right_topic", "/pico/palm_right").value
        self._left_pub = self.create_publisher(
            PoseStamped, self._left_topic, qos_profile_sensor_data
        )
        self._right_pub = self.create_publisher(
            PoseStamped, self._right_topic, qos_profile_sensor_data
        )
        self._subscription = self.create_subscription(
            PicoHands, self._input_topic, self._on_hands, qos_profile_sensor_data
        )
        self.get_logger().info(
            f"optical palm source {self._input_topic} -> "
            f"{self._left_topic}, {self._right_topic}; "
            "joint 0, no T_controller_palm artifact"
        )

    @staticmethod
    def _pose(message: PicoHands, side: str) -> PoseStamped:
        output = PoseStamped()
        output.header = message.header
        output.header.frame_id = "pico"
        output.pose = message.left_joints[0] if side == "left" else message.right_joints[0]
        return output

    def _on_hands(self, message: PicoHands) -> None:
        # The timestamp and stream epoch are retained by the source message;
        # inactive hands still publish a frame, while downstream retargeters
        # use the active flags from the same PicoHands message for HOLD.
        self._left_pub.publish(self._pose(message, "left"))
        self._right_pub.publish(self._pose(message, "right"))


def main() -> None:
    rclpy.init()
    node = OpticalPalmPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
