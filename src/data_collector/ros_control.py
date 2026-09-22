"""ROS Trigger transport for a physics-thread-owned CollectionSession."""
from __future__ import annotations

import json
import time
from typing import Any


SERVICE_NAMES = {name: f"/spd/collection/{name}" for name in ("start", "save", "discard")}
STATUS_TOPIC = "/spd/collection/status"


class CollectionRosControl:
    """Callbacks run in the viewer's spin_once, never on a writer thread."""

    def __init__(self, node: Any, session: Any) -> None:
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String
        from std_srvs.srv import Trigger

        self.session = session
        self.node = node
        self._message_type = String
        self._last_publish_ns = 0
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = node.create_publisher(String, STATUS_TOPIC, qos)
        self.services = [
            node.create_service(Trigger, endpoint, self._callback(operation))
            for operation, endpoint in SERVICE_NAMES.items()
        ]
        session.on_transition = self.publish
        self.publish()

    def _callback(self, operation: str):
        def callback(_request, response):
            accepted, payload = self.session.request(operation)
            response.success = accepted
            response.message = json.dumps(payload, ensure_ascii=False)
            return response
        return callback

    def publish(self) -> None:
        if not self.node.context.ok():
            return
        message = self._message_type()
        message.data = json.dumps(self.session.snapshot(), ensure_ascii=False)
        self.publisher.publish(message)
        self._last_publish_ns = time.monotonic_ns()

    def heartbeat(self) -> None:
        if time.monotonic_ns() - self._last_publish_ns >= 250_000_000:
            self.publish()
