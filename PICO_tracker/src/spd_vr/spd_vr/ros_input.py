"""Lazy ROS input mailbox for live PICO hand and episode pause messages.

The mailbox deliberately contains no simulator or MuJoCo references.  ROS
callbacks only copy an atomic hand message or enqueue an episode command;
the runtime owns all state mutation and consumes the snapshots from its loop.
"""

from __future__ import annotations

import copy
import queue
import threading
from typing import Any

from .episode import EpisodeCommandType


class LiveInputMailbox:
    """Keep only the newest ``PicoHands`` message and pause edge commands.

    ROS dependencies are imported and the node is created only when this
    class is instantiated.  This keeps deterministic ``--mock`` runs usable
    on machines without a ROS installation.
    """

    def __init__(self, hands_topic: str, pause_topic: str) -> None:
        self.hands_topic = str(hands_topic)
        self.pause_topic = str(pause_topic)
        self._lock = threading.Lock()
        self._latest_hands: Any | None = None
        self._commands: queue.SimpleQueue[EpisodeCommandType] = queue.SimpleQueue()
        self._paused = False
        self._pause_seeded = False
        self._closed = False
        self._hands_callback_count = 0
        self._paused_hands_count = 0
        self._pause_callback_count = 0
        self._pause_edge_count = 0
        self._owns_context = False

        try:
            import rclpy
        except ImportError as exc:  # pragma: no cover - depends on host ROS install
            raise RuntimeError(
                "live mode requires the ROS dependency 'rclpy'"
            ) from exc
        try:
            from pico_bridge.msg import PicoHands
        except ImportError as exc:  # pragma: no cover - depends on ROS workspace
            raise RuntimeError(
                "live mode requires the ROS dependency 'pico_bridge' with PicoHands"
            ) from exc
        try:
            from std_msgs.msg import Bool
        except ImportError as exc:  # pragma: no cover - depends on host ROS install
            raise RuntimeError(
                "live mode requires the ROS dependency 'std_msgs' with Bool"
            ) from exc
        try:
            from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
        except ImportError as exc:  # pragma: no cover - depends on host ROS install
            raise RuntimeError(
                "live mode requires the ROS dependency 'rclpy' QoS support"
            ) from exc

        self._rclpy = rclpy
        self._node: Any | None = None
        try:
            try:
                context_ok = bool(rclpy.ok())
            except (AttributeError, RuntimeError):
                context_ok = False
            if not context_ok:
                # Mark ownership before init: rclpy can activate the default
                # context and then fail while installing signal handlers.
                self._owns_context = True
                try:
                    rclpy.init(args=None)
                except TypeError:
                    # Small test doubles and older rclpy releases may not accept
                    # the keyword; the real API does.
                    rclpy.init()

            create_node = getattr(rclpy, "create_node", None)
            if callable(create_node):
                self._node = create_node("spd_vr_live_input")
            else:
                try:
                    from rclpy.node import Node
                except ImportError as exc:  # pragma: no cover - host ROS install
                    raise RuntimeError(
                        "live mode requires the ROS dependency 'rclpy' node support"
                    ) from exc
                self._node = Node("spd_vr_live_input")
            try:
                reliable_qos = QoSProfile(depth=10)
            except TypeError:
                reliable_qos = QoSProfile()
            try:
                reliable_qos.reliability = ReliabilityPolicy.RELIABLE
            except (AttributeError, TypeError):
                # QoSProfile implementations that accept a default reliable policy
                # need no extra mutation.
                pass
            self._hands_subscription = self._node.create_subscription(
                PicoHands, self._on_hands, self.hands_topic, qos_profile_sensor_data
            )
            self._pause_subscription = self._node.create_subscription(
                Bool, self._on_pause, self.pause_topic, reliable_qos
            )
        except Exception:
            # ``__init__`` has not returned yet, so runtime cannot call close()
            # through its mailbox variable.  Release any node/context acquired
            # during this staged setup before propagating the original failure.
            self._cleanup_resources(suppress_destroy_errors=True)
            raise

    def _on_hands(self, message: Any) -> None:
        with self._lock:
            self._hands_callback_count += 1
            if self._paused or self._closed:
                self._paused_hands_count += 1
                return
            # ROS messages are mutable Python objects.  Snapshot the complete
            # message before releasing the callback thread's lock.
            self._latest_hands = copy.deepcopy(message)

    def _on_pause(self, message: Any) -> None:
        value = bool(getattr(message, "data", message))
        with self._lock:
            if self._closed:
                return
            self._pause_callback_count += 1
            if not self._pause_seeded:
                self._paused = value
                self._pause_seeded = True
                if value:
                    self._latest_hands = None
                    self._pause_edge_count += 1
                    self._commands.put(EpisodeCommandType.PAUSE)
                return
            if value == self._paused:
                return
            self._paused = value
            self._latest_hands = None
            self._pause_edge_count += 1
            self._commands.put(
                EpisodeCommandType.PAUSE if value else EpisodeCommandType.RESUME
            )

    def spin_once(self, timeout_sec: float) -> None:
        """Dispatch pending ROS callbacks; callbacks never touch MuJoCo."""
        if self._closed:
            return
        self._rclpy.spin_once(self._node, timeout_sec=float(timeout_sec))

    def take_latest_hands(self) -> Any | None:
        """Return and clear the one newest hand message, if not paused."""
        with self._lock:
            if self._paused or self._closed:
                self._latest_hands = None
                return None
            message = self._latest_hands
            self._latest_hands = None
            return message

    def take_episode_commands(self) -> list[EpisodeCommandType]:
        """Drain all pause/resume edges observed since the previous loop."""
        commands: list[EpisodeCommandType] = []
        while True:
            try:
                commands.append(self._commands.get_nowait())
            except queue.Empty:
                return commands

    @property
    def paused(self) -> bool:
        with self._lock:
            return self._paused

    @property
    def hands_callback_count(self) -> int:
        with self._lock:
            return self._hands_callback_count

    @property
    def paused_hands_count(self) -> int:
        with self._lock:
            return self._paused_hands_count

    @property
    def pause_callback_count(self) -> int:
        with self._lock:
            return self._pause_callback_count

    @property
    def pause_edge_count(self) -> int:
        with self._lock:
            return self._pause_edge_count

    @property
    def health(self) -> dict[str, int | bool]:
        with self._lock:
            return {
                "paused": self._paused,
                "hands_callbacks": self._hands_callback_count,
                "paused_hands": self._paused_hands_count,
                "pause_callbacks": self._pause_callback_count,
                "pause_edges": self._pause_edge_count,
            }

    def _cleanup_resources(self, *, suppress_destroy_errors: bool) -> None:
        node = self._node
        self._node = None
        try:
            if node is not None:
                node.destroy_node()
        except Exception:
            if not suppress_destroy_errors:
                raise
        finally:
            if self._owns_context:
                self._owns_context = False
                try:
                    if self._rclpy.ok():
                        self._rclpy.shutdown()
                except (AttributeError, RuntimeError):
                    pass

    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True
            self._latest_hands = None
        self._cleanup_resources(suppress_destroy_errors=False)


__all__ = ["LiveInputMailbox"]
