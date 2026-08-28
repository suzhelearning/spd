from types import ModuleType, SimpleNamespace
import sys

import numpy as np
import pytest

from spd_vr.runtime import run_runtime


class _FakeNode:
    def __init__(self):
        self.subscriptions = {}
        self.destroyed = False

    def create_subscription(self, msg_type, callback, topic, qos):
        self.subscriptions[topic] = callback
        return SimpleNamespace(msg_type=msg_type, topic=topic, qos=qos)

    def destroy_node(self):
        self.destroyed = True


class _FakeRclpy(ModuleType):
    def __init__(self):
        super().__init__("rclpy")
        self.node = _FakeNode()
        self.initialized = False
        self.shutdown_called = False

    def ok(self):
        return self.initialized

    def init(self, args=None):
        self.initialized = True

    def create_node(self, name):
        assert name == "spd_vr_live_input"
        return self.node

    def spin_once(self, node, timeout_sec=0.0):
        assert node is self.node
        assert timeout_sec == 0.0

    def shutdown(self):
        self.shutdown_called = True
        self.initialized = False


class _FakeQoS(ModuleType):
    def __init__(self):
        super().__init__("rclpy.qos")
        self.qos_profile_sensor_data = "sensor"

    class QoSProfile:
        def __init__(self, depth=10, **kwargs):
            self.depth = depth
            self.reliability = kwargs.get("reliability")

    class ReliabilityPolicy:
        RELIABLE = "reliable"


class _Pose:
    def __init__(self, index):
        self.position = SimpleNamespace(x=float(index), y=0.0, z=0.0)
        self.orientation = SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)


class _Hands:
    def __init__(self, sequence):
        self.header = SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=sequence))
        self.tracking_epoch = 3
        self.sequence_id = sequence
        self.left_active = True
        self.right_active = True
        self.left_scale = 1.0
        self.right_scale = 1.0
        self.left_joints = [_Pose(index) for index in range(26)]
        self.right_joints = [_Pose(index) for index in range(26)]


class _Bool:
    def __init__(self, value):
        self.data = value


def _install_fake_ros(monkeypatch):
    rclpy = _FakeRclpy()
    qos = _FakeQoS()
    node_mod = ModuleType("rclpy.node")
    node_mod.Node = object
    pico_msg = ModuleType("pico_bridge.msg")
    pico_msg.PicoHands = _Hands
    pico_pkg = ModuleType("pico_bridge")
    pico_pkg.msg = pico_msg
    std_msg = ModuleType("std_msgs.msg")
    std_msg.Bool = _Bool
    std_pkg = ModuleType("std_msgs")
    std_pkg.msg = std_msg
    monkeypatch.setitem(sys.modules, "rclpy", rclpy)
    monkeypatch.setitem(sys.modules, "rclpy.qos", qos)
    monkeypatch.setitem(sys.modules, "rclpy.node", node_mod)
    monkeypatch.setitem(sys.modules, "pico_bridge", pico_pkg)
    monkeypatch.setitem(sys.modules, "pico_bridge.msg", pico_msg)
    monkeypatch.setitem(sys.modules, "std_msgs", std_pkg)
    monkeypatch.setitem(sys.modules, "std_msgs.msg", std_msg)
    return rclpy


def test_mailbox_is_latest_only_and_pause_edges_clear_frames(monkeypatch):
    from spd_vr.ros_input import LiveInputMailbox
    rclpy = _install_fake_ros(monkeypatch)

    mailbox = LiveInputMailbox("/pico/hands", "/spd_vr/pause")
    hands_callback = rclpy.node.subscriptions["/pico/hands"]
    pause_callback = rclpy.node.subscriptions["/spd_vr/pause"]
    hands_callback(_Hands(1))
    hands_callback(_Hands(2))
    assert mailbox.take_latest_hands().sequence_id == 2
    assert mailbox.take_latest_hands() is None

    pause_callback(_Bool(False))
    assert mailbox.take_episode_commands() == []
    hands_callback(_Hands(3))
    pause_callback(_Bool(True))
    assert mailbox.take_latest_hands() is None
    assert [command.value for command in mailbox.take_episode_commands()] == ["pause"]
    hands_callback(_Hands(4))
    assert mailbox.take_latest_hands() is None
    pause_callback(_Bool(True))
    assert mailbox.take_episode_commands() == []
    pause_callback(_Bool(False))
    assert [command.value for command in mailbox.take_episode_commands()] == ["resume"]
    assert mailbox.hands_callback_count == 4
    assert mailbox.paused_hands_count == 1
    mailbox.close()
    assert rclpy.node.destroyed is True


def test_mock_runtime_still_writes_a_valid_episode(tmp_path):
    episode = run_runtime(
        output=tmp_path,
        scene="jenga",
        task="handover_lr",
        duration_s=0.05,
        seed=7,
        headless=True,
        mock=True,
    )
    assert (episode / "episode.hdf5").is_file()
    assert (episode / "manifest.json").is_file()
