from types import ModuleType, SimpleNamespace
import json
import sys

import numpy as np
import pytest
from spd_vr.episode import EpisodeCommandType
import spd_vr.retarget_pair as retarget_pair_module
import spd_vr.runtime as runtime_module
from spd_vr.runtime import _convert_live_hands, run_runtime


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


def test_initial_true_pause_is_forwarded_and_invalid_side_isolated(monkeypatch):
    from spd_vr.ros_input import LiveInputMailbox

    rclpy = _install_fake_ros(monkeypatch)
    mailbox = LiveInputMailbox("/pico/hands", "/spd_vr/pause")
    pause_callback = rclpy.node.subscriptions["/spd_vr/pause"]
    pause_callback(_Bool(True))
    assert mailbox.paused is True
    assert mailbox.take_episode_commands() == [EpisodeCommandType.PAUSE]
    mailbox.close()

    bad = _Hands(5)
    bad.left_joints[4].position.x = float("nan")
    frame = _convert_live_hands(bad)
    assert frame.left_active is False
    assert frame.right_active is True
    assert np.isfinite(frame.left_hand).all()
    assert frame.right_hand.shape == (26, 7)


@pytest.mark.parametrize(
    ("context_active", "shutdown_expected"),
    [(False, True), (True, False)],
)
def test_mailbox_subscription_failure_cleans_only_owned_ros_resources(
    monkeypatch, context_active, shutdown_expected
):
    from spd_vr.ros_input import LiveInputMailbox

    rclpy = _install_fake_ros(monkeypatch)
    rclpy.initialized = context_active

    def fail_subscription(*_args):
        raise RuntimeError("subscription setup failure")

    rclpy.node.create_subscription = fail_subscription
    with pytest.raises(RuntimeError, match="subscription setup failure"):
        LiveInputMailbox("/pico/hands", "/spd_vr/pause")

    assert rclpy.node.destroyed is True
    assert rclpy.shutdown_called is shutdown_expected
    assert rclpy.initialized is context_active


def test_mailbox_partial_rclpy_init_failure_shuts_down_owned_context(monkeypatch):
    from spd_vr.ros_input import LiveInputMailbox

    rclpy = _install_fake_ros(monkeypatch)

    def fail_init(*_args, **_kwargs):
        rclpy.initialized = True
        raise RuntimeError("signal handler setup failure")

    rclpy.init = fail_init
    with pytest.raises(RuntimeError, match="signal handler setup failure"):
        LiveInputMailbox("/pico/hands", "/spd_vr/pause")

    assert rclpy.shutdown_called is True
    assert rclpy.initialized is False


def test_live_mailbox_is_closed_when_simulator_setup_fails(tmp_path, monkeypatch):
    class _Task:
        def reset(self, _seed):
            return SimpleNamespace(
                manifest=lambda: {"scene": "test"},
                objects=[],
            )

    class _Mailbox:
        instances = []

        def __init__(self, hands_topic, pause_topic):
            self.closed = False
            self.hands_topic = hands_topic
            self.pause_topic = pause_topic
            self.__class__.instances.append(self)

        def close(self):
            self.closed = True

    class _Pair:
        def __init__(self, *_args):
            pass

    def fail_simulator(**_kwargs):
        raise RuntimeError("simulator setup failure")

    monkeypatch.setattr(runtime_module, "LiveInputMailbox", _Mailbox)
    monkeypatch.setattr(retarget_pair_module, "WujiRetargetPair", _Pair)
    monkeypatch.setattr(runtime_module, "get_task", lambda _scene, _task: _Task())
    monkeypatch.setattr(runtime_module, "write_scene_model", lambda *_args: None)
    monkeypatch.setattr(runtime_module, "UnifiedSimulator", fail_simulator)
    with pytest.raises(RuntimeError, match="simulator setup failure"):
        run_runtime(
            output=tmp_path,
            scene="jenga",
            task="handover_lr",
            duration_s=0.01,
            mock=False,
        )
    assert len(_Mailbox.instances) == 1
    assert _Mailbox.instances[0].closed is True

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
    manifest = json.loads((episode / "manifest.json").read_text())
    assert manifest["task_reset_manifest"]["teleop"]["input"] == "mock_pico_hands"
    assert (episode / "manifest.json").is_file()

def test_live_tick_pacer_uses_deadlines_and_resets_after_pause():
    from spd_vr.runtime import _LiveTickPacer

    class FakeClock:
        def __init__(self):
            self.now_ns = 10_000
            self.sleeps_ns = []

        def now(self):
            return self.now_ns

        def sleep(self, seconds):
            duration_ns = int(round(seconds * 1_000_000_000))
            self.sleeps_ns.append(duration_ns)
            self.now_ns += duration_ns

    clock = FakeClock()
    pacer = _LiveTickPacer(1_000_000_000 // 480, clock.now, clock.sleep)

    pacer.wait()
    assert clock.sleeps_ns == []
    pacer.wait()
    assert clock.sleeps_ns == [1_000_000_000 // 480]

    # Pause resets the deadline; resume starts from the current instant and
    # must not sleep for a stale pre-pause deadline.
    pacer.reset()
    pacer.wait()
    assert clock.sleeps_ns == [1_000_000_000 // 480]

def test_mock_runs_persist_deterministic_sim_clock_timestamps(tmp_path):
    import h5py

    from spd_vr.runtime import MOCK_EPOCH_NS

    episodes = []
    for name in ("first", "second"):
        episodes.append(
            run_runtime(
                output=tmp_path / name,
                scene="jenga",
                task="handover_lr",
                duration_s=0.05,
                seed=23,
                headless=True,
                mock=True,
            )
        )

    with h5py.File(episodes[0] / "episode.hdf5", "r") as first, h5py.File(
        episodes[1] / "episode.hdf5", "r"
    ) as second:
        first_timestamps = first["timestamps/hands_ns"][:]
        second_timestamps = second["timestamps/hands_ns"][:]
    np.testing.assert_array_equal(first_timestamps, second_timestamps)
    assert first_timestamps[0] == MOCK_EPOCH_NS
    assert np.all(first_timestamps >= MOCK_EPOCH_NS)
    np.testing.assert_array_equal(
        first_timestamps - MOCK_EPOCH_NS,
        np.asarray([0, 16_666_667, 35_416_667], dtype=np.uint64),
    )
