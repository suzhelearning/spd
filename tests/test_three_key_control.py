"""Local R/S/D behavior against real DDS targets, physics and recorded files."""
import importlib.util
import os
from pathlib import Path
import tempfile
import time
import unittest
from uuid import uuid4

from _spd_native import ThreeKeyControl

try:
    ROS_AVAILABLE = (importlib.util.find_spec("rclpy") is not None
                     and importlib.util.find_spec("tianji_spd_interfaces.msg") is not None)
except ModuleNotFoundError:
    ROS_AVAILABLE = False


@unittest.skipUnless(ROS_AVAILABLE, "run inside ros-jazzy with the local interface overlay")
class ThreeKeyGateTests(unittest.TestCase):
    def setUp(self):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from tianji_spd_interfaces.msg import JointCommand
        from data_collector.config import CollectionConfig
        from data_collector.session import CollectionSession
        from _spd_native import RosJointCommandExecutor
        from interfaces.ros_joint_command import TOPIC, best_effort_qos
        from simulation.viewer import PlantController

        directory = tempfile.TemporaryDirectory(prefix="spd-rsd-test-")
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name)
        self.context = rclpy.context.Context()
        self.context.init(domain_id=126)
        self.addCleanup(self.context.try_shutdown)
        self.node = rclpy.create_node("local_rsd_test", context=self.context)
        self.addCleanup(self.node.destroy_node)
        self.peer = rclpy.create_node("live_joint_source", context=self.context)
        self.addCleanup(self.peer.destroy_node)
        self.ros = SingleThreadedExecutor(context=self.context)
        self.ros.add_node(self.node)
        self.ros.add_node(self.peer)
        self.addCleanup(self.ros.shutdown)
        self.publisher = self.peer.create_publisher(JointCommand, TOPIC, best_effort_qos())
        self.plant = PlantController()
        self.addCleanup(self.plant.close)
        self.executor = RosJointCommandExecutor(self.plant, domain_id=126)
        self.addCleanup(self.executor.close)
        self.collection = CollectionSession(
            CollectionConfig(2, self.root, 60, 128, 0), self.plant, self.executor, {"task": "rsd-test"},
        )
        self.addCleanup(self.collection.close)
        self.control = ThreeKeyControl(self.collection, self.executor)
        self.session_id = str(uuid4())
        self.sequence = 0
        self.targets = self.plant.joint_command_targets()
        self.online = True
        self.until(lambda: self.executor.mailbox.latest is not None)

    def cycle(self):
        from interfaces.ros_joint_command import JOINT_NAMES
        from tianji_spd_interfaces.msg import JointCommand

        if self.online:
            self.sequence += 1
            message = JointCommand(schema_version=1, robot_config="tianji_wuji2_v1",
                                   session_id=self.session_id, sequence=self.sequence, ready_mask=7,
                                   joint_names=list(JOINT_NAMES), position_rad=self.targets.tolist())
            message.stamp.sec, message.stamp.nanosec = divmod(time.time_ns(), 1_000_000_000)
            self.publisher.publish(message)
        for _ in range(4):
            self.ros.spin_once(timeout_sec=0.)
        self.executor.spin_once()
        self.control.poll()
        self.collection.poll()
        if not self.collection.physics_paused:
            self.executor.apply_pending()
            if self.executor.mailbox.enabled and not self.executor.hold_mask & 1:
                self.collection.tick(self.plant.physics_tick(), recovery=self.control.recovery)
            else:
                self.control.poll()

    def until(self, predicate):
        deadline = time.monotonic() + 5
        while not predicate():
            self.assertLess(time.monotonic(), deadline, (self.control.stage, self.control.notice, self.collection.snapshot()))
            self.cycle()
            time.sleep(.002)

    def press(self, key):
        self.control.key(key)

    def start(self):
        self.press("r")
        self.assertEqual(self.collection.snapshot()["checkpoint_frames"], 0)
        self.until(lambda: self.control.stage == "recording" and self.collection.state_frames > 2)

    def test_large_gap_start_keys_locked_then_checkpoint_rewind_auto_recovers(self):
        import h5py
        from data_collector.recorder import validate_episode_path

        self.press("s")  # Local s never opens an episode.
        self.assertEqual(self.collection.state, "idle")
        self.targets[0] += .4
        self.cycle()
        self.press("r")
        self.assertEqual(self.collection.snapshot()["checkpoint_frames"], 0)
        self.until(lambda: self.control.stage == "blending")
        for key in "rsd":
            self.press(key)
        self.assertEqual(self.control.stage, "blending")
        self.assertEqual(self.collection.snapshot()["checkpoint_frames"], 0)
        self.until(lambda: self.control.stage == "recording")
        count = self.collection.state_frames
        self.press("d")
        self.assertEqual(self.collection.state_frames, count)
        self.assertEqual(self.collection.state, "recording")
        self.press("r")
        checkpoint = self.collection.snapshot()["checkpoint_frames"]
        self.assertGreater(checkpoint, 0)
        self.press("s")
        pose = self.plant.joint_command_positions().copy()
        self.targets[0] += .3
        for _ in range(5):
            self.cycle()
        self.assertTrue((self.plant.joint_command_positions() == pose).all())
        self.assertEqual(self.collection.state_frames, count)
        self.press("s")
        self.until(lambda: self.control.stage == "recording")
        self.press("s")
        self.press("d")
        self.assertEqual(self.control.stage, "reverting")
        self.until(lambda: self.control.stage == "blending")
        self.until(lambda: self.control.stage == "recording")  # No additional s.
        self.press("s")
        self.press("r")
        self.assertEqual(self.collection.state, "paused")
        self.press("r")
        self.until(lambda: self.control.stage == "idle")
        saved = self.collection.last_saved_path
        self.assertTrue(validate_episode_path(saved)["success"])
        with h5py.File(saved) as handle:
            flags = handle['collection_events/recovery_transition'][:]
            self.assertIn(1, flags)
            self.assertIn(3, flags)
            self.assertNotIn(2, flags)  # Resume branch was rewound away.
            self.assertEqual(int(handle['collection_events/rewind'][-1]['frame_count']), checkpoint)

    def test_initial_checkpoint_rewind_and_stale_transition_fail_closed(self):
        self.start()
        self.press("s")
        self.press("d")
        self.until(lambda: self.control.stage == "rewind_wait")
        self.online = False
        self.executor.clear()
        for _ in range(3):  # A normal inter-packet gap is not an automatic-resume failure.
            self.cycle()
        self.assertEqual(self.control.stage, "rewind_wait")
        self.assertTrue(self.collection.physics_paused)
        self.online = True
        self.until(lambda: self.control.stage == "blending")
        self.assertLess(self.collection.state_frames, 5)
        self.online = False
        self.until(lambda: self.control.stage == "paused")
        self.assertTrue(self.collection.physics_paused)
        self.assertFalse(self.executor.mailbox.enabled)
        self.online = True
        for _ in range(4):
            self.cycle()
        self.assertEqual(self.control.stage, "paused")
        self.press("s")
        self.until(lambda: self.control.stage == "recording")

    def test_preparation_does_not_advance_physics_and_save_error_never_advances_task(self):
        import threading
        from unittest.mock import patch

        entered, release = threading.Event(), threading.Event()
        start_episode = self.collection.recorder.start_episode
        def delayed_start(*args, **kwargs):
            entered.set()
            if not release.wait(2):
                raise TimeoutError("test writer not released")
            return start_episode(*args, **kwargs)
        with patch.object(self.collection.recorder, "start_episode", delayed_start):
            try:
                before = self.plant.tick
                self.press("r")
                self.until(entered.is_set)
                for _ in range(5):
                    self.cycle()
                self.assertEqual(self.plant.tick, before)
                self.assertEqual(self.collection.snapshot()["checkpoint_frames"], 0)
            finally:
                release.set()
            self.until(lambda: self.control.stage == "recording")
        self.press("s")
        with patch.object(self.collection.recorder, "finish_episode", side_effect=OSError("disk unavailable")):
            self.press("r")
            self.press("r")
            self.until(lambda: self.control.stage == "error")
        self.assertTrue(self.collection.physics_paused)
        self.assertEqual(self.collection.completed_episodes, 0)
        self.assertEqual(self.collection.last_saved_path, "")
        self.assertTrue(Path(self.collection.episode_path).is_file())

    def test_real_terminal_starts_pauses_and_saves_without_remote_control(self):
        import pty
        import queue
        from interfaces.keyboard_control import ControlTerminal
        from simulation.ros_viewer import RosViewerApp
        from data_collector.recorder import validate_episode_path

        app = RosViewerApp.__new__(RosViewerApp)
        app.three_key, app.collection = self.control, self.collection
        app._actions = queue.SimpleQueue()
        master, slave = pty.openpty()
        self.addCleanup(os.close, slave)
        self.addCleanup(os.close, master)
        terminal = ControlTerminal(app.joint_control, app.recording_control, fd=slave)
        self.addCleanup(terminal.close)
        terminal.start()
        def key(text):
            os.write(master, text.encode())
            deadline = time.monotonic() + 1
            while app._actions.empty():
                self.assertLess(time.monotonic(), deadline)
                time.sleep(.002)
            app._process_actions()
            self.cycle()
        key("r")
        self.until(lambda: self.control.stage == "recording")
        key("s")
        key("r")
        self.assertEqual(self.collection.last_saved_path, "")
        key("r")
        self.until(lambda: self.control.stage == "idle")
        self.assertTrue(validate_episode_path(self.collection.last_saved_path)["recovery_annotated"])


if __name__ == '__main__':
    unittest.main()
