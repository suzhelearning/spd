"""Operator transitions against real DDS, physics and durable episode files."""
import importlib.util
import os
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch
from uuid import uuid4

import numpy as np

try:
    ROS_AVAILABLE = (importlib.util.find_spec("rclpy") is not None
                     and importlib.util.find_spec("tianji_spd_interfaces.msg") is not None)
except ModuleNotFoundError:
    ROS_AVAILABLE = False


@unittest.skipUnless(ROS_AVAILABLE, "requires ros-jazzy and the local native overlay")
class CollectionControlTests(unittest.TestCase):
    def setUp(self):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from tianji_spd_interfaces.msg import JointCommand
        from interfaces.ros_joint_command import TOPIC, best_effort_qos
        from description.model_builder import config_root
        from simulation.ros_viewer import RosViewerApp

        directory = tempfile.TemporaryDirectory(prefix="spd-collection-control-")
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name)
        environment = patch.dict(os.environ, ROS_DOMAIN_ID="126")
        environment.start()
        self.addCleanup(environment.stop)
        self.context = rclpy.context.Context()
        self.context.init(domain_id=126)
        self.addCleanup(self.context.try_shutdown)
        self.peer = rclpy.create_node("collection_source", context=self.context)
        self.addCleanup(self.peer.destroy_node)
        self.ros = SingleThreadedExecutor(context=self.context)
        self.ros.add_node(self.peer)
        self.addCleanup(self.ros.shutdown)
        self.publisher = self.peer.create_publisher(JointCommand, TOPIC, best_effort_qos())
        args = SimpleNamespace(collection_config=config_root() / "collect_sim.yaml", output=self.root,
                               max_frames=0, scene="hardware_free", task=None, seed=0,
                               table_distance=None, headless=True, height_m=None, repeat_task=False)
        self.app = RosViewerApp(args)
        self.addCleanup(self.app.close)
        self.control = self.app.three_key
        self.collection = self.app.collection
        self.session_id = uuid4().hex
        self.sequence = 0
        self.targets = self.app.plant.joint_command_targets().copy()
        self.online = True
        self.until(lambda: self.app.executor.mailbox.latest is not None)

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
        self.ros.spin_once(timeout_sec=0.)
        self.app.executor.spin_once()
        self.control.poll()
        self.collection.poll()
        if not self.collection.physics_paused:
            self.app.executor.apply_pending()
            if self.app.executor.mailbox.enabled and not self.app.executor.hold_mask & 1:
                self.collection.tick(self.app.plant.physics_tick(), recovery=self.control.recovery,
                                     control_flags=self.control.control_flags)
            else:
                self.control.poll()

    def until(self, predicate, timeout=15):
        deadline = time.monotonic() + timeout
        while not predicate():
            self.assertLess(time.monotonic(), deadline,
                            (self.control.stage, self.control.notice, self.collection.snapshot()))
            self.cycle()
            time.sleep(.002)

    def start(self):
        self.control.key("r")
        self.until(lambda: self.control.stage == "recording" and self.collection.state_frames > 2)

    def test_moving_checkpoint_rewind_auto_resumes_and_paused_r_saves_prefix(self):
        import h5py
        from data_collector.recorder import validate_episode_path

        self.control.key("s")
        self.assertEqual(self.collection.state, "idle")
        self.targets[0] += .2
        self.start()
        self.until(lambda: self.control.recovery == 0)
        self.assertEqual(self.collection.snapshot()["checkpoint_frames"], 0)
        self.control.key("r")
        checkpoint = self.collection.snapshot()["checkpoint_frames"]
        saved_qpos = self.app.plant.data.qpos.copy()
        saved_qvel = self.app.plant.data.qvel.copy()
        saved_targets = self.app.plant.joint_command_targets().copy()
        self.until(lambda: self.collection.state_frames > checkpoint + 3)
        self.control.key("d")
        self.assertTrue(self.collection.physics_paused)
        for key in ("r", "s", "d"):
            self.app.joint_control(key)  # Disk-stage keys must never be replayed.
        self.until(lambda: self.control.stage == "rebinding")
        self.app._process_actions()
        self.assertEqual(self.collection.state_frames, checkpoint)
        self.assertEqual(self.control.stage, "rebinding")
        np.testing.assert_array_equal(self.app.plant.data.qpos, saved_qpos)
        np.testing.assert_array_equal(self.app.plant.data.qvel, saved_qvel)
        np.testing.assert_array_equal(self.app.plant.joint_command_targets(), saved_targets)
        self.until(lambda: self.control.stage == "recording" and self.control.recovery == 3)
        self.until(lambda: self.collection.state_frames > checkpoint + 3)
        previous = self.app.plant
        final_frames = self.collection.state_frames
        self.control.key("s")
        self.control.key("r")
        self.assertTrue(self.collection.physics_paused)
        self.app.joint_control("r")  # Queued for the old scene: never start the new one.
        self.until(lambda: self.control.stage == "idle")
        self.app._process_actions()
        self.assertIsNot(self.app.plant, previous)
        self.assertEqual(self.collection.last_outcome, "saved")
        np.testing.assert_array_equal(self.app.plant.joint_command_positions(), self.app.home_targets)
        np.testing.assert_array_equal(self.app.plant.joint_command_targets(), self.app.home_targets)
        np.testing.assert_array_equal(self.app.plant.data.qvel, 0)
        self.assertTrue(self.collection.physics_paused)
        self.assertFalse(self.app.executor.mailbox.enabled)
        home = self.app.plant.data.qpos.copy()
        for _ in range(5):
            self.cycle()
        np.testing.assert_array_equal(self.app.plant.data.qpos, home)
        self.assertEqual(self.collection.state, "idle")
        result = validate_episode_path(self.collection.last_saved_path)
        self.assertTrue(result["success"])
        self.assertEqual(result["frames"], final_frames)
        with h5py.File(self.collection.last_saved_path) as handle:
            self.assertIn(3, handle["collection_events/recovery_transition"][:])
            self.assertEqual(int(handle["collection_events/rewind"][-1]["frame_count"]), checkpoint)
            self.assertEqual(handle["collection_events/control_flags"].shape,
                             handle["collection_events/recovery_transition"].shape)
        self.targets = self.app.home_targets.copy()
        self.start()  # Only a fresh r may authorize and create the next episode.

    def test_manual_pause_uses_s_and_paused_d_discards_without_confirmation(self):
        for key in ("s", "d", " ", "x", "r+s", "s+d"):
            self.control.key(key)
        self.assertEqual(self.control.stage, "idle")
        self.assertIsNone(self.collection.snapshot()["checkpoint_frames"])
        self.control.key("r")
        self.assertEqual(self.control.stage, "binding")
        self.assertIsNone(self.collection.snapshot()["checkpoint_frames"])
        self.control.key("s")  # Cancel an initial bind without opening a file.
        self.assertEqual(self.control.stage, "idle")
        self.start()
        for key in (" ", "x", "r+s", "s+d"):
            self.control.key(key)
        self.assertEqual(self.control.stage, "recording")
        self.control.key("s")
        positions = self.app.plant.data.qpos.copy()
        velocities = self.app.plant.data.qvel.copy()
        frames = self.collection.state_frames
        self.targets[0] += .3
        for _ in range(10):
            self.cycle()
        np.testing.assert_array_equal(self.app.plant.data.qpos, positions)
        np.testing.assert_array_equal(self.app.plant.data.qvel, velocities)
        self.assertEqual(self.collection.state_frames, frames)
        self.control.key("s")
        self.until(lambda: self.control.stage == "recording" and self.control.recovery == 2)
        self.until(lambda: self.collection.state_frames > frames)
        previous = self.app.plant
        partial = Path(self.collection.episode_path)
        self.control.key("s")
        self.control.key("d")
        self.assertEqual(self.control.stage, "discarding")
        self.until(lambda: self.control.stage == "idle")
        self.assertEqual(self.collection.last_outcome, "discarded")
        self.assertFalse(partial.exists())
        self.assertEqual(self.collection.last_saved_path, "")
        self.assertIsNot(self.app.plant, previous)
        np.testing.assert_array_equal(self.app.plant.joint_command_positions(), self.app.home_targets)
        np.testing.assert_array_equal(self.app.plant.data.qvel, 0)
        self.assertFalse(self.app.executor.mailbox.enabled)
        self.assertTrue(self.collection.physics_paused)

    def test_each_tracking_loss_rebinds_without_replacing_manual_checkpoint(self):
        from pico2_hands.collection_session import TeleopSnapshot
        from data_collector.recorder import validate_episode_path
        import h5py

        self.start()
        self.control.key("r")
        manual_frames = self.collection.state_frames
        self.until(lambda: self.collection.state_frames > manual_frames + 2)
        self.online = False
        state = SimpleNamespace(tracked=False, generation=0, sequence=0, mode="follow",
                                targets=self.app.plant.joint_command_targets().copy())

        def snapshot():
            state.sequence += 1
            now = time.monotonic_ns() - 1_000_000
            return TeleopSnapshot(state.generation, state.sequence, tuple(state.targets),
                                  now, now if state.tracked else now - 200_000_000,
                                  state.tracked, state.tracked, not state.tracked,
                                  0 if state.tracked else 1, ("live", "live"), state.mode, "")

        def pause():
            state.mode = "waiting"

        def rebind(targets):
            state.generation += 1
            state.targets = targets.copy()
            state.mode = "bound"
            return state.generation

        def follow(generation):
            state.mode = "follow"

        self.app.teleop = SimpleNamespace(snapshot=snapshot, pause=pause, rebind=rebind,
                                         follow=follow, close=lambda: None)
        for _ in range(2):
            state.tracked = False
            self.until(lambda: self.control.stage == "auto_paused")
            saved_frames = self.collection.state_frames
            saved_tick = self.app.plant.tick
            saved_qpos = self.app.plant.data.qpos.copy()
            saved_qvel = self.app.plant.data.qvel.copy()
            saved_targets = self.app.plant.joint_command_targets().copy()
            for key in ("r", "s", "d", " ", "x", "r+s", "s+d"):
                self.control.key(key)
                self.cycle()
                self.assertEqual(self.control.stage, "auto_paused")
            state.tracked = True
            for _ in range(20):
                self.cycle()
            self.assertEqual(self.control.stage, "auto_paused")
            self.control.key("s")
            self.control.key("d")
            self.assertEqual(self.control.stage, "auto_paused")
            self.control.key("r")
            self.assertEqual(self.control.stage, "rebinding")
            self.assertEqual(self.collection.state_frames, saved_frames)
            self.assertEqual(self.app.plant.tick, saved_tick)
            np.testing.assert_array_equal(self.app.plant.data.qpos, saved_qpos)
            np.testing.assert_array_equal(self.app.plant.data.qvel, saved_qvel)
            np.testing.assert_array_equal(self.app.plant.joint_command_targets(), saved_targets)
            self.assertEqual(self.collection.snapshot()["checkpoint_frames"], manual_frames)
            self.until(lambda: self.control.stage == "recording" and self.control.recovery == 4)
            self.assertIsNone(self.collection.snapshot()["auto_checkpoint_frames"])
            self.until(lambda: self.collection.state_frames > saved_frames + 2)
        # First r after either loss does not overwrite the original manual point.
        state.tracked = False
        self.control.key("d")
        self.until(lambda: self.control.stage == "rebinding")
        self.assertEqual(self.collection.state_frames, manual_frames)
        for _ in range(5):
            self.cycle()
        self.assertEqual(self.control.stage, "rebinding")
        self.assertTrue(self.collection.physics_paused)
        state.tracked = True
        self.until(lambda: self.control.stage == "recording" and self.control.recovery == 3)
        self.until(lambda: self.collection.state_frames > manual_frames + 2)
        self.control.key("r")
        updated_frames = self.collection.state_frames
        self.assertEqual(self.collection.snapshot()["checkpoint_frames"], updated_frames)
        self.until(lambda: self.collection.state_frames > updated_frames + 2)
        self.control.key("d")
        self.until(lambda: self.control.stage == "rebinding")
        self.assertEqual(self.collection.state_frames, updated_frames)
        self.until(lambda: self.control.stage == "recording")
        self.until(lambda: self.collection.state_frames > updated_frames + 2)
        self.control.key("s")
        paused_frames = self.collection.state_frames
        state.tracked = False
        self.control.key("s")
        for _ in range(5):
            self.cycle()
        self.assertEqual(self.control.stage, "rebinding")
        self.assertEqual(self.collection.state_frames, paused_frames)
        state.tracked = True
        self.until(lambda: self.control.stage == "recording" and self.control.recovery == 2)
        self.until(lambda: self.collection.state_frames > paused_frames + 2)
        final_frames = self.collection.state_frames
        teleop = self.app.teleop
        stale_snapshot = snapshot()
        self.control.key("s")
        self.control.key("r")
        self.collection._job.result(timeout=10)
        self.collection.poll()
        result = validate_episode_path(self.collection.last_saved_path)
        self.assertEqual(result["frames"], final_frames)
        with h5py.File(self.collection.last_saved_path) as handle:
            self.assertIn(3, handle["collection_events/recovery_transition"][:])
            self.assertEqual(int(handle["collection_events/rewind"][-1]["frame_count"]), updated_frames)
        self.control.poll()
        self.assertIs(self.app.teleop, teleop)
        self.assertEqual(self.control.stage, "idle")
        self.assertTrue(self.collection.physics_paused)
        self.assertFalse(self.app.executor.mailbox.enabled)
        np.testing.assert_array_equal(self.app.plant.joint_command_positions(), self.app.home_targets)
        np.testing.assert_array_equal(self.app.plant.data.qvel, 0)
        with patch.object(teleop, "snapshot", return_value=stale_snapshot):
            for _ in range(5):
                self.control.poll()
            state.tracked = True
            self.control.key("r")
            for _ in range(5):
                self.control.poll()
            self.assertEqual(self.control.stage, "binding")
            self.assertTrue(self.collection.physics_paused)
            self.assertIsNone(self.app.executor.mailbox.latest)
        self.assertFalse(self.app.executor.mailbox.enabled)
        self.until(lambda: self.control.stage == "recording" and self.collection.state_frames > 2)
        self.assertGreater(state.generation, stale_snapshot.generation)

    def test_preparation_freezes_and_disk_failure_preserves_partial_and_scene(self):
        import threading
        entered, release = threading.Event(), threading.Event()
        original = self.collection.recorder.start_episode

        def delayed_start(*args, **kwargs):
            entered.set()
            if not release.wait(3):
                raise TimeoutError("writer not released")
            return original(*args, **kwargs)

        with patch.object(self.collection.recorder, "start_episode", delayed_start):
            before = self.app.plant.tick
            self.control.key("r")
            try:
                self.until(entered.is_set)
                self.control.key("s")  # Safety pause during asynchronous file open.
                for _ in range(4):
                    self.cycle()
                self.assertEqual(self.app.plant.tick, before)
            finally:
                release.set()
            self.until(lambda: self.control.stage == "paused")
        self.assertEqual(self.collection.state_frames, 0)
        self.control.key("s")
        self.until(lambda: self.collection.state_frames > 2)
        plant = self.app.plant
        self.control.key("s")
        with patch.object(self.collection.recorder, "finish_episode", side_effect=OSError("disk unavailable")):
            self.control.key("r")
            self.until(lambda: self.control.stage == "error")
        self.assertTrue(self.collection.physics_paused)
        self.assertIs(self.app.plant, plant)
        self.assertEqual(self.collection.completed_episodes, 0)
        self.assertEqual(self.collection.last_saved_path, "")
        self.assertTrue(Path(self.collection.episode_path).is_file())

    def test_terminal_keys_use_single_authority_and_quit_preserves_partial(self):
        import pty
        from interfaces.keyboard_control import ControlTerminal

        master, slave = pty.openpty()
        self.addCleanup(os.close, slave)
        self.addCleanup(os.close, master)
        terminal = ControlTerminal(self.app.joint_control, fd=slave)
        self.addCleanup(terminal.close)
        terminal.start()

        def key(text):
            os.write(master, text.encode())
            self.until(lambda: not self.app._actions.empty())
            self.app._process_actions()

        key("r")
        self.until(lambda: self.collection.state_frames > 2)
        key("s")
        self.assertTrue(self.collection.physics_paused)
        key("s")
        self.until(lambda: self.control.stage == "recording")
        key("q")
        self.assertTrue(self.app.stop)
        self.app.close()
        self.assertEqual(self.collection.last_saved_path, "")
        self.assertTrue(Path(self.collection.episode_path).is_file())


if __name__ == "__main__":
    unittest.main()
