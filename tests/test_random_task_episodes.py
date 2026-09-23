"""Real episode boundaries must rebuild random scenes, never paused branches."""
import argparse
import importlib.util
from pathlib import Path
import tempfile
import time
import unittest

import numpy as np

try:
    ROS_AVAILABLE = (importlib.util.find_spec("rclpy") is not None
                     and importlib.util.find_spec("tianji_spd_interfaces.msg") is not None)
except ModuleNotFoundError:
    ROS_AVAILABLE = False


@unittest.skipUnless(ROS_AVAILABLE, "run inside ros-jazzy with the local interface overlay")
class RandomTaskEpisodeTests(unittest.TestCase):
    def setUp(self):
        from description.model_builder import config_root
        from simulation.ros_viewer import RosViewerApp

        directory = tempfile.TemporaryDirectory(prefix="spd-random-episode-test-")
        self.addCleanup(directory.cleanup)
        self.app = RosViewerApp(argparse.Namespace(
            collection_config=config_root() / "collect_sim.yaml", output=Path(directory.name),
            max_frames=0, scene="cups", task=None, seed=7, table_distance=.2,
            headless=True,
        ))
        self.sequence = 0
        self.addCleanup(lambda: self.app.executor.close())
        self.addCleanup(lambda: self.app.plant.close())
        self.addCleanup(lambda: self.app.window.close())
        self.addCleanup(lambda: self.app.collection.close())

    def publish(self):
        from interfaces.ros_joint_command import JOINT_NAMES
        from tianji_spd_interfaces.msg import JointCommand

        self.sequence += 1
        message = JointCommand(schema_version=1, robot_config="tianji_wuji2_v1",
                               session_id="random-task-test", sequence=self.sequence, ready_mask=7,
                               joint_names=list(JOINT_NAMES),
                               position_rad=self.app.plant.joint_command_targets().tolist())
        message.stamp.sec, message.stamp.nanosec = divmod(time.time_ns(), 1_000_000_000)
        self.assertTrue(self.app.executor.mailbox.receive(message))

    def wait_state(self, state):
        end = time.monotonic() + 10
        while self.app.collection.state != state:
            self.assertLess(time.monotonic(), end, self.app.collection.snapshot())
            self.publish()
            self.app.collection.poll()
            time.sleep(.002)

    def start(self):
        self.publish()
        self.assertTrue(self.app.executor.authorize(True))
        self.assertTrue(self.app.collection.request("start")[0])
        self.wait_state("recording")
        for _ in range(16):
            self.app.collection.tick(self.app.plant.physics_tick())

    def assert_rotation(self):
        old = self.app.plant
        robot = old.joint_command_positions().copy(), old.joint_command_velocities().copy()
        targets = old.joint_command_targets()
        self.app.joint_control("e")
        self.app.recording_control("start")  # Queued against the completed scene.
        self.app._maybe_rotate_task()
        self.assertIsNot(self.app.plant, old)
        np.testing.assert_array_equal(self.app.plant.joint_command_positions(), robot[0])
        np.testing.assert_array_equal(self.app.plant.joint_command_velocities(), robot[1])
        np.testing.assert_array_equal(self.app.plant.joint_command_targets(), targets)
        self.app._process_actions()
        self.assertFalse(self.app.executor.mailbox.enabled)
        self.assertEqual(self.app.collection.state, "idle")
        self.assertIsNone(self.app.collection.snapshot()["checkpoint_frames"])
        self.assertFalse(self.app.collection.request("start")[0])
        self.assertEqual(self.app.collection.task_manifest["seed"], self.app.args.seed)
        self.assertEqual(self.app.plant.scene_manifest["table"]["near_edge_x_m"], .2)
        return old

    def test_save_and_discard_rotate_but_pause_and_failed_save_do_not(self):
        from data_collector.recorder import validate_episode_path
        from unittest.mock import patch

        self.start()
        first = self.app.plant
        self.assertTrue(self.app.collection.request("pause")[0])
        self.app._maybe_rotate_task()
        self.assertIs(self.app.plant, first)
        self.assertTrue(self.app.collection.request("save")[0])
        self.wait_state("idle")
        saved = self.app.collection.last_saved_path
        self.assertTrue(validate_episode_path(saved)["success"])
        self.assert_rotation()
        self.assertTrue(Path(saved).is_file())
        self.start()
        self.assertTrue(self.app.collection.request("discard")[0])
        self.wait_state("idle")
        self.assert_rotation()
        self.assertTrue(Path(saved).is_file())
        self.start()
        failed_plant = self.app.plant
        self.assertTrue(self.app.collection.request("pause")[0])
        with patch.object(self.app.collection.recorder, "finish_episode", side_effect=OSError("disk unavailable")):
            self.assertTrue(self.app.collection.request("save")[0])
            self.wait_state("error")
        self.app._maybe_rotate_task()
        self.assertIs(self.app.plant, failed_plant)
        self.assertTrue(self.app.collection.physics_paused)
        self.assertFalse(self.app.executor.mailbox.enabled)


if __name__ == "__main__":
    unittest.main()
