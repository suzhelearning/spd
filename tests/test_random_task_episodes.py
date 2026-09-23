"""Explicit Home boundaries rebuild tasks without teleporting the physical robot."""
import argparse
import importlib.util
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch

import numpy as np

try:
    ROS_AVAILABLE = (importlib.util.find_spec("rclpy") is not None
                     and importlib.util.find_spec("tianji_spd_interfaces.msg") is not None)
except ModuleNotFoundError:
    ROS_AVAILABLE = False


@unittest.skipUnless(ROS_AVAILABLE, "run inside ros-jazzy with the local interface overlay")
class RandomTaskEpisodeTests(unittest.TestCase):
    def setUp(self):
        self.app = self.make_app()
        self.sequence = 0

    def make_app(self, *, scene="cups", task=None):
        from description.model_builder import config_root
        from simulation.ros_viewer import RosViewerApp

        directory = tempfile.TemporaryDirectory(prefix="spd-task-episode-test-")
        self.addCleanup(directory.cleanup)
        app = RosViewerApp(argparse.Namespace(
            collection_config=config_root() / "collect_sim.yaml", output=Path(directory.name),
            max_frames=0, scene=scene, task=task, seed=7, table_distance=None, headless=True,
        ))
        self.addCleanup(app.close)
        return app

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

    def assert_home_boundary(self, *, saved):
        app = self.app
        old = app.plant
        coordinator = app.three_key
        robot = old.joint_command_positions().copy(), old.joint_command_velocities().copy()
        targets = old.joint_command_targets().copy()
        app.begin_home_return(saved)
        prep = app.plant
        self.assertIsNot(prep, old)
        self.assertIsNone(prep.scene_manifest)
        self.assertTrue(app.preparation_scene)
        np.testing.assert_array_equal(prep.joint_command_positions(), robot[0])
        np.testing.assert_array_equal(prep.joint_command_velocities(), robot[1])
        np.testing.assert_array_equal(prep.joint_command_targets(), targets)
        self.assertTrue(old._closed)
        app.recording_control("checkpoint")  # Never authorize a newly replaced scene.
        app.complete_home_return()
        self.assertIsNot(app.plant, prep)
        self.assertIs(app.three_key, coordinator)
        self.assertFalse(app.preparation_scene)
        self.assertTrue(prep._closed)
        np.testing.assert_array_equal(app.plant.joint_command_positions(), robot[0])
        np.testing.assert_array_equal(app.plant.joint_command_velocities(), robot[1])
        np.testing.assert_array_equal(app.plant.joint_command_targets(), targets)
        app._process_actions()
        self.assertFalse(app.executor.mailbox.enabled)
        self.assertEqual(app.collection.state, "idle")
        self.assertIsNone(app.collection.snapshot()["checkpoint_frames"])
        self.assertIsNone(app.collection.snapshot()["auto_checkpoint_frames"])
        self.assertFalse(app.collection.request("start")[0])
        self.assertEqual(app.collection.task_manifest["seed"], app.args.seed)

    def test_save_advances_only_after_home_and_discard_rebuilds_identical_seed(self):
        from data_collector.recorder import validate_episode_path
        from simulation.scene import EpisodeTasks, build_selected_scene

        expected = EpisodeTasks("cups", None, 7)
        expected.next()
        self.start()
        first = self.app.plant
        self.assertTrue(self.app.collection.request("pause")[0])
        self.assertIs(self.app.plant, first)
        self.assertTrue(self.app.collection.request("save")[0])
        self.wait_state("idle")
        saved = self.app.collection.last_saved_path
        self.assertTrue(validate_episode_path(saved)["success"])
        self.assertIs(self.app.plant, first)  # A counter increment is not a scene-switch command.
        self.assert_home_boundary(saved=True)
        self.assertEqual((self.app.args.scene, self.app.args.task, self.app.args.seed), expected.next())
        replay = build_selected_scene(self.app.args.scene, self.app.args.task, self.app.args.seed)
        self.assertEqual(self.app.plant.scene_manifest["table"], replay.manifest()["table"])
        original_spec = self.app.args.scene, self.app.args.task, self.app.args.seed
        original_scene = self.app.plant.scene_manifest
        self.start()
        self.assertTrue(self.app.collection.request("discard")[0])
        self.wait_state("idle")
        self.assert_home_boundary(saved=False)
        self.assertEqual((self.app.args.scene, self.app.args.task, self.app.args.seed), original_spec)
        self.assertEqual(self.app.plant.scene_manifest, original_scene)
        self.assertTrue(Path(saved).is_file())
        self.start()
        self.assertTrue(self.app.collection.request("save")[0])
        self.wait_state("idle")
        self.assert_home_boundary(saved=True)
        self.assertEqual((self.app.args.scene, self.app.args.task, self.app.args.seed), expected.next())

    def test_failed_save_retains_scene_and_freezes(self):
        self.start()
        failed_plant = self.app.plant
        self.assertTrue(self.app.collection.request("pause")[0])
        with patch.object(self.app.collection.recorder, "finish_episode", side_effect=OSError("disk unavailable")):
            self.assertTrue(self.app.collection.request("save")[0])
            self.wait_state("error")
        self.assertIs(self.app.plant, failed_plant)
        self.assertTrue(self.app.collection.physics_paused)
        self.assertFalse(self.app.executor.mailbox.enabled)

    def test_fixed_and_hardware_free_tasks_rebuild_fresh_destinations(self):
        task = self.app.args.task
        self.app.close()
        for scene, selected_task in (("cups", task), ("hardware_free", None)):
            with self.subTest(scene=scene):
                self.app = self.make_app(scene=scene, task=selected_task)
                spec = self.app.args.scene, self.app.args.task, self.app.args.seed
                manifest = self.app.plant.scene_manifest
                self.assert_home_boundary(saved=True)
                self.assertEqual((self.app.args.scene, self.app.args.task, self.app.args.seed), spec)
                self.assertEqual(self.app.plant.scene_manifest, manifest)
                self.app.close()

    def test_failed_replacement_keeps_live_scene_and_closes_new_plant(self):
        previous = self.app.plant
        replacement = self.app._create_plant(None)
        with patch.object(self.app, "_create_plant", return_value=replacement), patch(
            "simulation.ros_viewer.RosJointCommandExecutor", side_effect=RuntimeError("executor startup failed")
        ):
            with self.assertRaisesRegex(RuntimeError, "executor startup failed"):
                self.app.begin_home_return(False)
        self.assertIs(self.app.plant, previous)
        self.assertFalse(previous._closed)
        self.assertTrue(replacement._closed)
        self.assertFalse(self.app.executor.mailbox.enabled)


if __name__ == "__main__":
    unittest.main()
