"""Validated saves start fresh random Home scenes; discards retain Home motion."""
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


class EpisodeTaskSequenceTests(unittest.TestCase):
    def test_startup_selection_does_not_restrict_later_catalog_or_seed(self):
        from simulation.scene import EpisodeTasks
        from spd_envs.registry import TASKS

        catalog = {(spec.scene, spec.name) for spec in TASKS}
        for scene, task in (("cups", "pyramid"), ("cups", None), ("hardware_free", None)):
            with self.subTest(scene=scene, task=task):
                sequence = EpisodeTasks(scene, task, 7)
                first = sequence.next()
                if task is not None or scene == "hardware_free":
                    self.assertEqual(first, (scene, task, 7))
                else:
                    self.assertEqual(first[0], scene)
                following = [sequence.next() for _ in range(256)]
                self.assertEqual({(scene, task) for scene, task, _ in following}, catalog)
                self.assertNotEqual({seed for _, _, seed in following}, {first[2]})
                replay = EpisodeTasks(scene, task, 7)
                self.assertEqual([replay.next() for _ in range(257)], [first, *following])


@unittest.skipUnless(ROS_AVAILABLE, "run inside ros-jazzy with the local interface overlay")
class RandomTaskEpisodeTests(unittest.TestCase):
    def setUp(self):
        self.app = self.make_app()
        self.sequence = 0

    def make_app(self, *, scene="cups", task=None, table_distance=None):
        from description.model_builder import config_root
        from simulation.ros_viewer import RosViewerApp

        directory = tempfile.TemporaryDirectory(prefix="spd-task-episode-test-")
        self.addCleanup(directory.cleanup)
        app = RosViewerApp(argparse.Namespace(
            collection_config=config_root() / "collect_sim.yaml", output=Path(directory.name),
            max_frames=0, scene=scene, task=task, seed=7, table_distance=table_distance, headless=True,
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

    def assert_discard_home_boundary(self):
        app = self.app
        old = app.plant
        coordinator = app.three_key
        robot = old.joint_command_positions().copy(), old.joint_command_velocities().copy()
        targets = old.joint_command_targets().copy()
        app.begin_home_return()
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

    def save_and_advance(self):
        from data_collector.recorder import validate_episode_path

        app = self.app
        previous = app.plant
        app.three_key.key("s")
        self.wait_state("idle")
        saved = app.collection.last_saved_path
        self.assertTrue(validate_episode_path(saved)["success"])
        self.assertIs(app.plant, previous)  # Disk completion precedes the coordinator switch.
        app.recording_control("checkpoint")
        app.three_key.poll()
        self.assertIsNot(app.plant, previous)
        self.assertTrue(previous._closed)
        self.assertFalse(app.preparation_scene)
        np.testing.assert_array_equal(app.plant.joint_command_positions(), app.home_targets)
        np.testing.assert_array_equal(app.plant.joint_command_targets(), app.home_targets)
        np.testing.assert_array_equal(app.plant.data.qvel, 0)
        app._process_actions()
        self.assertEqual(app.three_key.stage, "idle")
        self.assertEqual(app.collection.state, "idle")
        self.assertTrue(app.collection.physics_paused)
        self.assertFalse(app.executor.mailbox.enabled)
        self.assertFalse(app.collection.request("start")[0])
        self.assertIsNone(app.collection.snapshot()["checkpoint_frames"])
        self.assertIsNone(app.collection.snapshot()["auto_checkpoint_frames"])
        self.assertEqual(app.collection.task_manifest["seed"], app.args.seed)
        return saved

    def test_save_advances_directly_and_discard_rebuilds_identical_seed(self):
        from simulation.scene import EpisodeTasks, build_selected_scene

        expected = EpisodeTasks("cups", None, 7)
        expected.next()
        self.start()
        saved = self.save_and_advance()
        self.assertEqual((self.app.args.scene, self.app.args.task, self.app.args.seed), expected.next())
        replay = build_selected_scene(self.app.args.scene, self.app.args.task, self.app.args.seed)
        self.assertEqual(self.app.plant.scene_manifest["table"], replay.manifest()["table"])
        original_spec = self.app.args.scene, self.app.args.task, self.app.args.seed
        original_scene = self.app.plant.scene_manifest
        self.start()
        self.assertTrue(self.app.collection.request("discard")[0])
        self.wait_state("idle")
        self.assert_discard_home_boundary()
        self.assertEqual((self.app.args.scene, self.app.args.task, self.app.args.seed), original_spec)
        self.assertEqual(self.app.plant.scene_manifest, original_scene)
        self.assertTrue(Path(saved).is_file())
        self.start()
        self.save_and_advance()
        self.assertEqual((self.app.args.scene, self.app.args.task, self.app.args.seed), expected.next())

    def test_failed_save_retains_scene_and_freezes(self):
        self.start()
        failed_plant = self.app.plant
        with patch.object(self.app.collection.recorder, "finish_episode", side_effect=OSError("disk unavailable")):
            self.app.three_key.key("s")
            self.wait_state("error")
            self.app.three_key.poll()
        self.assertEqual(self.app.three_key.stage, "error")
        self.assertIs(self.app.plant, failed_plant)
        self.assertTrue(self.app.collection.physics_paused)
        self.assertFalse(self.app.executor.mailbox.enabled)

    def test_explicit_startup_task_and_table_override_expire_after_save(self):
        from simulation.scene import EpisodeTasks, build_selected_scene

        task = self.app.args.task
        self.app.close()
        for scene, selected_task in (("cups", task), ("hardware_free", None)):
            with self.subTest(scene=scene):
                self.app = self.make_app(scene=scene, task=selected_task, table_distance=.23)
                expected = EpisodeTasks(scene, selected_task, 7)
                expected.next()
                self.start()
                self.save_and_advance()
                spec = self.app.args.scene, self.app.args.task, self.app.args.seed
                self.assertEqual(spec, expected.next())
                replay = build_selected_scene(*spec)
                self.assertEqual(self.app.plant.scene_manifest, replay.manifest())
                self.assertNotEqual(self.app.args.table_distance, .23)
                self.app.close()

    def test_no_next_task_until_writer_closes_and_validation_succeeds(self):
        import threading
        from data_collector.recorder import validate_episode_path

        self.start()
        previous = self.app.plant
        with self.assertRaisesRegex(RuntimeError, "newly saved"):
            self.app.next_task_after_save()
        entered, release = threading.Event(), threading.Event()

        def delayed_validation(*args, **kwargs):
            entered.set()
            if not release.wait(10):
                raise TimeoutError("validation not released")
            return validate_episode_path(*args, **kwargs)

        with patch("data_collector.recorder.validate_episode_path", delayed_validation):
            self.app.three_key.key("s")
            try:
                self.assertTrue(entered.wait(10))
                for _ in range(5):
                    self.app.collection.poll()
                    self.app.three_key.poll()
                self.assertEqual(self.app.three_key.stage, "saving")
                self.assertIs(self.app.plant, previous)
                self.assertEqual(self.app.collection.last_saved_path, "")
                self.assertTrue(self.app.collection.physics_paused)
                self.assertFalse(self.app.executor.mailbox.enabled)
                with self.assertRaisesRegex(RuntimeError, "newly saved"):
                    self.app.next_task_after_save()
            finally:
                release.set()
            self.wait_state("idle")
        self.app.three_key.poll()
        self.assertIsNot(self.app.plant, previous)
        self.assertEqual(self.app.three_key.stage, "idle")
        self.assertTrue(validate_episode_path(self.app.collection.last_saved_path)["success"])

    def test_validation_failure_preserves_original_scene(self):
        self.start()
        previous = self.app.plant
        with patch("data_collector.recorder.validate_episode_path", side_effect=ValueError("invalid trajectory")):
            self.app.three_key.key("s")
            self.wait_state("error")
            self.app.three_key.poll()
        self.assertIs(self.app.plant, previous)
        self.assertEqual(self.app.three_key.stage, "error")
        self.assertEqual(self.app.collection.last_saved_path, "")
        self.assertTrue(self.app.collection.physics_paused)
        self.assertFalse(self.app.executor.mailbox.enabled)

    def test_failed_replacement_keeps_live_scene_and_closes_new_plant(self):
        self.start()
        self.app.three_key.key("s")
        self.wait_state("idle")
        previous = self.app.plant
        replacement = self.app._create_plant(None)
        with patch.object(self.app, "_create_plant", return_value=replacement), patch(
            "simulation.ros_viewer.RosJointCommandExecutor", side_effect=RuntimeError("executor startup failed")
        ):
            with self.assertRaisesRegex(RuntimeError, "executor startup failed"):
                self.app.next_task_after_save()
        self.assertIs(self.app.plant, previous)
        self.assertFalse(previous._closed)
        self.assertTrue(replacement._closed)
        self.assertFalse(self.app.executor.mailbox.enabled)


if __name__ == "__main__":
    unittest.main()
