"""Completed saves and discards start fresh Home scenes under the selected task policy."""
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

    def test_repeat_task_preserves_selection_with_reproducible_new_seeds(self):
        from simulation.scene import EpisodeTasks, build_selected_scene

        for scene, task in ((None, "bottles/toss_in_bin"), ("bottles", "toss_in_bin")):
            with self.subTest(scene=scene, task=task):
                sequence = EpisodeTasks(scene, task, 7, repeat_task=True)
                episodes = [sequence.next() for _ in range(5)]
                self.assertEqual(episodes[0], (scene, task, 7))
                self.assertEqual({(s, t) for s, t, _ in episodes}, {(scene, task)})
                replay = EpisodeTasks(scene, task, 7, repeat_task=True)
                self.assertEqual([replay.next() for _ in episodes], episodes)
                manifests = [build_selected_scene(*episode).manifest() for episode in episodes]
                self.assertTrue(all(manifest["scene"] == "bottles" and manifest["task"] == "toss_in_bin"
                                    for manifest in manifests))
                self.assertNotEqual(manifests[0]["table"], manifests[1]["table"])

    def test_repeat_task_requires_an_explicit_task(self):
        from simulation.scene import EpisodeTasks

        for scene in (None, "bottles", "hardware_free"):
            with self.subTest(scene=scene):
                with self.assertRaisesRegex(ValueError, "--repeat-task requires an explicit --task"):
                    EpisodeTasks(scene, None, 7, repeat_task=True)


@unittest.skipUnless(ROS_AVAILABLE, "run inside ros-jazzy with the local interface overlay")
class RandomTaskEpisodeTests(unittest.TestCase):
    def setUp(self):
        self.app = self.make_app()
        self.sequence = 0

    def make_app(self, *, scene="cups", task=None, table_distance=None, repeat_task=False):
        from description.model_builder import config_root
        from simulation.ros_viewer import RosViewerApp

        directory = tempfile.TemporaryDirectory(prefix="spd-task-episode-test-")
        self.addCleanup(directory.cleanup)
        app = RosViewerApp(argparse.Namespace(
            collection_config=config_root() / "collect_sim.yaml", output=Path(directory.name),
            max_frames=0, scene=scene, task=task, seed=7, table_distance=table_distance,
            headless=True, repeat_task=repeat_task,
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
        self.app.three_key.key("r")
        end = time.monotonic() + 10
        while self.app.three_key.stage != "recording":
            self.assertLess(time.monotonic(), end, self.app.collection.snapshot())
            self.publish()
            self.app.three_key.poll()
            self.app.collection.poll()
            time.sleep(.002)
        for _ in range(16):
            self.app.collection.tick(self.app.plant.physics_tick())

    def finish_and_advance(self, key="r"):
        from data_collector.recorder import validate_episode_path

        app = self.app
        previous = app.plant
        old_window = app.window
        old_generation = app._scene_generation
        partial = Path(app.collection.episode_path)
        app.three_key.key("s")
        self.assertEqual(app.three_key.stage, "paused")
        app.three_key.key(key)
        self.wait_state("idle")
        saved = app.collection.last_saved_path
        if key == "r":
            self.assertTrue(validate_episode_path(saved)["success"])
        else:
            self.assertFalse(partial.exists())
        self.assertIs(app.plant, previous)
        app.joint_control("r")  # Disk-stage input is not deferred into the new scene.
        app.three_key.poll()
        self.assertIsNot(app.plant, previous)
        self.assertTrue(previous._closed)
        np.testing.assert_array_equal(app.plant.joint_command_positions(), app.home_targets)
        np.testing.assert_array_equal(app.plant.joint_command_targets(), app.home_targets)
        np.testing.assert_array_equal(app.plant.data.qvel, 0)
        app._actions.put((old_generation, "r"))
        old_window.on_key("r")
        app._process_actions()
        self.assertEqual(app.three_key.stage, "idle")
        self.assertEqual(app.collection.state, "idle")
        self.assertTrue(app.collection.physics_paused)
        self.assertFalse(app.executor.mailbox.enabled)
        self.assertFalse(app.collection.request("start")[0])
        self.assertIsNone(app.collection.snapshot()["checkpoint_frames"])
        self.assertIsNone(app.collection.snapshot()["auto_checkpoint_frames"])
        self.assertEqual(app.collection.task_manifest["seed"], app.args.seed)
        with self.assertRaises(RuntimeError):
            app.next_task_after_episode()  # Each completion advances exactly once.
        return saved

    def test_save_and_discard_each_advance_to_fresh_random_layout(self):
        from simulation.scene import EpisodeTasks, build_selected_scene

        expected = EpisodeTasks("cups", None, 7)
        expected.next()
        self.start()
        saved = self.finish_and_advance()
        self.assertEqual((self.app.args.scene, self.app.args.task, self.app.args.seed), expected.next())
        replay = build_selected_scene(self.app.args.scene, self.app.args.task, self.app.args.seed)
        self.assertEqual(self.app.plant.scene_manifest["table"], replay.manifest()["table"])
        self.start()
        self.finish_and_advance("d")
        self.assertEqual((self.app.args.scene, self.app.args.task, self.app.args.seed), expected.next())
        replay = build_selected_scene(self.app.args.scene, self.app.args.task, self.app.args.seed)
        self.assertEqual(self.app.plant.scene_manifest, replay.manifest())
        self.assertTrue(Path(saved).is_file())
        self.start()
        self.finish_and_advance()
        self.assertEqual((self.app.args.scene, self.app.args.task, self.app.args.seed), expected.next())

    def test_repeat_task_after_save_and_discard_keeps_bottles_and_refreshes_layout(self):
        from simulation.scene import build_selected_scene

        self.app.close()
        self.app = self.make_app(scene=None, task="bottles/toss_in_bin", repeat_task=True)
        previous_seed = self.app.args.seed
        previous_table = self.app.plant.scene_manifest["table"]
        for key in ("r", "d", "r"):
            with self.subTest(completion=key):
                self.start()
                self.finish_and_advance(key)
                self.assertEqual((self.app.args.scene, self.app.args.task), ("bottles", "toss_in_bin"))
                self.assertNotEqual(self.app.args.seed, previous_seed)
                self.assertNotEqual(self.app.plant.scene_manifest["table"], previous_table)
                expected = build_selected_scene("bottles", "toss_in_bin", self.app.args.seed)
                self.assertEqual(self.app.plant.scene_manifest, expected.manifest())
                previous_seed = self.app.args.seed
                previous_table = self.app.plant.scene_manifest["table"]

    def test_failed_save_retains_scene_and_freezes(self):
        self.start()
        failed_plant = self.app.plant
        self.app.three_key.key("s")
        with patch.object(self.app.collection.recorder, "finish_episode", side_effect=OSError("disk unavailable")):
            self.app.three_key.key("r")
            self.wait_state("error")
            self.app.three_key.poll()
        self.assertEqual(self.app.three_key.stage, "error")
        self.assertIs(self.app.plant, failed_plant)
        self.assertTrue(self.app.collection.physics_paused)
        self.assertFalse(self.app.executor.mailbox.enabled)

    def test_failed_discard_preserves_scene_and_partial(self):
        self.start()
        previous = self.app.plant
        partial = Path(self.app.collection.episode_path)
        self.app.three_key.key("s")
        with patch.object(self.app.collection.recorder, "discard_episode", side_effect=OSError("disk unavailable")):
            self.app.three_key.key("d")
            self.wait_state("error")
            self.app.three_key.poll()
        self.assertIs(self.app.plant, previous)
        self.assertTrue(partial.is_file())
        self.assertEqual(self.app.collection.completed_episodes, 0)
        self.assertEqual(self.app.three_key.stage, "error")
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
                self.finish_and_advance()
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
        self.app.three_key.key("s")
        previous = self.app.plant
        with self.assertRaises(RuntimeError):
            self.app.next_task_after_episode()
        entered, release = threading.Event(), threading.Event()

        def delayed_validation(*args, **kwargs):
            entered.set()
            if not release.wait(10):
                raise TimeoutError("validation not released")
            return validate_episode_path(*args, **kwargs)

        with patch("data_collector.recorder.validate_episode_path", delayed_validation):
            self.app.three_key.key("r")
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
                with self.assertRaises(RuntimeError):
                    self.app.next_task_after_episode()
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
        self.app.three_key.key("s")
        with patch("data_collector.recorder.validate_episode_path", side_effect=ValueError("invalid trajectory")):
            self.app.three_key.key("r")
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
        self.app.three_key.key("r")
        self.wait_state("idle")
        previous = self.app.plant
        replacement = self.app._create_plant(None)
        with patch.object(self.app, "_create_plant", return_value=replacement), patch(
            "simulation.ros_viewer.RosJointCommandExecutor", side_effect=RuntimeError("executor startup failed")
        ):
            with self.assertRaisesRegex(RuntimeError, "executor startup failed"):
                self.app.next_task_after_episode()
        self.assertIs(self.app.plant, previous)
        self.assertFalse(previous._closed)
        self.assertTrue(replacement._closed)
        self.assertFalse(self.app.executor.mailbox.enabled)


if __name__ == "__main__":
    unittest.main()
