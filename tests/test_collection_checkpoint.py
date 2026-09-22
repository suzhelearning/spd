"""Physical checkpoint and contact-gate regressions; no ROS or renderer required."""
import tempfile
import unittest
from pathlib import Path

import mujoco
import numpy as np

from data_collector.recorder import EpisodeRecorder, validate_episode_path
from data_collector.trajectory import TrajectorySource
from simulation.scene import build_selected_scene
from simulation.viewer import PlantController


class CollectionCheckpointTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.plant = PlantController(scene_result=build_selected_scene(None, 'cups/pyramid', 0, 0.10))
        cls.source = TrajectorySource(cls.plant, {'task': 'pyramid'})
        cls.initial = cls.plant.capture_checkpoint()

    @classmethod
    def tearDownClass(cls):
        cls.plant.close()

    def setUp(self):
        self.plant.restore_checkpoint(self.initial)
        self.source.reset_contacts()

    def test_exact_continuation_and_fresh_contact_gate(self):
        plant, source = self.plant, self.source
        for _ in range(13):
            plant.physics_tick()
        snapshot = plant.capture_checkpoint()
        for _ in range(17):
            plant.physics_tick()
        expected_qpos = plant.data.qpos.copy()
        expected_qvel = plant.data.qvel.copy()
        plant.restore_checkpoint(snapshot)
        for _ in range(17):
            plant.physics_tick()
        np.testing.assert_array_equal(plant.data.qpos, expected_qpos)
        np.testing.assert_array_equal(plant.data.qvel, expected_qvel)
        plant.restore_checkpoint(snapshot)
        self.assertFalse(source.has_hand_object_contact())
        model, data = plant.model, plant.data
        body = source.metadata['object_body_ids'][0]
        joint = int(model.body_jntadr[body])
        adr = int(model.jnt_qposadr[joint])
        hand_geom = next(g for g in source.metadata['hand_geom_ids'][0] if model.geom_contype[g])
        object_geom = next(g for g in source.metadata['object_geom_ids'][0] if model.geom_contype[g])
        # Move an actual cup collision geom onto an actual hand collision geom.
        mujoco.mj_forward(model, data)
        data.qpos[adr:adr+3] += data.geom_xpos[hand_geom] - data.geom_xpos[object_geom]
        before = data.qpos.copy(), data.qacc_warmstart.copy(), data.time
        self.assertTrue(source.has_hand_object_contact())
        np.testing.assert_array_equal(data.qpos, before[0])
        np.testing.assert_array_equal(data.qacc_warmstart, before[1])
        self.assertEqual(data.time, before[2])
        mujoco.mj_forward(model, data)
        source.observe_contacts()
        interval = source.capture_contact_state()
        source.reset_contacts()
        source.restore_contact_state(interval)
        self.assertTrue(source.capture(plant.tick, 1)['hand_contact'][0])
        plant.restore_checkpoint(snapshot)
        # The old contact interval and cached contacts must not reject a clear pose.
        self.assertFalse(source.has_hand_object_contact())

    def test_truncation_preserves_prefix_and_replaces_branch_boundaries(self):
        source, plant = self.source, self.plant
        frames = []
        for i in range(5):
            for _ in range(8):
                plant.physics_tick()
                source.observe_contacts()
            frames.append(source.capture(plant.tick, (i+1)*100))
        with tempfile.TemporaryDirectory() as directory:
            recorder = EpisodeRecorder(Path(directory), queue_size=16)
            try:
                recorder.start_episode('rewind', {'task': 'pyramid'}, model_bytes=source.model_bytes,
                                       metadata=source.metadata)
                for frame in frames:
                    recorder.append_frame(frame)
                with self.assertRaises(ValueError):
                    recorder.truncate_frames(6)
                recorder.truncate_frames(3)
                recorder.append_frame({**frames[3], 'monotonic_ns': np.int64(1000)})
                recorder.truncate_frames(2)
                recorder.append_frame({**frames[2], 'monotonic_ns': np.int64(2000)})
                result = recorder.finish_episode()
                self.assertEqual(validate_episode_path(result)['frames'], 3)
                import h5py
                with h5py.File(result) as handle:
                    np.testing.assert_array_equal(handle['trajectory/qpos'][:2], [f['qpos'] for f in frames[:2]])
                    np.testing.assert_array_equal(handle['trajectory/tick'][:], [8, 16, 24])
                    np.testing.assert_array_equal(handle['trajectory/monotonic_ns'][:], [100, 200, 2000])
                    np.testing.assert_array_equal(handle['collection_events/rewind']['frame_count'], [2])
            finally:
                recorder.close()


if __name__ == '__main__':
    unittest.main()
