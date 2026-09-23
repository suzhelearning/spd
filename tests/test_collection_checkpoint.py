"""Exact physical checkpoints, annotation boundaries and failure preservation."""
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import h5py
import mujoco
import numpy as np

from data_collector.config import CollectionConfig
from data_collector.session import CollectionSession
from data_collector.recorder import EpisodeRecorder, validate_episode_path
from data_collector.replay import replay_episode
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
                for frame, label, flags in zip(frames, (1, 0, 2, 2, 0), (1, 2, 4, 8, 16)):
                    recorder.append_frame(frame, recovery=label, control_flags=flags)
                with self.assertRaises(ValueError):
                    recorder.truncate_frames(6)
                recorder.truncate_frames(3)
                recorder.append_frame({**frames[3], 'monotonic_ns': np.int64(1000)}, recovery=3)
                recorder.truncate_frames(2)
                recorder.append_frame({**frames[2], 'monotonic_ns': np.int64(2000)}, recovery=4, control_flags=24)
                result = recorder.finish_episode()
                self.assertEqual(validate_episode_path(result)['frames'], 3)
                with h5py.File(result) as handle:
                    np.testing.assert_array_equal(handle['trajectory/qpos'][:2], [f['qpos'] for f in frames[:2]])
                    np.testing.assert_array_equal(handle['trajectory/tick'][:], [8, 16, 24])
                    np.testing.assert_array_equal(handle['trajectory/monotonic_ns'][:], [100, 200, 2000])
                    np.testing.assert_array_equal(handle['collection_events/rewind']['frame_count'], [2])
                    np.testing.assert_array_equal(handle['collection_events/recovery_transition'][:], [1, 0, 4])
                    np.testing.assert_array_equal(handle['collection_events/control_flags'][:], [1, 2, 24])
                recorder.start_episode('rewind-failed', {'task': 'pyramid'}, model_bytes=source.model_bytes,
                                       metadata=source.metadata)
                for frame in frames:
                    recorder.append_frame(frame, control_flags=16)
                resize = h5py.Dataset.resize

                def fail_flag_shrink(dataset, size, axis=None):
                    if dataset.name == '/collection_events/control_flags' and size == 3 and dataset.shape[0] > size:
                        raise OSError('truncate storage failed')
                    return resize(dataset, size, axis=axis)

                with patch.object(h5py.Dataset, 'resize', fail_flag_shrink):
                    with self.assertRaises(RuntimeError):
                        recorder.truncate_frames(3)
                recorder.abort_episode('truncate storage failed')
                with self.assertRaises(ValueError):
                    validate_episode_path(recorder.partial_path)
                with h5py.File(recorder.partial_path) as handle:
                    np.testing.assert_array_equal(handle['trajectory/qpos'][:3], [f['qpos'] for f in frames[:3]])
                    np.testing.assert_array_equal(handle['collection_events/control_flags'][:3], [16] * 3)
                    self.assertFalse(handle.attrs['complete'])
                    self.assertFalse(handle.attrs['success'])
            finally:
                recorder.close()

    def test_start_checkpoint_restores_contacting_scene_and_repeated_zero_rewinds(self):
        plant = self.plant
        # Checkpoint zero and a manual checkpoint are both usable during a grasp.
        data, model = plant.data, plant.model
        metadata = self.source.metadata
        body = metadata['object_body_ids'][0]
        adr = int(model.jnt_qposadr[int(model.body_jntadr[body])])
        hand_geom = next(g for g in metadata['hand_geom_ids'][0] if model.geom_contype[g])
        object_geom = next(g for g in metadata['object_geom_ids'][0] if model.geom_contype[g])
        mujoco.mj_forward(model, data)
        data.qpos[adr:adr + 3] += data.geom_xpos[hand_geom] - data.geom_xpos[object_geom]
        mujoco.mj_forward(model, data)
        state_spec = mujoco.mjtState.mjSTATE_INTEGRATION
        original = np.empty(mujoco.mj_stateSize(model, state_spec))
        mujoco.mj_getState(model, data, original, state_spec)
        targets = plant.joint_command_targets()
        candidate = SimpleNamespace(ready_mask=1, stamp_ns=time.time_ns())
        mailbox = SimpleNamespace(enabled=True, latest=candidate)
        executor = SimpleNamespace(mailbox=mailbox, hold_mask=0,
                                   clear=lambda: setattr(mailbox, 'enabled', False))
        with tempfile.TemporaryDirectory() as directory:
            session = CollectionSession(CollectionConfig(2, Path(directory), 60, 64, 0),
                                        plant, executor, {'task': 'pyramid'})
            try:
                self.assertTrue(session.source.has_hand_object_contact())
                candidate.stamp_ns = time.time_ns()
                accepted, _ = session.request('start')
                self.assertTrue(accepted)
                self.assertTrue(session.physics_paused)
                self.assertEqual(session.snapshot()['checkpoint_frames'], 0)
                np.testing.assert_array_equal(session.checkpoint_targets, targets)
                with self.assertRaises(ValueError):
                    session.checkpoint_targets[0] = 0
                with self.assertRaises(ValueError):
                    session.checkpoint_targets.flags.writeable = True
                session._job.result(timeout=10)
                candidate.stamp_ns = time.time_ns()
                session.poll()
                self.assertEqual(session.state, 'recording')
                self.assertTrue(session.request('checkpoint')[0])
                for label in (1, 2):
                    for _ in range(9):
                        session.tick(plant.physics_tick(), recovery=label)
                    self.assertEqual(session.state_frames, 2)
                    session.capture_auto_checkpoint()
                    self.assertTrue(session.request('pause')[0])
                    self.assertTrue(session.request('revert')[0])
                    session._job.result(timeout=10)
                    session.poll()
                    self.assertEqual(session.state, 'paused')
                    self.assertEqual(session.state_frames, 0)
                    self.assertIsNone(session.snapshot()['auto_checkpoint_frames'])
                    self.assertEqual(plant.tick, 0)
                    restored = np.empty_like(original)
                    mujoco.mj_getState(model, data, restored, state_spec)
                    np.testing.assert_array_equal(restored, original)
                    np.testing.assert_array_equal(plant.joint_command_targets(), targets)
                    mailbox.enabled = True
                    candidate.stamp_ns = time.time_ns()
                    self.assertTrue(session.request('resume')[0])
                session.tick(plant.physics_tick(), recovery=3)
                for index in range(8):
                    session.tick(plant.physics_tick(), recovery=4 if index == 1 else 0,
                                 control_flags=2 if index == 1 else 16 if index == 2 else 0)
                    if index == 2:
                        session.capture_auto_checkpoint()
                self.assertEqual(session.snapshot()['checkpoint_frames'], 0)
                self.assertEqual(session.snapshot()['auto_checkpoint_frames'], 1)
                self.assertIsNotNone(session.auto_checkpoint_targets)
                self.assertTrue(session.request('save')[0])
                self.assertTrue(session.physics_paused)
                session._job.result(timeout=10)
                session.poll()
                self.assertEqual(session.last_outcome, 'saved')
                self.assertEqual(session.completed_episodes, 1)
                result = replay_episode(session.last_saved_path)
                self.assertTrue(result['recovery_annotated'])
                self.assertTrue(result['success'])
                self.assertEqual(result['restored_frames'], 2)
                with h5py.File(session.last_saved_path) as handle:
                    np.testing.assert_array_equal(handle['trajectory/tick'][:], [1, 9])
                    np.testing.assert_array_equal(handle['collection_events/recovery_transition'][:], [3, 4])
                    np.testing.assert_array_equal(handle['collection_events/control_flags'][:], [0, 18])
            finally:
                session.close()

    def test_recovery_validation_distinguishes_legacy_and_rejects_bad_annotations(self):
        for _ in range(8):
            self.plant.physics_tick()
            self.source.observe_contacts()
        frame = self.source.capture(self.plant.tick, 100)
        with tempfile.TemporaryDirectory() as directory:
            recorder = EpisodeRecorder(Path(directory))
            try:
                recorder.start_episode('labels', {'task': 'pyramid'}, model_bytes=self.source.model_bytes,
                                       metadata=self.source.metadata)
                recorder.append_frame(frame, recovery=4, control_flags=31)
                result = recorder.finish_episode()
                self.assertTrue(validate_episode_path(result)['recovery_annotated'])
                self.assertTrue(validate_episode_path(result)['control_flags_annotated'])
                invalid = (
                    np.array([5], dtype=np.uint8),
                    np.array([2], dtype=np.int64),
                    np.array([[2]], dtype=np.uint8),
                    np.array([], dtype=np.uint8),
                )
                for labels in invalid:
                    with self.subTest(dtype=labels.dtype, shape=labels.shape, values=labels):
                        with h5py.File(result, 'r+') as handle:
                            del handle['collection_events/recovery_transition']
                            handle['collection_events'].create_dataset('recovery_transition', data=labels)
                        with self.assertRaises(ValueError):
                            validate_episode_path(result)
                with h5py.File(result, 'r+') as handle:
                    del handle['collection_events/recovery_transition']
                    handle['collection_events'].create_dataset(
                        'rewind', data=np.array([(0, 10)], dtype=[('frame_count', '<i8'), ('monotonic_ns', '<i8')]))
                self.assertFalse(validate_episode_path(result)['recovery_annotated'])
                with h5py.File(result, 'r+') as handle:
                    del handle['collection_events']
                self.assertFalse(replay_episode(result)['recovery_annotated'])
                self.assertFalse(replay_episode(result)['control_flags_annotated'])
            finally:
                recorder.close()

    def test_failed_label_write_rolls_back_entire_row_and_preserves_partial(self):
        frames = []
        for i in range(2):
            for _ in range(8):
                self.plant.physics_tick()
                self.source.observe_contacts()
            frames.append(self.source.capture(self.plant.tick, (i + 1) * 100))
        setitem = h5py.Dataset.__setitem__

        def fail_second_label(dataset, key, value):
            if dataset.name == '/collection_events/control_flags' and key == 1:
                raise OSError('label storage failed')
            return setitem(dataset, key, value)

        with tempfile.TemporaryDirectory() as directory:
            recorder = EpisodeRecorder(Path(directory))
            try:
                recorder.start_episode('failed', {'task': 'pyramid'}, model_bytes=self.source.model_bytes,
                                       metadata=self.source.metadata)
                with patch.object(h5py.Dataset, '__setitem__', fail_second_label):
                    recorder.append_frame(frames[0], recovery=1, control_flags=2)
                    recorder.append_frame(frames[1], recovery=3, control_flags=16)
                    self.assertTrue(recorder._done.wait(timeout=10))
                with self.assertRaises(RuntimeError):
                    recorder.discard_episode()
                self.assertTrue(recorder.partial_path.is_file())
                recorder.abort_episode('label storage failed')
                report = validate_episode_path(recorder.partial_path, allow_partial=True)
                self.assertFalse(report['complete'])
                self.assertEqual(report['frames'], 1)
                with h5py.File(recorder.partial_path) as handle:
                    np.testing.assert_array_equal(handle['trajectory/qpos'][:], [frames[0]['qpos']])
                    np.testing.assert_array_equal(handle['collection_events/recovery_transition'][:], [1])
                    np.testing.assert_array_equal(handle['collection_events/control_flags'][:], [2])
            finally:
                recorder.close()

    def test_failed_save_freezes_and_preserves_without_completing(self):
        candidate = SimpleNamespace(ready_mask=1, stamp_ns=time.time_ns())
        mailbox = SimpleNamespace(enabled=True, latest=candidate)
        executor = SimpleNamespace(mailbox=mailbox, hold_mask=0,
                                   clear=lambda: setattr(mailbox, 'enabled', False))
        with tempfile.TemporaryDirectory() as directory:
            session = CollectionSession(CollectionConfig(2, Path(directory), 60, 64, 0),
                                        self.plant, executor, {'task': 'pyramid'})
            try:
                candidate.stamp_ns = time.time_ns()
                self.assertTrue(session.request('start')[0])
                session._job.result(timeout=10)
                candidate.stamp_ns = time.time_ns()
                session.poll()
                session.tick(self.plant.physics_tick())
                qpos, qvel = self.plant.data.qpos.copy(), self.plant.data.qvel.copy()
                with patch.object(session.recorder, 'finish_episode', side_effect=OSError('disk failure')):
                    self.assertTrue(session.request('save')[0])
                    self.assertTrue(session.physics_paused)
                    with self.assertRaises(OSError):
                        session._job.result(timeout=10)
                    session.poll()
                session._job.result(timeout=10)
                session.poll()
                self.assertEqual(session.state, 'error')
                self.assertTrue(session.physics_paused)
                self.assertEqual(session.last_outcome, '')
                self.assertEqual(session.completed_episodes, 0)
                self.assertEqual(session.last_saved_path, '')
                self.assertTrue(Path(session.episode_path).is_file())
                np.testing.assert_array_equal(self.plant.data.qpos, qpos)
                np.testing.assert_array_equal(self.plant.data.qvel, qvel)
                self.assertFalse(validate_episode_path(session.episode_path, allow_partial=True)['complete'])
            finally:
                session.close()


if __name__ == '__main__':
    unittest.main()
