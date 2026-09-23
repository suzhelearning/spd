"""Live-target blend regressions using the real plant and deterministic clocks."""
from types import SimpleNamespace
import unittest

import numpy as np

from _spd_native import RosJointCommandExecutor
from interfaces.ros_joint_command import JOINT_NAME_TUPLE, JointCommandSnapshot, VALID_READY_MASK
from simulation.viewer import PlantController


class ExecutorTransitionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.plant = PlantController()
        cls.initial = cls.plant.capture_checkpoint()

    @classmethod
    def tearDownClass(cls):
        cls.plant.close()

    def setUp(self):
        self.plant.restore_checkpoint(self.initial)
        self.now = 10_000_000_000
        self.utc_base = 1_700_000_000_000_000_000
        self.sequence = 0
        self.session = "transition-test"
        self.executor = RosJointCommandExecutor(
            self.plant, subscribe=False,
            clock_ns=lambda: self.utc, monotonic_ns=lambda: self.now,
        )
        self.addCleanup(self.executor.close)
        self.origin = self.plant.joint_command_positions().copy()
        entries = {entry.joint: entry for entry in self.plant.joints}
        limits = np.asarray([entries[name].range for name in JOINT_NAME_TUPLE])
        self.target = limits[:, 0] + .35 * (limits[:, 1] - limits[:, 0])
        self.moving_target = limits[:, 0] + .65 * (limits[:, 1] - limits[:, 0])

    @property
    def utc(self):
        return self.utc_base + self.now

    def receive(self, values=None, *, ready=VALID_READY_MASK, session=None, age_ns=0):
        self.sequence += 1
        stamp = self.utc - age_ns
        message = SimpleNamespace(
            schema_version=1, robot_config="tianji_wuji2_v1",
            session_id=self.session if session is None else session,
            sequence=self.sequence, ready_mask=ready,
            joint_names=JOINT_NAME_TUPLE,
            position_rad=tuple(self.target if values is None else values),
            stamp=SimpleNamespace(sec=stamp // 1_000_000_000, nanosec=stamp % 1_000_000_000),
        )
        return self.executor.mailbox.receive(message, now_ns=self.utc)

    def apply(self, elapsed_ns=0):
        self.now += elapsed_ns
        return self.executor.apply_pending(now_ns=self.now)

    def assert_held(self, reference):
        self.assertFalse(self.executor.transition_active)
        self.assertFalse(self.executor.transition_frame)
        self.assertFalse(self.executor.mailbox.enabled)
        self.assertEqual(self.executor.hold_mask, VALID_READY_MASK)
        self.assertEqual(self.plant.hold_mask, VALID_READY_MASK)
        np.testing.assert_array_equal(self.plant.joint_command_targets(), reference)

    def test_large_gap_blends_actual_positions_to_moving_endpoint_every_tick(self):
        self.assertTrue(self.receive())
        self.assertFalse(self.executor.authorize(True))
        # Retained targets deliberately differ from actual qpos: a resume must
        # begin at actual positions, not at these previous actuator references.
        self.plant.submit_joint_command(JointCommandSnapshot.from_values(
            session_id=self.session, sequence=1, ready_mask=7,
            position_rad=self.moving_target, stamp_ns=self.utc,
        ))
        qpos, qvel = self.plant.data.qpos.copy(), self.plant.data.qvel.copy()
        self.assertTrue(self.executor.authorize_transition())
        first = self.apply()
        np.testing.assert_array_equal(first.position_rad, self.origin)
        self.assertTrue(self.executor.transition_active)
        self.assertTrue(self.executor.transition_frame)

        self.now += 500_000_000
        self.assertTrue(self.receive(self.moving_target))
        halfway = self.apply()
        np.testing.assert_allclose(halfway.position_rad, (self.origin + self.moving_target) / 2)
        between_packets = self.apply(50_000_000)
        weight = .55 ** 3 * (10 - 15 * .55 + 6 * .55 ** 2)
        np.testing.assert_allclose(between_packets.position_rad,
                                   self.origin + weight * (self.moving_target - self.origin))
        self.assertEqual(between_packets.snapshot.sequence, halfway.snapshot.sequence)

        self.now += 450_000_000
        self.assertTrue(self.receive(self.target))
        endpoint = self.apply()
        np.testing.assert_array_equal(endpoint.position_rad, self.target)
        self.assertFalse(self.executor.transition_active)
        self.assertTrue(self.executor.transition_frame)
        np.testing.assert_array_equal(self.plant.data.qpos, qpos)
        np.testing.assert_array_equal(self.plant.data.qvel, qvel)
        self.assertIsNone(self.apply(1_000_000))
        self.assertFalse(self.executor.transition_frame)
        self.assertTrue(self.receive(self.moving_target))
        np.testing.assert_array_equal(self.apply().position_rad, self.moving_target)

    def test_preparation_delay_uses_current_source_and_starts_timer_on_first_apply(self):
        self.assertTrue(self.receive())
        self.assertTrue(self.executor.authorize_transition())
        self.now += 2_000_000_000
        self.assertTrue(self.receive(self.moving_target))
        first = self.apply()
        np.testing.assert_array_equal(first.position_rad, self.origin)
        self.assertEqual(first.hold_mask, 0)
        next_frame = self.apply(50_000_000)
        weight = .05 ** 3 * (10 - 15 * .05 + 6 * .05 ** 2)
        np.testing.assert_allclose(next_frame.position_rad,
                                   self.origin + weight * (self.moving_target - self.origin))
        self.assertEqual(next_frame.hold_mask, 0)
        self.assertTrue(self.executor.transition_active)

    def test_stale_source_before_or_during_blend_revokes_without_phantom_resume(self):
        for apply_first in (False, True):
            with self.subTest(apply_first=apply_first):
                self.executor.clear()
                self.assertTrue(self.receive())
                self.assertTrue(self.executor.authorize_transition())
                if apply_first:
                    self.apply()
                reference = self.plant.joint_command_targets()
                self.assertIsNone(self.apply(100_000_001))
                self.assert_held(reference)
                self.assertTrue(self.receive(self.moving_target))
                self.assertIsNone(self.apply())
                self.assert_held(reference)

    def test_clear_discards_elapsed_progress_and_resamples_actual_pose(self):
        self.assertTrue(self.receive())
        self.assertTrue(self.executor.authorize_transition())
        self.apply()
        self.now += 500_000_000
        self.assertTrue(self.receive())
        self.apply()
        reference = self.plant.joint_command_targets()
        self.executor.clear()
        self.assertIsNone(self.apply())
        self.assert_held(reference)
        self.now += 2_000_000_000
        self.assertTrue(self.receive(self.moving_target))
        self.assertTrue(self.executor.authorize_transition())
        np.testing.assert_array_equal(self.apply().position_rad, self.origin)
        self.assertTrue(self.executor.transition_active)

    def test_session_change_cancels_even_if_original_session_returns_before_tick(self):
        self.assertTrue(self.receive())
        self.assertTrue(self.executor.authorize_transition())
        self.apply()
        reference = self.plant.joint_command_targets()
        self.assertTrue(self.receive(session="other-session"))
        self.assertTrue(self.receive())
        self.assertIsNone(self.apply())
        self.assert_held(reference)

    def test_ready_groups_cannot_leave_or_join_even_between_physics_ticks(self):
        for initial_mask, changed_mask in ((7, 3), (1, 7)):
            with self.subTest(initial=initial_mask, changed=changed_mask):
                self.executor.clear()
                self.assertTrue(self.receive(ready=initial_mask))
                self.assertTrue(self.executor.authorize_transition())
                self.apply()
                reference = self.plant.joint_command_targets()
                self.assertTrue(self.receive(ready=changed_mask))
                self.assertTrue(self.receive(ready=initial_mask))
                self.assertIsNone(self.apply())
                self.assert_held(reference)

    def test_invalid_candidate_cancels_instead_of_reusing_previous_endpoint(self):
        self.assertTrue(self.receive())
        self.assertTrue(self.executor.authorize_transition())
        self.apply()
        reference = self.plant.joint_command_targets()
        invalid = self.target.copy()
        invalid[14] = 100.0
        self.assertFalse(self.receive(invalid))
        self.assertTrue(self.receive())
        self.assertIsNone(self.apply())
        self.assert_held(reference)

    def test_disable_then_standard_authorization_cannot_reuse_transition(self):
        self.assertTrue(self.receive())
        self.assertTrue(self.executor.authorize_transition())
        self.apply()
        self.assertTrue(self.executor.authorize(False))
        reference = self.plant.joint_command_targets()
        self.assertIsNone(self.apply())
        self.assert_held(reference)
        self.assertTrue(self.receive(reference))
        self.assertTrue(self.executor.authorize(True))
        self.assertFalse(self.executor.transition_active)
        np.testing.assert_array_equal(self.apply().position_rad, reference)
        self.assertFalse(self.executor.transition_frame)

    def test_soft_limit_overshoot_has_legal_entry_reference_without_teleporting(self):
        joint = self.plant.model.joint(JOINT_NAME_TUPLE[14])
        address = int(joint.qposadr[0])
        lower = float(joint.range[0])
        self.plant.data.qpos[address] = lower - .001
        actual = self.plant.data.qpos.copy()
        self.assertTrue(self.receive())
        self.assertTrue(self.executor.authorize_transition())
        first = self.apply()
        self.assertEqual(first.position_rad[14], lower)
        np.testing.assert_array_equal(self.plant.data.qpos, actual)
        self.now += 1_000_000_000
        self.assertTrue(self.receive())
        np.testing.assert_array_equal(self.apply().position_rad, self.target)
        np.testing.assert_array_equal(self.plant.data.qpos, actual)
        self.executor.clear()
        self.plant.data.qpos[address] = np.nan
        self.assertTrue(self.receive())
        self.assertFalse(self.executor.authorize_transition())
        self.assertFalse(self.executor.mailbox.enabled)

    def test_transition_authorization_still_requires_fresh_ready_legal_candidate(self):
        self.assertFalse(self.executor.authorize_transition())
        self.assertTrue(self.receive(ready=0))
        self.assertFalse(self.executor.authorize_transition())
        self.assertTrue(self.receive())
        self.now += 100_000_001
        self.assertFalse(self.executor.authorize_transition())
        self.assertFalse(self.receive(np.full(54, np.nan)))
        self.assertFalse(self.executor.authorize_transition())


if __name__ == "__main__":
    unittest.main()
