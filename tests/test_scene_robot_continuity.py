"""Robot continuity across real procedural models without ROS or a renderer."""
import unittest
from unittest.mock import patch

import mujoco
import numpy as np

from interfaces.ros_joint_command import JOINT_NAME_TUPLE, JointCommandSnapshot, VALID_READY_MASK
from simulation.scene import build_selected_scene
from simulation.viewer import PlantController


class SceneRobotContinuityTests(unittest.TestCase):
    def setUp(self):
        self.previous = PlantController(
            scene_result=build_selected_scene(None, 'cups/pyramid', 0, 0.10),
        )
        self.addCleanup(self.previous.close)
        self.scene = build_selected_scene(None, 'mugs/hang_mug', 1, 0.18)
        self.current = PlantController(scene_result=self.scene)
        self.addCleanup(self.current.close)
        entries = {entry.joint: entry for entry in self.previous.joints}
        limits = np.asarray([entries[name].range for name in JOINT_NAME_TUPLE])
        self.targets = limits[:, 0] + np.linspace(0.35, 0.65, 54) * np.diff(limits, axis=1)[:, 0]
        self.previous.submit_joint_command(self.command(self.targets))
        for _ in range(3):
            self.previous.physics_tick()
        # A retained command can legitimately be newer than the last physics ctrl.
        self.targets = 0.99 * self.targets + 0.01 * limits[:, 0]
        self.previous.submit_joint_command(self.command(self.targets, sequence=2))

    @staticmethod
    def command(targets, sequence=1):
        return JointCommandSnapshot.from_values(
            session_id='continuity', sequence=sequence, stamp_ns=sequence,
            ready_mask=VALID_READY_MASK, position_rad=targets,
        )

    @staticmethod
    def state(plant):
        spec = mujoco.mjtState.mjSTATE_INTEGRATION
        state = np.empty(mujoco.mj_stateSize(plant.model, spec))
        mujoco.mj_getState(plant.model, plant.data, state, spec)
        return (
            state, plant.data.qacc.copy(), plant.data.xpos.copy(),
            plant.joint_command_targets(), np.asarray([plant.tick, plant.hold_mask]),
        )

    def assert_state_equal(self, plant, expected):
        for actual, before in zip(self.state(plant), expected):
            np.testing.assert_array_equal(actual, before)

    def assert_rejected(self, source=None, error=ValueError):
        source = self.previous if source is None else source
        before_current, before_source = self.state(self.current), self.state(source)
        with self.assertRaises(error):
            self.current.inherit_robot_state(source)
        self.assert_state_equal(self.current, before_current)
        self.assert_state_equal(source, before_source)

    def test_transfers_robot_only_and_retains_new_scene_initial_state(self):
        old, new = self.previous, self.current
        self.assertNotEqual(old.model.nq, new.model.nq)
        self.assertNotEqual(old.model.nv, new.model.nv)
        before_old = self.state(old)
        before_qpos, before_qvel = new.data.qpos.copy(), new.data.qvel.copy()
        before_objects = {obj.name: new.data.body(obj.name).xpos.copy() for obj in self.scene.objects}
        before_fixtures = new.model.body_pos.copy()
        joint_ids = [mujoco.mj_name2id(new.model, mujoco.mjtObj.mjOBJ_JOINT, name)
                     for name in JOINT_NAME_TUPLE]
        qpos_ids = new.model.jnt_qposadr[joint_ids]
        dof_ids = new.model.jnt_dofadr[joint_ids]
        scene_qpos = np.ones(new.model.nq, dtype=bool)
        scene_qpos[qpos_ids] = False
        scene_dof = np.ones(new.model.nv, dtype=bool)
        scene_dof[dof_ids] = False
        self.assertFalse(np.array_equal(old.joint_command_positions(), new.joint_command_positions()))
        self.assertFalse(np.array_equal(old.joint_command_positions(), self.targets))
        self.assertTrue(np.any(old.joint_command_velocities()))

        new.inherit_robot_state(old)

        np.testing.assert_array_equal(new.joint_command_positions(), old.joint_command_positions())
        np.testing.assert_array_equal(new.joint_command_velocities(), old.joint_command_velocities())
        np.testing.assert_array_equal(new.joint_command_targets(), self.targets)
        entries = {entry.joint: entry for entry in new.joints}
        actuator_ids = [mujoco.mj_name2id(new.model, mujoco.mjtObj.mjOBJ_ACTUATOR, entries[name].actuator)
                        for name in JOINT_NAME_TUPLE]
        np.testing.assert_array_equal(new.data.ctrl[actuator_ids], self.targets)
        np.testing.assert_array_equal(new.data.qpos[scene_qpos], before_qpos[scene_qpos])
        np.testing.assert_array_equal(new.data.qvel[scene_dof], before_qvel[scene_dof])
        np.testing.assert_array_equal(new.model.body_pos, before_fixtures)
        for name, position in before_objects.items():
            np.testing.assert_array_equal(new.data.body(name).xpos, position)
        self.assertEqual((new.tick, new.sim_time_ns, new.hold_mask), (0, 0, VALID_READY_MASK))
        # A fresh forward computation must agree with the transferred data's kinematics.
        forwarded = mujoco.MjData(new.model)
        forwarded.qpos[:] = new.data.qpos
        forwarded.qvel[:] = new.data.qvel
        forwarded.ctrl[:] = new.data.ctrl
        mujoco.mj_forward(new.model, forwarded)
        np.testing.assert_array_equal(new.data.xpos, forwarded.xpos)
        self.assert_state_equal(old, before_old)
        self.assert_rejected()  # Even a tick-zero transfer cannot be repeated.
        new.physics_tick()
        self.assertEqual(new.tick, 1)
        np.testing.assert_array_equal(new.joint_command_targets(), self.targets)
        self.assertEqual(new.hold_mask, VALID_READY_MASK)

    def test_rejects_bad_sources_atomically_and_keeps_destination_usable(self):
        self.assert_rejected(self.current)
        for plant in (self.previous, self.current):
            with self.subTest(closed=plant is self.previous):
                with patch.object(plant, '_closed', True):
                    self.assert_rejected(error=RuntimeError)
        for artifact_hash in ('different-base-artifact', 'unverified', 'unknown'):
            with self.subTest(artifact_hash=artifact_hash):
                with patch.object(self.previous, 'artifact_hash', artifact_hash):
                    self.assert_rejected()
        mutations = (
            (self.previous.data.qpos, 0, np.nan),
            (self.previous.data.qvel, 0, np.inf),
            (self.previous.data.ctrl, 0, np.inf),
            (self.current.data.qpos, -1, np.nan),
        )
        for array, index, value in mutations:
            with self.subTest(value=value):
                saved = array[index]
                try:
                    array[index] = value
                    self.assert_rejected()
                finally:
                    array[index] = saved
        self.current.inherit_robot_state(self.previous)
        np.testing.assert_array_equal(self.current.joint_command_targets(), self.targets)

    def test_rejects_changed_or_unsupported_actuator_semantics_atomically(self):
        model = self.current.model
        actuator = self.current._command_actuators[0]
        for array, index, value in (
            (model.actuator_gear, (actuator, 0), 2),
            (model.actuator_dyntype, actuator, mujoco.mjtDyn.mjDYN_FILTER),
            (model.actuator_trnid, (actuator, 0), self.current._command_joints[1]),
            (model.actuator_forcerange, (actuator, 1), model.actuator_forcerange[actuator, 1] + 1),
        ):
            saved = array[index]
            try:
                array[index] = value
                self.assert_rejected()
            finally:
                array[index] = saved
        self.current.inherit_robot_state(self.previous)
        np.testing.assert_array_equal(self.current.joint_command_positions(), self.previous.joint_command_positions())

    def test_checkpoint_restore_does_not_make_used_destination_fresh(self):
        initial = self.current.capture_checkpoint()
        self.current.physics_tick()
        self.assert_rejected()
        self.current.restore_checkpoint(initial)
        self.assertEqual(self.current.tick, 0)
        self.assert_rejected()

    def test_commanded_tick_zero_destination_is_not_fresh(self):
        self.current.submit_joint_command(self.command(self.current.joint_command_targets()))
        self.assertEqual(self.current.tick, 0)
        self.assert_rejected()


if __name__ == '__main__':
    unittest.main()
