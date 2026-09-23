"""A loaded articulated drawer must move physically, not lock on its guide shelf."""
import unittest

import numpy as np

from simulation.scene import build_selected_scene
from simulation.viewer import PlantController


class DrawerAffordanceTests(unittest.TestCase):
    def test_loaded_drawer_closes_and_reopens_with_blocks_retained(self):
        scene = build_selected_scene(None, 'spelling_blocks/sort_and_unload', 0)
        plant = PlantController(scene_result=scene)
        self.addCleanup(plant.close)
        drawer = [obj for obj in scene.objects if obj.class_name == 'drawer'][1]
        info = scene.sampled_values['affordances'][str(drawer.instance_id)]
        blocks = [obj for obj in scene.objects if obj.instance_id in info['contained_instance_ids']]
        joint = plant.model.joint(info['joint_name'])
        qpos, dof = int(joint.qposadr[0]), int(joint.dofadr[0])
        initial_positions = np.array([plant.data.body(obj.name).xpos.copy() for obj in blocks])
        checkpoint = plant.capture_checkpoint()
        for force, expected in ((-6., 0.), (6., .25)):
            plant.data.qfrc_applied[dof] = force
            for _ in range(480):
                self.assertTrue(plant.physics_tick().finite)
            self.assertAlmostEqual(float(plant.data.qpos[qpos]), expected, delta=.002)
            origin = plant.data.body(drawer.name).xpos
            rotation = plant.data.body(drawer.name).xmat.reshape(3, 3)
            positions = np.array([plant.data.body(obj.name).xpos.copy() for obj in blocks])
            local = (positions - origin) @ rotation
            self.assertTrue(np.all(np.abs(local[:, 0]) < drawer.size[0] / 2))
            self.assertTrue(np.all(np.abs(local[:, 1]) < drawer.size[1] / 2))
            self.assertTrue(np.all((local[:, 2] > 0) & (local[:, 2] < drawer.size[2])))
            if expected == 0:
                self.assertTrue(np.all(np.linalg.norm(positions - initial_positions, axis=1) > .08))
        plant.restore_checkpoint(checkpoint)
        self.assertEqual(float(plant.data.qpos[qpos]), info['initial_opening_m'])
        np.testing.assert_array_equal(
            np.array([plant.data.body(obj.name).xpos for obj in blocks]), initial_positions,
        )


if __name__ == '__main__':
    unittest.main()
