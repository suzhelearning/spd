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


class BottleAffordanceTests(unittest.TestCase):
    def test_two_free_bottles_keep_seeded_positions_and_shape_variation(self):
        scenes = [build_selected_scene(None, 'bottles/toss_in_bin', seed)
                  for seed in range(4)]
        bottle_sets = []
        for scene in scenes:
            selected = [obj for obj in scene.objects if obj.class_name == 'bottle']
            self.assertEqual(len(selected), 2, f'seed={scene.seed}')
            self.assertEqual(sum(obj.class_name == 'bin' for obj in scene.objects), 1)
            bin_object = next(obj for obj in scene.objects if obj.class_name == 'bin')
            self.assertLessEqual(.4, bin_object.position[0])
            self.assertLessEqual(bin_object.position[0], .6)
            bottle_sets.append(selected)
            plant = PlantController(scene_result=scene)
            try:
                for bottle in selected:
                    body = plant.model.body(bottle.name)
                    self.assertEqual(int(plant.model.body_jntnum[body.id]), 1)
                    joint_id = int(plant.model.body_jntadr[body.id])
                    self.assertEqual(int(plant.model.jnt_type[joint_id]), 0)  # MuJoCo free joint.
                    self.assertAlmostEqual(bottle.position[2], scene.manifest()['table']['top_z_m'])
            finally:
                plant.close()
        self.assertNotEqual(
            {bottle.position[:2] for bottle in bottle_sets[0]},
            {bottle.position[:2] for bottle in bottle_sets[1]},
        )
        self.assertNotEqual(
            {bottle.yaw_rad for bottle in bottle_sets[0]},
            {bottle.yaw_rad for bottle in bottle_sets[1]},
        )
        self.assertNotEqual(
            {bottle.size for bottle in bottle_sets[0]},
            {bottle.size for bottle in bottle_sets[1]},
        )
        self.assertNotEqual(
            {bottle.asset_id for bottles in bottle_sets for bottle in bottles},
            {bottle_sets[0][0].asset_id},
        )
        repeated = build_selected_scene(None, 'bottles/toss_in_bin', 0)
        self.assertEqual(repeated.manifest(), scenes[0].manifest())


if __name__ == '__main__':
    unittest.main()
