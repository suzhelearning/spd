"""Material-aware contacts preserve geometry and change actual solver behavior."""
import copy
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np

from _spd_native import material_forward, material_step
from simulation.scene import build_selected_scene


# IDs follow the portable model policy: unknown, wood, PE, glaze, raw, iron,
# woven polyester, silicone. Expected values are independent of the producer.
APPROVED_PAIRS = (
    (1, 1, .4), (2, 2, .2), (1, 2, .3), (1, 3, .3), (1, 4, .4),
    (1, 5, .4), (2, 3, .2), (2, 4, .25), (2, 5, .2), (3, 3, .25),
    (3, 4, .35), (4, 4, .5), (3, 5, .25), (4, 5, .35), (5, 5, .3),
    (6, 1, .5), (6, 2, .3), (6, 3, .35), (6, 4, .45), (6, 5, .4),
    (7, 1, .8), (7, 2, .8), (7, 3, .7), (7, 4, .8), (7, 5, .7),
    (7, 6, .8),
)


class MaterialFrictionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Use the real scene producer's matrix, not a test-only policy.
        scene = build_selected_scene(None, "jenga/hollow_tower", 0)
        cls.scene_xml = scene.xml_string()
        cls.object_names = tuple(obj.name for obj in scene.objects[:2])
        root = ET.fromstring(cls.scene_xml)
        cls.policy = root.find("custom/numeric[@name='spd_material_friction']")
        if cls.policy is None:
            raise AssertionError("scene does not carry its material policy")

    def model(self, worldbody, *, policy=True, gravity="0 0 0"):
        root = ET.fromstring(f'''<mujoco>
          <option timestep="0.001" integrator="implicitfast" cone="elliptic"
                  noslip_iterations="1" gravity="{gravity}"/>
          <size nuser_geom="4"/>
          <default><geom friction="0.17 0.003 0.0004" condim="6"
                         solref="0.004 1" solimp="0.95 0.99 0.001 0.5 2"/></default>
          <worldbody>{worldbody}</worldbody>
        </mujoco>''')
        if policy:
            ET.SubElement(root, "custom").append(copy.deepcopy(self.policy))
        return mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))

    def touching(self, material, other=1, *, region=0, direction=(0, -1, 0),
                 rotated=False, reverse=False, name="surface", policy=True):
        # Rotate the BODY around world X; the sphere geom has a separate local
        # rotation, so classification must use body axes, not geom axes.
        rotation = (np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])
                    if rotated else np.eye(3))
        position = .019 * (rotation @ np.asarray(direction))
        quat = "0.7071067811865476 0.7071067811865476 0 0" if rotated else "1 0 0 0"
        surface = f'''<body name="{name}" quat="{quat}">
          <geom name="surface" type="sphere" size="0.01"
                quat="0.7071067811865476 0 0 0.7071067811865476"
                user="11 22 {material} {region}"/>
        </body>'''
        probe = f'''<body name="probe" pos="{' '.join(map(str, position))}">
          <freejoint/><geom name="probe" type="sphere" size="0.01"
                            mass="0.1" user="33 44 {other} 0"/>
        </body>'''
        model = self.model(probe + surface if reverse else surface + probe, policy=policy)
        return model, mujoco.MjData(model)

    def assert_contact_mu(self, model, data, expected):
        material_forward(model, data)
        self.assertGreater(data.ncon, 0)
        for contact in data.contact:
            np.testing.assert_allclose(contact.friction[:2], [expected, expected], atol=1e-12)
        return data.contact[0]

    def test_all_approved_pairs_are_symmetric_on_real_contacts(self):
        for first, second, expected in APPROVED_PAIRS:
            for materials in {(first, second), (second, first)}:
                with self.subTest(materials=materials):
                    model, data = self.touching(*materials)
                    self.assert_contact_mu(model, data, expected)

    def test_palms_and_pads_only_cover_the_palmar_face_in_body_coordinates(self):
        observed_orders = set()
        for side, region, front in (("left", 2, -1), ("right", 3, 1)):
            for part in ("palm", "pad"):
                for face in (-1, 1):
                    for rotated in (False, True):
                        for reverse in (False, True):
                            with self.subTest(side=side, part=part, face=face,
                                              rotated=rotated, reverse=reverse):
                                model, data = self.touching(
                                    7, region=region, direction=(0, face, 0),
                                    rotated=rotated, reverse=reverse, name=f"{side}_{part}")
                                contact = self.assert_contact_mu(
                                    model, data, .8 if face == front else .17)
                                observed_orders.add(int(contact.geom[0]) == model.geom("surface").id)
        self.assertEqual(observed_orders, {False, True})

    def test_distal_tips_cover_all_faces_on_both_sides(self):
        for side in ("left", "right"):
            for axis in range(3):
                for sign in (-1, 1):
                    direction = np.zeros(3)
                    direction[axis] = sign
                    with self.subTest(side=side, axis=axis, sign=sign):
                        model, data = self.touching(
                            7, direction=direction, rotated=True, name=f"{side}_distal")
                        self.assert_contact_mu(model, data, .8)

    def test_ceramic_bottom_is_raw_but_sides_and_top_are_glazed(self):
        for direction, expected in (((0, 0, -1), .4), ((0, 0, 1), .3),
                                    ((1, 0, 0), .3), ((0, 1, 0), .3)):
            for rotated in (False, True):
                for reverse in (False, True):
                    with self.subTest(direction=direction, rotated=rotated, reverse=reverse):
                        model, data = self.touching(
                            3, region=1, direction=direction, rotated=rotated, reverse=reverse)
                        self.assert_contact_mu(model, data, expected)

    def test_unknown_and_unapproved_pairs_and_old_models_keep_stock_behavior(self):
        for first, second in ((0, 7), (1, 0), (6, 6), (7, 7)):
            with self.subTest(materials=(first, second)):
                model, data = self.touching(first, second)
                self.assert_contact_mu(model, data, .17)
        model, native = self.touching(7, 1, policy=False)
        stock = mujoco.MjData(model)
        native.qvel[0] = stock.qvel[0] = .2
        material_forward(model, native)
        mujoco.mj_forward(model, stock)
        np.testing.assert_array_equal(native.qacc, stock.qacc)
        for _ in range(10):
            material_step(model, native)
            mujoco.mj_step(model, stock)
        np.testing.assert_array_equal(native.qpos, stock.qpos)
        np.testing.assert_array_equal(native.qvel, stock.qvel)
        np.testing.assert_array_equal(native.qacc, stock.qacc)

    def test_policy_changes_friction_without_contact_topology_or_normal_changes(self):
        updated = mujoco.MjModel.from_xml_string(self.scene_xml)
        before, after = mujoco.MjData(updated), mujoco.MjData(updated)
        first, second = (updated.body(name).id for name in self.object_names)
        a1, a2 = (int(updated.jnt_qposadr[updated.body_jntadr[body]])
                  for body in (first, second))
        table = updated.geom("scene_table").id
        before.qpos[a1 + 2] = updated.geom_pos[table, 2] + updated.geom_size[table, 2] + .0065
        before.qpos[a2:a2 + 7] = before.qpos[a1:a1 + 7]
        before.qpos[a2 + 2] += .014
        after.qpos[:] = before.qpos
        mujoco.mj_forward(updated, before)
        material_forward(updated, after)
        self.assertGreater(before.ncon, 0)
        self.assertEqual(before.ncon, after.ncon)
        for original, contact in zip(before.contact, after.contact):
            for field in ("geom", "pos", "frame", "dist", "dim", "solref", "solimp"):
                np.testing.assert_array_equal(getattr(contact, field), getattr(original, field))
            np.testing.assert_array_equal(contact.friction[2:], original.friction[2:])
        object_contacts = [c for c in after.contact
                           if set(updated.geom_bodyid[c.geom]) == {first, second}]
        table_contacts = [c for c in after.contact
                          if table in c.geom and first in updated.geom_bodyid[c.geom]]
        self.assertTrue(object_contacts)
        self.assertTrue(table_contacts)
        for contact in object_contacts:
            np.testing.assert_allclose(contact.friction[:2], [.4, .4], atol=1e-12)
        for contact in table_contacts:
            np.testing.assert_allclose(contact.friction[:2], [.5, .5], atol=1e-12)

    def sliding_model(self):
        return self.model('''<geom name="floor" type="plane" size="1 1 0.1" user="0 0 2 0"/>
          <body name="slider" pos="0 0 0.05"><freejoint/>
            <geom name="slider" type="box" size="0.05 0.05 0.05" mass="1" user="1 1 7 0"/>
          </body>''', gravity="0 0 -9.81")

    def test_material_step_changes_sliding_motion_not_just_contact_arrays(self):
        model = self.sliding_model()
        native, stock = mujoco.MjData(model), mujoco.MjData(model)
        native.qvel[0] = stock.qvel[0] = 1.
        for _ in range(150):
            material_step(model, native)
            mujoco.mj_step(model, stock)
        self.assertTrue(np.isfinite(native.qpos).all())
        self.assertTrue(np.isfinite(native.qvel).all())
        self.assertLess(native.qpos[0] + .025, stock.qpos[0])
        self.assertLess(native.qvel[0], .3)
        self.assertGreater(stock.qvel[0], .5)

    def test_mjb_carries_policy_and_surface_regions_without_scene_source(self):
        for model, initial in (self.touching(3, region=1, direction=(0, 0, -1), rotated=True),
                               self.touching(7, region=3, direction=(0, 1, 0), reverse=True)):
            with self.subTest(region=int(model.geom_user[model.geom("surface").id, 3])):
                with tempfile.TemporaryDirectory() as directory:
                    path = Path(directory) / "scene.mjb"
                    mujoco.mj_saveModel(model, str(path), None)
                    restored = mujoco.MjModel.from_binary_path(str(path))
                np.testing.assert_array_equal(restored.numeric_data, model.numeric_data)
                np.testing.assert_array_equal(restored.geom_user, model.geom_user)
                loaded = mujoco.MjData(restored)
                loaded.qpos[:] = initial.qpos
                loaded.qvel[:] = initial.qvel
                material_forward(model, initial)
                material_forward(restored, loaded)
                np.testing.assert_array_equal(loaded.contact.friction, initial.contact.friction)
                np.testing.assert_array_equal(loaded.qacc, initial.qacc)
                for _ in range(10):
                    material_step(model, initial)
                    material_step(restored, loaded)
                np.testing.assert_array_equal(loaded.qpos, initial.qpos)
                np.testing.assert_array_equal(loaded.qvel, initial.qvel)


if __name__ == "__main__":
    unittest.main()
