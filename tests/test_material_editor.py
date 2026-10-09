"""Consumer regressions: real native physics/MJB with isolated editable config."""
import copy
import os
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import mujoco
import numpy as np

from _spd_native import Physics, material_forward
from data_collector.config import CollectionConfig
from data_collector.session import CollectionSession
from data_collector.trajectory import load_model
from interfaces.ros_joint_command import JOINT_NAMES
from simulation.material_editor import MaterialEditor
from spd_envs.physical_materials import (
    MATERIAL_IDS, PAIR_KEYS, load_material_coefficients, load_task_material_coefficients,
    material_manifest, material_numeric_values, model_material_pairs,
    save_material_coefficients, task_material_config_path,
)


class MaterialEditorTests(unittest.TestCase):
    def setUp(self):
        # Independent policy fixture: editable repository settings must not affect tests.
        self.coefficients = {pair: .6 for pair in PAIR_KEYS}
        directory = tempfile.TemporaryDirectory(prefix="spd-material-editor-")
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name)
        self.config_path = self.root / "friction.yaml"
        save_material_coefficients(self.coefficients, self.config_path)
        self.task_root = self.root / "tasks"
        environment = patch.dict(os.environ, {
            "SPD_MATERIAL_FRICTION_CONFIG": str(self.config_path),
            "SPD_TASK_MATERIAL_FRICTION_DIR": str(self.task_root),
        })
        environment.start()
        self.addCleanup(environment.stop)
        self.plant = self.make_plant()
        self.addCleanup(self.plant.physics.close)
        self.collection = CollectionSession(
            CollectionConfig(2, self.root / "episodes", 60, 8, 0), self.plant,
            SimpleNamespace(), self.task_manifest(self.plant, "test", "editor", 17),
        )
        self.addCleanup(self.collection.recorder.close)
        self.addCleanup(self.collection.close)
        self.app = SimpleNamespace(
            plant=self.plant, collection=self.collection, three_key=SimpleNamespace(stage="idle"),
            args=SimpleNamespace(scene="test", task="editor", seed=17),
            _task_manifest=self.task_manifest,
        )
        self.editor = MaterialEditor(self.app)

    @staticmethod
    def task_manifest(plant, scene, task, seed):
        return {**plant.scene_manifest, "scene": scene, "task": task, "seed": seed}

    def make_plant(self, *, policy=True, scene="test", task="editor",
                   materials=("wood", "polyethylene", "polyester_woven_fabric", "silicone")):
        # A small but complete canonical-54 model exercises the actual native
        # cached numeric pointer and the real collector snapshot contract.
        bodies = []
        for side, names in (("l", JOINT_NAMES[:27]), ("r", JOINT_NAMES[27:])):
            joints = "".join(
                f'<body name="joint_body_{index}_{side}" pos="{1 + index * .04} 0 0">'
                f'<joint name="{name}" type="hinge" range="-1 1"/>'
                '<geom type="sphere" size=".01" mass=".01" contype="0" conaffinity="0"/>'
                '</body>' for index, name in enumerate(names)
            )
            bodies.append(f'<body name="{side}_wrist">{joints}</body>')
        numeric = " ".join(str(value) for value in material_numeric_values(self.coefficients))
        custom = f'<custom><numeric name="spd_material_friction" data="{numeric}"/></custom>' if policy else ""
        actuators = "".join(f'<position name="servo_{index}" joint="{name}" kp="10" '
                            'ctrllimited="true" ctrlrange="-1 1"/>'
                            for index, name in enumerate(JOINT_NAMES))
        extra_geoms = "".join(
            f'<geom name="material_{name}" type="sphere" size=".01" pos="{2 + index} 0 0" '
            f'user="0 0 {MATERIAL_IDS[name]} 0"/>'
            for index, name in enumerate(materials)
        )
        load_task_material_coefficients(scene, task)
        profile = {"scene": scene, "task": task, "path": str(task_material_config_path(scene, task))}
        model = mujoco.MjModel.from_xml_string(
            '<mujoco><compiler angle="radian"/>'
            '<option timestep="0.0020833333333333333" gravity="0 0 0" integrator="implicitfast"/>'
            '<size nuser_geom="4"/><default><geom friction=".17 .003 .0004" condim="6"/></default>'
            f'{custom}<worldbody>{"".join(bodies)}'
            f'{extra_geoms}<geom name="surface" type="sphere" size=".01" user="0 0 1 0"/>'
            '<body name="probe" pos=".019 0 0"><freejoint/>'
            '<geom name="probe" type="sphere" size=".01" mass=".1" user="0 0 1 0"/>'
            f'</body></worldbody><actuator>{actuators}</actuator></mujoco>'
        )
        data = mujoco.MjData(model)
        joints = np.asarray([model.joint(name).id for name in JOINT_NAMES], dtype=np.int32)
        physics = Physics(model, data, model.jnt_qposadr[joints].copy(), model.jnt_dofadr[joints].copy(),
                          np.arange(54, dtype=np.int32), np.tile([-1., 1.], (54, 1)), np.zeros(54))
        data.time = .73
        data.qvel[-1] = .03
        material_forward(model, data)
        return SimpleNamespace(
            model=model, data=data, physics=physics, physics_hz=480,
            joint_command_targets=physics.targets, tick=0,
            scene_manifest={"scene": scene, "task": task, "objects": [{"name": "probe"}], "seed": 17,
                            "sampled_values": {"fixture": "unchanged"},
                            "physical_materials": {
                                **material_manifest(self.coefficients), "task_profile": profile,
                            }},
        )

    def open_and_change(self):
        self.assertTrue(self.editor.key("m"))
        self.assertTrue(self.editor.key("right"))

    def assert_mu(self, model, data, coefficient):
        material_forward(model, data)
        self.assertGreater(data.ncon, 0)
        np.testing.assert_allclose(data.contact.friction[:, :2], coefficient, atol=1e-12)

    def runtime_state(self):
        return (self.plant.data.qpos.copy(), self.plant.data.qvel.copy(), self.plant.data.time,
                self.plant.joint_command_targets().copy(), self.plant.physics.tick,
                self.plant.model.body_mass.copy(), self.plant.model.body_inertia.copy())

    def assert_runtime_state(self, before):
        after = self.runtime_state()
        for old, new in zip(before, after):
            np.testing.assert_array_equal(old, new)

    def test_apply_updates_real_contacts_cached_native_policy_and_owned_mjb(self):
        before = self.runtime_state()
        old_source = self.collection.source
        old_hash = old_source.metadata["model_sha256"]
        numeric_address = self.plant.model.numeric_data.ctypes.data
        original = self.coefficients[PAIR_KEYS[0]]
        self.open_and_change()
        self.assert_mu(self.plant.model, self.plant.data, original)
        self.editor.key("enter")
        expected = original + .05
        self.assert_runtime_state(before)
        self.assertEqual(self.plant.model.numeric_data.ctypes.data, numeric_address)
        self.assert_mu(self.plant.model, self.plant.data, expected)
        new_source = self.collection.source
        self.assertIsNot(old_source, new_source)
        self.assertNotEqual(old_hash, new_source.metadata["model_sha256"])
        self.assertEqual(load_task_material_coefficients("test", "editor")[PAIR_KEYS[0]], expected)
        self.assertEqual(load_material_coefficients(), self.coefficients)
        self.assertEqual(self.plant.scene_manifest["seed"], 17)
        self.assertEqual(self.plant.scene_manifest["sampled_values"], {"fixture": "unchanged"})
        policy = self.plant.scene_manifest["physical_materials"]
        self.assertEqual(self.collection.task_manifest["physical_materials"], policy)
        self.assertEqual(new_source.metadata["scene_manifest"]["physical_materials"], policy)
        self.assertEqual(new_source.metadata["task_manifest"], self.collection.task_manifest)
        for source, mu in ((old_source, original), (new_source, expected)):
            restored = load_model(source.model_bytes, source.metadata)
            data = mujoco.MjData(restored)
            data.qpos[:] = self.plant.data.qpos
            self.assert_mu(restored, data, mu)
        # Physics existed before editing and holds a native pointer into numeric.
        self.plant.physics.physics_tick()
        self.assertGreater(self.plant.data.ncon, 0)
        np.testing.assert_allclose(self.plant.data.contact.friction[:, :2], expected, atol=1e-12)
        self.assertIn("已应用并保存", " ".join(self.editor.hud().values()))
        numeric = self.plant.model.numeric_data
        approved = {(MATERIAL_IDS[a], MATERIAL_IDS[b]) for a, b in PAIR_KEYS}
        for i in range(8):
            for j in range(8):
                if (i, j) not in approved and (j, i) not in approved:
                    self.assertEqual(numeric[1 + i * 8 + j], -1)

    def test_drafts_are_independent_cancelled_and_have_no_upper_bound(self):
        old_config = self.config_path.read_bytes()
        old_numeric = self.plant.model.numeric_data.copy()
        self.open_and_change()
        self.editor.key("down")
        self.editor.key("left")
        self.assertAlmostEqual(self.editor._draft[PAIR_KEYS[0]], self.coefficients[PAIR_KEYS[0]] + .05)
        self.assertAlmostEqual(self.editor._draft[PAIR_KEYS[1]], self.coefficients[PAIR_KEYS[1]] - .05)
        for _ in range(50):
            self.editor.key("left")
        self.assertEqual(self.editor._draft[PAIR_KEYS[1]], 0)
        for _ in range(25):
            self.editor.key("right")
        self.assertEqual(self.editor._draft[PAIR_KEYS[1]], 1.25)
        self.editor.key("m")
        self.assertFalse(self.editor.opened)
        self.assertFalse(self.editor.key("r"))
        self.assertEqual(self.config_path.read_bytes(), old_config)
        np.testing.assert_array_equal(self.plant.model.numeric_data, old_numeric)
        self.editor.key("m")
        self.assertEqual(self.editor._draft, self.coefficients)
        self.editor.key("up")
        self.assertEqual(self.editor._selected, len(self.editor._pairs) - 1)
        self.editor.key("down")
        self.assertEqual(self.editor._selected, 0)

    def test_nonidle_and_active_episode_states_are_readonly_and_controls_isolated(self):
        self.editor.key("m")
        original = self.editor._draft.copy()
        cases = [(stage, state) for stage in (
            "recording", "paused", "auto_paused", "binding", "rebinding", "preparing",
            "saving", "discarding", "reverting", "error",
        ) for state in ("idle", "recording", "paused")]
        cases.extend(("idle", state) for state in ("recording", "paused", "preparing", "error"))
        with patch("simulation.material_editor.save_task_material_coefficients") as save:
            for stage, state in cases:
                with self.subTest(stage=stage, state=state):
                    self.app.three_key.stage, self.collection.state = stage, state
                    for key in ("right", "left", "enter", "r", "s", "d"):
                        self.assertTrue(self.editor.key(key))
                    self.assertEqual(self.editor._draft, original)
                    self.assertIn("只读", " ".join(self.editor.hud().values()))
                    selected = self.editor._selected
                    self.assertTrue(self.editor.key("down"))
                    self.assertEqual(self.editor._selected, (selected + 1) % len(self.editor._pairs))
                    self.assertEqual((self.app.three_key.stage, self.collection.state), (stage, state))
            save.assert_not_called()
        self.app.three_key.stage = self.collection.state = "idle"
        self.assertTrue(self.editor.key("down"))  # Selection is also allowed in readonly.
        self.assertFalse(self.editor.key("]"))

    def test_idle_busy_closed_or_pending_job_cannot_edit(self):
        self.editor.key("m")
        draft = self.editor._draft.copy()
        with patch("simulation.material_editor.save_task_material_coefficients") as save:
            self.collection._closed = True
            self.editor.key("right")
            self.editor.key("enter")
            self.collection._closed = False
            self.collection._job = object()
            self.editor.key("right")
            self.editor.key("enter")
            self.collection._job = None
            with patch.object(type(self.collection.recorder), "is_busy", new_callable=property,
                              fget=lambda recorder: True):
                self.editor.key("right")
                self.editor.key("enter")
            save.assert_not_called()
        self.assertEqual(self.editor._draft, draft)

    def test_atomic_save_failure_restores_policy_entire_data_and_snapshot(self):
        self.open_and_change()
        before = self.runtime_state()
        old_numeric = self.plant.model.numeric_data.copy()
        old_data = mujoco.MjData(self.plant.model)
        mujoco.mj_copyData(old_data, self.plant.model, self.plant.data)
        old_scene, old_manifest, old_source = (
            self.plant.scene_manifest, self.collection.task_manifest, self.collection.source,
        )
        old_config = self.config_path.read_bytes()
        old_profile = self.editor._profile_path.read_bytes()
        with patch("spd_envs.physical_materials.os.replace", side_effect=OSError("disk denied")):
            self.editor.key("enter")
        self.assert_runtime_state(before)
        np.testing.assert_array_equal(self.plant.model.numeric_data, old_numeric)
        for field in ("qacc", "qacc_warmstart", "ctrl", "qfrc_constraint", "efc_force"):
            np.testing.assert_array_equal(getattr(self.plant.data, field), getattr(old_data, field))
        np.testing.assert_array_equal(self.plant.data.contact.friction, old_data.contact.friction)
        self.assertIs(self.plant.scene_manifest, old_scene)
        self.assertIs(self.collection.task_manifest, old_manifest)
        self.assertIs(self.collection.source, old_source)
        self.assertEqual(self.config_path.read_bytes(), old_config)
        self.assertEqual(self.editor._profile_path.read_bytes(), old_profile)
        text = " ".join(self.editor.hud().values())
        self.assertIn("disk denied", text)
        self.assertNotIn("已应用并保存", text)
        self.assertEqual(tuple(self.editor._profile_path.parent.glob(".*.tmp")), ())
        self.assert_mu(self.plant.model, self.plant.data, self.coefficients[PAIR_KEYS[0]])
        self.editor.key("enter")  # A failed transaction can be retried explicitly.
        self.assertEqual(load_task_material_coefficients("test", "editor")[PAIR_KEYS[0]],
                         self.coefficients[PAIR_KEYS[0]] + .05)

    def test_real_open_recorder_prevents_apply_even_with_idle_lifecycle_labels(self):
        source = self.collection.source
        self.collection.recorder.start_episode(
            "material_guard", self.collection.task_manifest,
            model_bytes=source.model_bytes, metadata=source.metadata,
        )
        try:
            self.assertTrue(self.collection.recorder.is_busy)
            self.assertEqual(self.collection.state, "idle")
            self.editor.key("m")
            before = self.plant.model.numeric_data.copy()
            old_config = self.config_path.read_bytes()
            for key in ("right", "enter", "r", "s", "d"):
                self.assertTrue(self.editor.key(key))
            np.testing.assert_array_equal(self.plant.model.numeric_data, before)
            self.assertEqual(self.config_path.read_bytes(), old_config)
            self.assertIs(self.collection.source, source)
        finally:
            self.collection.recorder.discard_episode()

    def test_apply_persists_above_one_and_zero_without_changing_other_pairs(self):
        self.editor.key("m")
        for _ in range(25):
            self.editor.key("right")
        self.editor.key("enter")
        coefficients = load_task_material_coefficients("test", "editor")
        self.assertGreater(coefficients[PAIR_KEYS[0]], 1)
        self.assert_mu(self.plant.model, self.plant.data, coefficients[PAIR_KEYS[0]])
        for pair in PAIR_KEYS[1:]:
            self.assertEqual(coefficients[pair], self.coefficients[pair])
        for _ in range(100):
            self.editor.key("left")
        self.editor.key("enter")
        self.assertEqual(load_task_material_coefficients("test", "editor")[PAIR_KEYS[0]], 0)
        model = load_model(self.collection.source.model_bytes, self.collection.source.metadata)
        first, second = PAIR_KEYS[0]
        self.assertEqual(model.numeric_data[1 + 8 * MATERIAL_IDS[first] + MATERIAL_IDS[second]], 0)

    def test_snapshot_preparation_failure_never_writes_config(self):
        self.open_and_change()
        before = self.runtime_state()
        old_source, old_manifest = self.collection.source, self.collection.task_manifest
        old_scene = self.plant.scene_manifest
        old_numeric = self.plant.model.numeric_data.copy()
        with patch("simulation.material_editor.TrajectorySource", side_effect=ValueError("bad snapshot")), \
                patch("simulation.material_editor.save_task_material_coefficients") as save:
            self.editor.key("enter")
            save.assert_not_called()
        self.assert_runtime_state(before)
        self.assertIs(self.collection.source, old_source)
        self.assertIs(self.collection.task_manifest, old_manifest)
        self.assertIs(self.plant.scene_manifest, old_scene)
        np.testing.assert_array_equal(self.plant.model.numeric_data, old_numeric)
        self.assertIn("bad snapshot", " ".join(self.editor.hud().values()))

    def test_hud_reads_live_numeric_not_external_config_and_reset_discards_draft(self):
        changed_config = self.coefficients.copy()
        changed_config[PAIR_KEYS[0]] = 1.7
        save_material_coefficients(changed_config)
        self.editor.key("m")
        self.assertEqual(self.editor._draft, self.coefficients)
        self.editor.key("right")
        new_plant = self.make_plant()
        self.addCleanup(new_plant.physics.close)
        new_coefficients = self.coefficients.copy()
        new_coefficients[PAIR_KEYS[0]] = .85
        new_plant.model.numeric_data[:] = material_numeric_values(new_coefficients)
        self.app.plant = new_plant
        self.editor.reset_scene()
        self.assertFalse(self.editor.opened)
        self.editor.key("m")
        self.assertEqual(self.editor._draft, new_coefficients)

    def test_no_policy_model_only_previews_config_without_claiming_effect(self):
        plant = self.make_plant(policy=False)
        self.addCleanup(plant.physics.close)
        self.app.plant = plant
        self.editor.reset_scene()
        self.editor.key("m")
        draft = copy.deepcopy(self.editor._draft)
        with patch("simulation.material_editor.save_task_material_coefficients") as save:
            for key in ("right", "left", "enter", "r", "s", "d"):
                self.assertTrue(self.editor.key(key))
            save.assert_not_called()
        self.assertEqual(self.editor._draft, draft)
        self.assertIn("没有材质策略", " ".join(self.editor.hud().values()))
        self.assertEqual(plant.model.nnumeric, 0)

    def test_malformed_or_unapproved_matrix_is_readonly(self):
        self.plant.model.numeric_data[1] = .2  # unknown/unknown must remain -1.
        self.editor.reset_scene()
        self.editor.key("m")
        before = self.plant.model.numeric_data.copy()
        with patch("simulation.material_editor.save_task_material_coefficients") as save:
            self.editor.key("right")
            self.editor.key("enter")
            save.assert_not_called()
        np.testing.assert_array_equal(self.plant.model.numeric_data, before)
        self.assertIn("未批准材质对", " ".join(self.editor.hud().values()))

    def test_task_switch_uses_new_manifest_with_stale_args_and_isolated_profiles(self):
        self.open_and_change()
        self.editor.key("enter")
        first_path = self.editor._profile_path
        first_bytes = first_path.read_bytes()
        template_bytes = self.config_path.read_bytes()
        new_plant = self.make_plant(task="other")
        self.addCleanup(new_plant.physics.close)
        second_path = Path(new_plant.scene_manifest["physical_materials"]["task_profile"]["path"])
        self.app.plant = new_plant
        # Both args and collection are still old during parent _replace_scene.
        self.editor.reset_scene()
        self.editor.key("m")
        self.assertEqual(self.editor.hud()["当前任务"], "test/other")
        self.assertEqual(self.editor._profile_path, second_path)
        self.assertEqual(self.editor._draft, self.coefficients)
        with patch.dict(os.environ, SPD_TASK_MATERIAL_FRICTION_DIR=str(self.root / "moved")):
            self.editor.key("right")
            self.editor.key("right")
            self.editor.key("enter")
        self.assertEqual(self.collection.task_manifest["task"], "other")
        self.assertEqual(first_path.read_bytes(), first_bytes)
        self.assertEqual(self.config_path.read_bytes(), template_bytes)
        self.assertEqual(load_task_material_coefficients("test", "other")[PAIR_KEYS[0]], .7)
        self.assertEqual(load_task_material_coefficients("test", "editor")[PAIR_KEYS[0]], .65)
        self.editor.key("m")
        self.editor.key("m")
        self.assertEqual(self.editor._draft[PAIR_KEYS[0]], .7)
        restored = load_model(self.collection.source.model_bytes, self.collection.source.metadata)
        np.testing.assert_array_equal(restored.numeric_data, new_plant.model.numeric_data)
        self.assertEqual(self.collection.source.metadata["scene_manifest"]["physical_materials"]
                         ["task_profile"]["path"], str(second_path))

    def test_irrelevant_pairs_cannot_be_selected_or_changed(self):
        expected_pairs = model_material_pairs(self.plant.model)
        self.editor.key("m")
        self.assertEqual(self.editor._pairs, expected_pairs)
        for index in range(len(expected_pairs)):
            self.assertEqual(self.editor._pairs[self.editor._selected], expected_pairs[index])
            self.assertTrue(any(key.startswith("▶") for key in self.editor.hud()))
            self.editor.key("right")
            self.editor.key("down")
        self.editor.key("enter")
        expected = self.coefficients.copy()
        for pair in expected_pairs:
            expected[pair] += .05
        self.assertEqual(load_task_material_coefficients("test", "editor"), expected)
        np.testing.assert_array_equal(self.plant.model.numeric_data, material_numeric_values(expected))
        self.assertEqual(load_material_coefficients(), self.coefficients)

    def test_empty_material_set_navigation_and_apply_are_safe(self):
        self.plant.model.geom_contype[:] = 0
        self.plant.model.geom_conaffinity[:] = 0
        self.editor.reset_scene()
        self.editor.key("m")
        before = self.plant.model.numeric_data.copy()
        with patch("simulation.material_editor.save_task_material_coefficients") as save:
            for key in ("up", "down", "left", "right", "enter"):
                self.assertTrue(self.editor.key(key))
            save.assert_not_called()
        self.assertEqual(self.editor._pairs, ())
        self.assertEqual(self.editor._selected, 0)
        np.testing.assert_array_equal(self.plant.model.numeric_data, before)

    def test_material_filter_includes_bottom_rack_robot_and_ignores_noncollidable(self):
        cases = (
            ("bottles", ("polyethylene",), 4),
            ("jenga", ("wood",), 4),
            ("plates_without_rack", ("ceramic_glaze",), 8),
            ("plates_with_rack", ("ceramic_glaze", "bare_iron"), 13),
            ("mug", ("wood", "ceramic_glaze", "bare_iron"), 19),
        )
        for name, objects, count in cases:
            with self.subTest(scene=name):
                materials = (*objects, "polyester_woven_fabric", "silicone")
                geoms = "".join(
                    f'<geom name="{material}" type="sphere" size=".01" pos="{index} 0 0" '
                    f'user="0 0 {MATERIAL_IDS[material]} {1 if material == "ceramic_glaze" else 0}"/>'
                    for index, material in enumerate(materials)
                )
                model = mujoco.MjModel.from_xml_string(
                    '<mujoco><size nuser_geom="4"/><worldbody>'
                    f'{geoms}<geom type="sphere" size=".01" contype="0" conaffinity="0" '
                    'user="0 0 2 0"/><geom type="sphere" size=".01" user="0 0 0 0"/>'
                    '</worldbody></mujoco>'
                )
                expected_ids = set(materials)
                if "ceramic_glaze" in materials:
                    expected_ids.add("unglazed_ceramic")
                expected = tuple(pair for pair in PAIR_KEYS if all(m in expected_ids for m in pair))
                self.assertEqual(model_material_pairs(model), expected)
                self.assertEqual(len(expected), count)
                # No MjData/contact generation is involved in panel selection.


if __name__ == "__main__":
    unittest.main()
