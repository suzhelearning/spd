"""Editable material policies are strict, atomic, and stable per built scene."""
from concurrent.futures import ThreadPoolExecutor
import copy
import os
from pathlib import Path
import tempfile
from threading import Barrier
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import yaml

from spd_envs.model_scene import write_scene_model
from spd_envs.physical_materials import (
    MATERIAL_IDS, PAIR_KEYS, load_material_coefficients, load_task_material_coefficients,
    material_config_path, model_material_pairs, save_material_coefficients,
    save_task_material_coefficients, task_material_config_path,
)
from spd_envs.scene_builder import ProceduralSceneBuilder


# Independent fixture: user edits to the repository policy must not alter tests.
DEFAULT_VALUES = (
    .6, .4, .5, .5, .6, .6, .4, .5, .4, .5, .6, .7, .5, .6, .5,
    .7, .5, .6, .7, .6, .9, .9, .9, .9, .9, .9,
)


class MaterialConfigTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.directory = Path(directory.name)
        self.config = self.directory / "material_friction.yaml"
        self.defaults = dict(zip(PAIR_KEYS, DEFAULT_VALUES, strict=True))
        save_material_coefficients(self.defaults, self.config)
        override = patch.dict(os.environ, {
            "SPD_MATERIAL_FRICTION_CONFIG": str(self.config),
            "SPD_TASK_MATERIAL_FRICTION_DIR": str(self.directory / "profiles"),
        })
        override.start()
        self.addCleanup(override.stop)

    def document(self):
        return yaml.safe_load(self.config.read_text(encoding="utf-8"))

    def write_document(self, document):
        self.config.write_text(yaml.safe_dump(document), encoding="utf-8")

    def test_persistence_allows_zero_and_values_above_one_and_normalizes_pair_order(self):
        self.assertEqual(material_config_path(), self.config)
        document = self.document()
        document["pairs"][2]["materials"].reverse()
        document["pairs"][2]["sliding_mu"] = 1.234567890123456
        document["pairs"][0]["sliding_mu"] = 0
        self.write_document(document)
        loaded = load_material_coefficients()
        self.assertEqual(tuple(loaded), PAIR_KEYS)
        self.assertEqual(loaded[("wood", "polyethylene")], 1.234567890123456)
        self.assertEqual(loaded[("wood", "wood")], 0)
        save_material_coefficients(loaded)
        self.assertEqual(load_material_coefficients(self.config), loaded)
        self.assertEqual(len(self.document()["pairs"]), 26)

    def test_read_rejects_incomplete_unknown_duplicate_and_invalid_coefficients(self):
        original = self.document()
        invalid = []
        document = copy.deepcopy(original)
        document["pairs"].pop()
        invalid.append(("missing", document))
        for pair in (("silicone", "silicone"), ("polyester_woven_fabric", "polyester_woven_fabric"),
                     ("unknown", "wood")):
            document = copy.deepcopy(original)
            document["pairs"][0]["materials"] = list(pair)
            invalid.append((str(pair), document))
        document = copy.deepcopy(original)
        duplicate = copy.deepcopy(document["pairs"][2])
        duplicate["materials"].reverse()
        document["pairs"].append(duplicate)
        invalid.append(("asymmetric duplicate", document))
        for value in (-.05, float("nan"), float("inf"), float("-inf"), True, "0.5", None):
            document = copy.deepcopy(original)
            document["pairs"][0]["sliding_mu"] = value
            invalid.append((repr(value), document))
        for name, replacement in (("version", True), ("pairs", {})):
            document = copy.deepcopy(original)
            document[name] = replacement
            invalid.append((name, document))
        document = copy.deepcopy(original)
        document["pairs"][0]["friction"] = .8
        invalid.append(("extra pair field", document))
        document = copy.deepcopy(original)
        document["matrix"] = []
        invalid.append(("extra root field", document))
        for name, document in invalid:
            with self.subTest(name=name):
                self.write_document(document)
                with self.assertRaises(ValueError):
                    load_material_coefficients()

    def test_duplicate_yaml_fields_are_errors_not_silent_overrides(self):
        original = self.config.read_text(encoding="utf-8")
        for text in (original + "version: 1\n",
                     original.replace("sliding_mu: 0.6", "sliding_mu: 0.6\n  sliding_mu: 0.9", 1),
                     "version: 1\npairs: [\n"):
            with self.subTest(text=text):
                self.config.write_text(text, encoding="utf-8")
                with self.assertRaises(ValueError):
                    load_material_coefficients()

    def test_invalid_saves_and_io_failures_never_replace_existing_policy(self):
        original = self.config.read_bytes()
        invalid = []
        values = self.defaults.copy()
        values.pop(PAIR_KEYS[0])
        invalid.append(values)
        for key, value in ((PAIR_KEYS[0], -.1), (PAIR_KEYS[0], float("nan")),
                           (("silicone", "silicone"), .8),
                           (("polyethylene", "wood"), .8)):
            values = self.defaults.copy()
            values[key] = value
            invalid.append(values)
        for values in invalid:
            with self.subTest(values=values):
                with self.assertRaises(ValueError):
                    save_material_coefficients(values)
                self.assertEqual(self.config.read_bytes(), original)
        changed = self.defaults.copy()
        changed[PAIR_KEYS[0]] = 1.25
        for operation in ("os.fsync", "os.replace"):
            with self.subTest(operation=operation):
                with patch("spd_envs.physical_materials." + operation,
                           side_effect=OSError("injected write failure")):
                    with self.assertRaises(OSError):
                        save_material_coefficients(changed)
                self.assertEqual(self.config.read_bytes(), original)
                self.assertEqual(load_material_coefficients(), self.defaults)
                self.assertEqual(set(self.directory.iterdir()), {self.config})

    @staticmethod
    def matrix(model):
        identifier = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_NUMERIC, "spd_material_friction")
        address, size = int(model.numeric_adr[identifier]), int(model.numeric_size[identifier])
        values = model.numeric_data[address:address + size]
        if values.shape != (65,) or values[0] != 2:
            raise AssertionError("invalid portable material numeric")
        return values[1:].reshape(8, 8)

    def test_scene_manifest_standalone_and_robot_merge_share_captured_policy(self):
        builder = ProceduralSceneBuilder("jenga", "hollow_tower", 0)
        old = builder.build()
        old_manifest = old.manifest()
        old_xml = old.xml_string()
        changed = self.defaults.copy()
        changed[("wood", "wood")] = 1.234567890123456
        changed[("wood", "silicone")] = 0
        save_task_material_coefficients(old.scene, old.task, changed, path=old.material_profile_path)
        fresh = builder.build()
        self.assertEqual(old.manifest(), old_manifest)
        self.assertEqual(old.xml_string(), old_xml)
        self.assertEqual(old.with_table_near_edge(.18).material_coefficients, self.defaults)
        self.assertEqual(fresh.material_coefficients, changed)
        # No random resampling side effects from loading a different material policy.
        self.assertEqual(old.objects, fresh.objects)
        self.assertEqual(old.sampled_values, fresh.sampled_values)
        self.assertEqual(ET.tostring(old.worldbody), ET.tostring(fresh.worldbody))
        base = self.directory / "robot.xml"
        base.write_text('''<mujoco><worldbody>
          <body name="l_index_finger_distal" pos="0 0 3">
            <geom name="tip" type="sphere" size="0.01"/>
          </body>
        </worldbody></mujoco>''', encoding="utf-8")
        for name, scene, expected in (("old", old, self.defaults), ("new", fresh, changed)):
            with self.subTest(scene=name):
                merged = write_scene_model(base, scene, self.directory / (name + ".xml"))
                standalone_model = mujoco.MjModel.from_xml_string(scene.xml_string())
                merged_model = mujoco.MjModel.from_xml_path(str(merged))
                np.testing.assert_array_equal(standalone_model.numeric_data, merged_model.numeric_data)
                binary = self.directory / (name + ".mjb")
                mujoco.mj_saveModel(merged_model, str(binary), None)
                restored = mujoco.MjModel.from_binary_path(str(binary))
                np.testing.assert_array_equal(restored.numeric_data, merged_model.numeric_data)
                matrix = self.matrix(merged_model)
                np.testing.assert_array_equal(matrix, matrix.T)
                np.testing.assert_array_equal(matrix[0], [-1.] * 8)
                self.assertEqual(matrix[6, 6], -1.)
                self.assertEqual(matrix[7, 7], -1.)
                self.assertEqual(merged_model.geom_user[merged_model.geom("tip").id, 2], 7)
                manifest_pairs = {
                    tuple(entry["materials"]): entry["effective_sliding_mu"]
                    for entry in scene.manifest()["physical_materials"]["approved_pairs"]
                }
                self.assertEqual(manifest_pairs, expected)
                self.assertEqual(scene.manifest()["physical_materials"]["task_profile"], {
                    "scene": scene.scene, "task": scene.task,
                    "path": str(scene.material_profile_path),
                })
                for (first, second), value in expected.items():
                    self.assertEqual(matrix[MATERIAL_IDS[first], MATERIAL_IDS[second]], value)
        # Even a now-invalid external config cannot change or invalidate an old scene.
        self.config.write_text("version: 1\npairs: []\n", encoding="utf-8")
        self.assertEqual(old.manifest(), old_manifest)
        self.assertEqual(old.xml_string(), old_xml)
        write_scene_model(base, old, self.directory / "old_after_invalid_config.xml")
        self.assertEqual(builder.build().material_coefficients, changed)
        old.material_profile_path.write_text("version: 1\npairs: []\n", encoding="utf-8")
        with self.assertRaises(ValueError):
            builder.build()

    def test_current_saved_global_is_copied_completely_once_and_never_modified(self):
        changed = {pair: value + 1.125 for pair, value in self.defaults.items()}
        save_material_coefficients(changed)
        original = self.config.read_bytes()
        self.assertEqual(load_task_material_coefficients("test", "first"), changed)
        profile = task_material_config_path("test", "first")
        document = yaml.safe_load(profile.read_text(encoding="utf-8"))
        self.assertEqual((document["scene"], document["task"]), ("test", "first"))
        self.assertEqual(len(document["pairs"]), 26)
        first_bytes = profile.read_bytes()
        self.assertEqual(self.config.read_bytes(), original)
        self.config.unlink()
        with patch("spd_envs.physical_materials.load_material_coefficients",
                   side_effect=AssertionError("existing task must not consult global")):
            self.assertEqual(load_task_material_coefficients("test", "first"), changed)
        self.assertEqual(profile.read_bytes(), first_bytes)

    def test_task_a_b_a_new_seed_and_reopening_keep_independent_complete_policies(self):
        first = ProceduralSceneBuilder("jenga", "hollow_tower", 0).build()
        other = ProceduralSceneBuilder("jenga", "tower", 0).build()
        original_global = self.config.read_bytes()
        changed = self.defaults.copy()
        changed[("wood", "wood")] = 1.45
        changed[("polyethylene", "silicone")] = 1.75  # Hidden in this task, but still retained.
        save_task_material_coefficients(first.scene, first.task, changed)
        self.assertEqual(self.config.read_bytes(), original_global)
        self.config.write_text("broken: [", encoding="utf-8")
        for seed in (0, 9):
            reopened = ProceduralSceneBuilder(first.scene, first.task, seed).build()
            self.assertEqual(reopened.material_coefficients, changed)
            self.assertEqual(reopened.material_profile_path, first.material_profile_path)
        self.assertEqual(ProceduralSceneBuilder(other.scene, other.task, 9).build().material_coefficients,
                         self.defaults)
        self.assertNotEqual(first.material_profile_path, other.material_profile_path)
        self.assertEqual(first.material_coefficients, self.defaults)

    def test_existing_bad_profiles_fail_without_fallback_or_replacement(self):
        load_task_material_coefficients("test", "bad")
        profile = task_material_config_path("test", "bad")
        valid = yaml.safe_load(profile.read_text(encoding="utf-8"))
        invalid = ["version: 1\npairs: [", "version: 1\npairs: []\n"]
        for field, value in (("scene", "other"), ("task", "other"), ("version", 2)):
            document = copy.deepcopy(valid)
            document[field] = value
            invalid.append(yaml.safe_dump(document))
        document = copy.deepcopy(valid)
        document["pairs"].pop()
        invalid.append(yaml.safe_dump(document))
        invalid.append(yaml.safe_dump(valid) + "scene: test\n")
        for payload in invalid:
            with self.subTest(payload=payload):
                profile.write_text(payload, encoding="utf-8")
                with patch("spd_envs.physical_materials.load_material_coefficients",
                           side_effect=AssertionError("no fallback")):
                    with self.assertRaises(ValueError):
                        load_task_material_coefficients("test", "bad")
                    with self.assertRaises(ValueError):
                        save_task_material_coefficients("test", "bad", self.defaults)
                self.assertEqual(profile.read_text(encoding="utf-8"), payload)
        profile.write_text(yaml.safe_dump(valid), encoding="utf-8")
        before = profile.read_bytes()
        with patch.object(Path, "read_text", side_effect=PermissionError("unreadable")):
            with self.assertRaises(PermissionError):
                load_task_material_coefficients("test", "bad")
        self.assertEqual(profile.read_bytes(), before)
        profile.unlink()
        profile.symlink_to(profile.parent / "missing.yaml")
        with self.assertRaises(FileNotFoundError):
            load_task_material_coefficients("test", "bad")
        self.assertTrue(profile.is_symlink())

    def test_concurrent_first_loads_publish_one_complete_profile(self):
        barrier = Barrier(8)

        def initialize(_):
            barrier.wait()
            return load_task_material_coefficients("test", "parallel")

        with ThreadPoolExecutor(max_workers=8) as workers:
            results = list(workers.map(initialize, range(8)))
        self.assertEqual(results, [self.defaults] * 8)
        profile = task_material_config_path("test", "parallel")
        self.assertEqual(set(profile.parent.iterdir()), {profile})

    def test_first_creation_never_overwrites_a_concurrent_saved_winner(self):
        profile = task_material_config_path("test", "winner")
        winner = {pair: value + .25 for pair, value in self.defaults.items()}
        real_link = os.link

        def publish_winner(source, destination):
            save_task_material_coefficients("test", "winner", winner, path=profile)
            real_link(source, destination)  # Fails because the complete winner exists.

        with patch("spd_envs.physical_materials.os.link", side_effect=publish_winner):
            self.assertEqual(load_task_material_coefficients("test", "winner"), winner)
        self.assertEqual(load_task_material_coefficients("test", "winner"), winner)
        self.assertEqual(set(profile.parent.iterdir()), {profile})

    def test_failed_first_publication_leaves_no_partial_profile(self):
        profile = task_material_config_path("test", "failed")
        for operation in ("os.fsync", "os.link"):
            with self.subTest(operation=operation):
                with patch("spd_envs.physical_materials." + operation,
                           side_effect=OSError("publication failed")):
                    with self.assertRaises(OSError):
                        load_task_material_coefficients("test", "failed")
                self.assertFalse(profile.exists())
                self.assertEqual(set(profile.parent.iterdir()), set())
        self.config.unlink()
        with self.assertRaises(FileNotFoundError):
            load_task_material_coefficients("test", "failed")
        self.assertFalse(profile.exists())

    def test_same_task_name_in_different_scenes_is_independent(self):
        load_task_material_coefficients("first_scene", "shared_task")
        load_task_material_coefficients("second_scene", "shared_task")
        changed = self.defaults.copy()
        changed[PAIR_KEYS[0]] = 3.0
        save_task_material_coefficients("first_scene", "shared_task", changed)
        self.assertEqual(load_task_material_coefficients("first_scene", "shared_task"), changed)
        self.assertEqual(load_task_material_coefficients("second_scene", "shared_task"), self.defaults)

    def test_profile_save_is_atomic_and_captured_path_survives_env_change(self):
        load_task_material_coefficients("test", "saved")
        profile = task_material_config_path("test", "saved")
        before = profile.read_bytes()
        changed = self.defaults.copy()
        changed[PAIR_KEYS[0]] = 2.75
        for operation in ("os.fsync", "os.replace"):
            with self.subTest(operation=operation):
                with patch("spd_envs.physical_materials." + operation,
                           side_effect=OSError("write failed")):
                    with self.assertRaises(OSError):
                        save_task_material_coefficients("test", "saved", changed, path=profile)
                self.assertEqual(profile.read_bytes(), before)
                self.assertEqual(set(profile.parent.iterdir()), {profile})
        with patch.dict(os.environ, {"SPD_TASK_MATERIAL_FRICTION_DIR": str(self.directory / "elsewhere")}):
            save_task_material_coefficients("test", "saved", changed, path=profile)
            self.assertFalse(task_material_config_path("test", "saved").exists())
            with self.assertRaises(ValueError):
                save_task_material_coefficients("test", "wrong", changed, path=profile)
        self.assertEqual(load_task_material_coefficients("test", "saved"), changed)

    def test_profile_paths_isolate_template_override_and_installed_user_config(self):
        with patch.dict(os.environ):
            os.environ.pop("SPD_TASK_MATERIAL_FRICTION_DIR")
            self.assertEqual(task_material_config_path("test", "path"),
                             self.directory / "task_material_friction/test/path.yaml")
            package = self.directory / "site-packages/spd_envs"
            with patch("spd_envs.physical_materials.__file__", str(package / "physical_materials.py")):
                os.environ.pop("SPD_MATERIAL_FRICTION_CONFIG")
                os.environ["XDG_CONFIG_HOME"] = str(self.directory / "user-config")
                template = package / "defaults/material_friction.yaml"
                save_material_coefficients(self.defaults, template)
                self.assertEqual(material_config_path(), template)
                profile = task_material_config_path("test", "installed")
                self.assertEqual(profile, self.directory / "user-config/spd/task_material_friction/test/installed.yaml")
                self.assertEqual(load_task_material_coefficients("test", "installed"), self.defaults)
                self.assertTrue(profile.is_file())
                template.unlink()
                self.assertEqual(load_task_material_coefficients("test", "installed"), self.defaults)
        for scene, task in (("", "a"), ("a", ""), ("..", "a"), ("a", "../b"),
                            ("/absolute", "a"), ("a", r"b\c"), ("a", " x ")):
            with self.subTest(scene=scene, task=task):
                with self.assertRaises(ValueError):
                    task_material_config_path(scene, task)

    def test_actual_task_models_filter_only_present_approved_surfaces(self):
        for scene, task, materials in (
            ("jenga", "hollow_tower", {"wood", "polyester_woven_fabric"}),
            ("cups", "pyramid", {"polyethylene", "polyester_woven_fabric"}),
            ("mugs", "hang_mug", {"wood", "ceramic_glaze", "unglazed_ceramic",
                                "bare_iron", "polyester_woven_fabric"}),
        ):
            with self.subTest(scene=scene):
                result = ProceduralSceneBuilder(scene, task, 0).build()
                standalone = mujoco.MjModel.from_xml_string(result.xml_string())
                expected = tuple(pair for pair in PAIR_KEYS if set(pair) <= materials)
                self.assertEqual(model_material_pairs(standalone), expected)
                robot = self.directory / (scene + "_robot.xml")
                robot.write_text('''<mujoco><worldbody><body name="l_wrist" pos="0 0 3">
                  <geom name="palm" type="sphere" size=".01"/>
                </body></worldbody></mujoco>''', encoding="utf-8")
                merged = write_scene_model(robot, result, self.directory / (scene + "_merged.xml"))
                merged_model = mujoco.MjModel.from_xml_path(str(merged))
                expected = tuple(pair for pair in PAIR_KEYS if set(pair) <= materials | {"silicone"})
                self.assertEqual(model_material_pairs(merged_model), expected)

    def test_filter_ignores_visual_unknown_and_unapproved_pairs_without_contacts(self):
        model = mujoco.MjModel.from_xml_string('''<mujoco><size nuser_geom="4"/>
          <worldbody>
            <geom type="sphere" pos="0 0 0" size=".01" user="0 0 2 0" contype="1" conaffinity="0"/>
            <geom type="sphere" pos="1 0 0" size=".01" user="0 0 6 0" contype="0" conaffinity="1"/>
            <geom type="sphere" pos="2 0 0" size=".01" user="0 0 7 3"/>
            <geom type="sphere" pos="3 0 0" size=".01" user="0 0 0 1"/>
            <geom type="sphere" pos="4 0 0" size=".01" user="0 0 1 0" contype="0" conaffinity="0"/>
          </worldbody></mujoco>''')
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        self.assertEqual(data.ncon, 0)
        self.assertEqual(model_material_pairs(model), (
            ("polyethylene", "polyethylene"), ("polyethylene", "polyester_woven_fabric"),
            ("polyethylene", "silicone"), ("polyester_woven_fabric", "silicone"),
        ))
        old = mujoco.MjModel.from_xml_string('<mujoco><worldbody><geom size=".01"/></worldbody></mujoco>')
        self.assertEqual(model_material_pairs(old), ())


if __name__ == "__main__":
    unittest.main()
