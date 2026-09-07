from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from spd_vr.model_compiler.artifacts import compile_models, verify_artifacts
from spd_vr.manifest import load_manifest
from spd_vr.model_compiler.collision import CollisionArtifact, _canonical_mesh_bytes
from spd_vr.model_builder import workspace_root


URDF = workspace_root() / "assets" / "tianji_wuji2" / "tianji_wuji2.urdf"


def _stub_quality_gated_collision(monkeypatch: pytest.MonkeyPatch, tmp_path: Path):
    piece = tmp_path / "piece.mesh"
    raw = _canonical_mesh_bytes(
        (
            np.array([[0, 0, 0], [0.1, 0, 0], [0, 0.1, 0], [0, 0, 0.1]], dtype=float),
            np.array([[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]], dtype=int),
        )
    )
    piece.write_bytes(raw)
    digest = __import__("hashlib").sha256(raw).hexdigest()
    source_digest = __import__("hashlib").sha256

    def decompose(mesh, settings, cache):
        return CollisionArtifact(
            "test-cache-key",
            (piece,),
            (digest,),
            0.0,
            source_digest(mesh.path.read_bytes()).hexdigest(),
            tuple(mesh.scale),
            metrics={"piece_count": 1, "surface_p95_m": 0.0},
        )

    monkeypatch.setattr("spd_vr.model_compiler.artifacts.decompose_mesh", decompose)


def test_compile_models_writes_five_loadable_artifacts(tmp_path: Path, monkeypatch: pytest.MonkeyPatch):
    pytest.importorskip("mujoco")
    _stub_quality_gated_collision(monkeypatch, tmp_path)
    result = compile_models(URDF, tmp_path / "generated", tmp_path / "cache")
    assert result.files == (
        "unified_plant.xml",
        "arm_ik.xml",
        "model_manifest.yaml",
        "collision_manifest.yaml",
        "actuator_calibration.yaml",
    )
    import mujoco
    import yaml

    full = mujoco.MjModel.from_xml_path(str(result.full_model))
    arm = mujoco.MjModel.from_xml_path(str(result.arm_model))
    assert (full.nq, full.nv, full.nu) == (54, 54, 54)
    assert (arm.nq, arm.nv, arm.nu) == (14, 14, 14)
    arm_actuator = mujoco.mj_name2id(full, mujoco.mjtObj.mjOBJ_ACTUATOR, "Joint1_L_position")
    assert full.actuator_gainprm[arm_actuator, 0] == pytest.approx(500.0)
    assert full.actuator_biasprm[arm_actuator, 2] < 0.0
    upstream_hand_root = (
        workspace_root()
        / "packages"
        / "wuji-retargeting"
        / "wuji_retargeting"
        / "wuji-description"
        / "hand2"
        / "hand2_beta2"
        / "body"
        / "mjcf"
    )
    for side in ("left", "right"):
        official = ET.parse(upstream_hand_root / f"{side}.xml").getroot()
        official_gains = {
            element.attrib["joint"]: (
                float(element.attrib["kp"]),
                float(element.attrib["kv"]),
            )
            for element in official.findall("./actuator/position")
        }
        assert len(official_gains) == 20
        for joint_name, (kp, kv) in official_gains.items():
            actuator_id = mujoco.mj_name2id(
                full,
                mujoco.mjtObj.mjOBJ_ACTUATOR,
                f"{joint_name}_position",
            )
            joint_id = mujoco.mj_name2id(
                full, mujoco.mjtObj.mjOBJ_JOINT, joint_name
            )
            assert actuator_id >= 0
            assert joint_id >= 0
            assert full.actuator_gainprm[actuator_id, 0] == pytest.approx(kp)
            assert -full.actuator_biasprm[actuator_id, 2] == pytest.approx(kv)
            dof_id = int(full.jnt_dofadr[joint_id])
            assert full.dof_damping[dof_id] == pytest.approx(0.0)
    assert all(mujoco.mj_name2id(arm, mujoco.mjtObj.mjOBJ_SITE, name) >= 0 for name in ("l_wrist_target", "r_wrist_target"))
    manifest = yaml.safe_load(result.path.read_text(encoding="utf-8"))
    assert load_manifest(result.path)["dof"] == 54
    assert len(manifest["axis_visuals"]) == 24
    assert manifest["arm_home_rad"]["left"] == pytest.approx(
        [0.9599310886, -1.1344640138, -1.2217304764, -1.0471975512, 1.0471975512, 0.0, 0.0]
    )
    assert manifest["arm_home_rad"]["right"] == pytest.approx(
        [-0.9599310886, -1.1344640138, 1.2217304764, -1.0471975512, -1.0471975512, 0.0, 0.0]
    )
    from spd_vr.arm_ik import _production_controller
    from spd_vr.viewer import PlantController

    ik = _production_controller(arm, type("Verified", (), {"manifest": manifest})())
    np.testing.assert_allclose(ik.left_solver.home, manifest["arm_home_rad"]["left"])
    np.testing.assert_allclose(ik.right_solver.home, manifest["arm_home_rad"]["right"])
    plant = PlantController(result.full_model, result.path, model=full, hand_retargeter=object(), strict_artifacts=False)
    np.testing.assert_allclose(plant.data.qpos[:7], manifest["arm_home_rad"]["left"])
    np.testing.assert_allclose(plant.data.qpos[27:34], manifest["arm_home_rad"]["right"])
    np.testing.assert_allclose(plant.data.ctrl[:7], manifest["arm_home_rad"]["left"])
    np.testing.assert_allclose(plant.data.ctrl[27:34], manifest["arm_home_rad"]["right"])
    plant.close()
    assert len(manifest["collision"]["adjacent_excludes"]) == 79
    assert manifest["source"]["meshes"]
    assert not Path(manifest["source"]["urdf"]).is_absolute()
    assert all(not Path(record["path"]).is_absolute() for record in manifest["visual_meshes"])


def test_compile_models_can_reuse_raw_collision_meshes(tmp_path: Path, monkeypatch: pytest.MonkeyPatch):
    monkeypatch.setattr(
        "spd_vr.model_compiler.artifacts.decompose_mesh",
        lambda *_args, **_kwargs: pytest.fail("raw collision mode called CoACD"),
    )
    output = tmp_path / "generated"
    output.mkdir()
    result = compile_models(URDF, output, raw_collisions=True)

    import mujoco
    import yaml

    assert mujoco.MjModel.from_xml_path(str(result.full_model)).nq == 54
    collision = yaml.safe_load(result.collision_manifest.read_text(encoding="utf-8"))
    assert collision["settings"] == {"mode": "raw"}
    assert collision["records"]
    assert all(record["mode"] == "raw" and record["piece_count"] == 1 for record in collision["records"])


def test_verify_artifacts_rejects_source_or_output_tampering(tmp_path: Path, monkeypatch: pytest.MonkeyPatch):
    _stub_quality_gated_collision(monkeypatch, tmp_path)
    result = compile_models(URDF, tmp_path / "generated", tmp_path / "cache")
    manifest_path = result.path
    verify_artifacts(manifest_path, URDF)
    source = tmp_path / "copy.urdf"
    source.write_bytes(URDF.read_bytes() + b"\n")
    with pytest.raises(ValueError):
        verify_artifacts(manifest_path, source)
    import yaml
    manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    visual = result.output_dir / manifest["visual_meshes"][0]["output_file"]
    visual_bytes = visual.read_bytes()
    visual.write_bytes(visual_bytes + b"\n")
    with pytest.raises(ValueError):
        verify_artifacts(manifest_path, URDF)
    visual.write_bytes(visual_bytes)
    manifest_bytes = manifest_path.read_bytes()
    manifest["visual_meshes"][0]["path"] = "../escape.stl"
    manifest["manifest_sha256"] = ""
    manifest["manifest_sha256"] = __import__("hashlib").sha256(
        yaml.safe_dump(manifest, sort_keys=True, allow_unicode=True).encode("utf-8")
    ).hexdigest()
    manifest_path.write_bytes(yaml.safe_dump(manifest, sort_keys=True, allow_unicode=True).encode("utf-8"))
    with pytest.raises(ValueError):
        verify_artifacts(manifest_path, URDF)
    manifest_path.write_bytes(manifest_bytes)
    xml = result.full_model
    original = xml.read_bytes()
    xml.write_bytes(original + b"\n")
    with pytest.raises(ValueError):
        verify_artifacts(manifest_path, URDF)
