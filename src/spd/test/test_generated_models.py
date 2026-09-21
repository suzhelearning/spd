from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from spd_vr.description.model_compiler.artifacts import ArtifactError, compile_models, verify_artifacts
from spd_vr.description.model_compiler.collision import CollisionArtifact, _canonical_mesh_bytes
from spd_vr.description.model_builder import description_root


URDF = description_root() / "assets" / "tianji_wuji2.urdf"


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

    monkeypatch.setattr('spd_vr.description.model_compiler.artifacts.decompose_mesh', decompose)


def test_compile_models_writes_five_loadable_artifacts(tmp_path: Path, monkeypatch: pytest.MonkeyPatch):
    pytest.importorskip("mujoco")
    _stub_quality_gated_collision(monkeypatch, tmp_path)
    result = compile_models(URDF, tmp_path / "generated", tmp_path / "cache")
    import mujoco
    import yaml

    full = mujoco.MjModel.from_xml_path(str(result.full_model))
    arm = mujoco.MjModel.from_xml_path(str(result.arm_model))
    assert (full.nq, full.nv, full.nu) == (54, 54, 54)
    assert (arm.nq, arm.nv, arm.nu) == (14, 14, 14)
    arm_actuator = mujoco.mj_name2id(full, mujoco.mjtObj.mjOBJ_ACTUATOR, "Joint1_L_position")
    assert full.actuator_gainprm[arm_actuator, 0] == pytest.approx(500.0)
    assert full.actuator_biasprm[arm_actuator, 2] < 0.0
    assert all(mujoco.mj_name2id(arm, mujoco.mjtObj.mjOBJ_SITE, name) >= 0 for name in ("l_wrist_target", "r_wrist_target"))
    manifest = yaml.safe_load(result.path.read_text(encoding="utf-8"))
    from spd_vr.simulation.viewer import PlantController

    plant = PlantController(result.full_model, result.path, model=full, strict_artifacts=False)
    np.testing.assert_allclose(plant.data.qpos[:7], manifest["arm_home_rad"]["left"])
    np.testing.assert_allclose(plant.data.qpos[27:34], manifest["arm_home_rad"]["right"])
    np.testing.assert_allclose(plant.data.ctrl[:7], manifest["arm_home_rad"]["left"])
    np.testing.assert_allclose(plant.data.ctrl[27:34], manifest["arm_home_rad"]["right"])
    plant.close()


def test_compile_models_can_reuse_raw_collision_meshes(tmp_path: Path, monkeypatch: pytest.MonkeyPatch):
    monkeypatch.setattr(
        'spd_vr.description.model_compiler.artifacts.decompose_mesh',
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


def test_selected_collisions_override_raw_without_changing_other_geometry(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch,
):
    import yaml
    selected_links = ("Link5_L", "Link7_L", "Link5_R", "Link7_R")

    _stub_quality_gated_collision(monkeypatch, tmp_path)
    raw = compile_models(URDF, tmp_path / "raw", raw_collisions=True)
    mixed = compile_models(
        URDF, tmp_path / "mixed", raw_collisions=True,
        decompose_links=iter(selected_links),
    )
    verify_artifacts(mixed.path, URDF)
    raw_records = {
        (record["link"], record["collision_index"]): record
        for record in yaml.safe_load(raw.collision_manifest.read_text())["records"]
    }
    collision = yaml.safe_load(mixed.collision_manifest.read_text())
    assert collision["settings"]["mode"] == "mixed"
    assert set(collision["settings"]["decompose_links"]) == set(selected_links)
    selected_records = [record for record in collision["records"] if record["mode"] == "decomposed"]
    assert {record["link"] for record in selected_records} == set(selected_links)
    for record in collision["records"]:
        if record["link"] not in selected_links:
            assert record == raw_records[(record["link"], record["collision_index"])]
            continue
        assert record["conservative_simplification"] is True
        assert record["conservative_simplification_revision"]
        assert record["settings"]["decimate"] is False
        assert record["settings"]["preprocess_resolution"] == 200
        assert record["settings"]["extrude"] is True
        assert record["settings"]["threshold"] == 0.05
        assert record["surface_p95_threshold_m"] == 0.003
        assert all(piece["file"].startswith("collision/") for piece in record["pieces"])

    # Beyond the selected collision geoms and their mesh assets, both model
    # projections must retain identical visuals, inertials, joints and contacts.
    for raw_path, mixed_path in ((raw.full_model, mixed.full_model), (raw.arm_model, mixed.arm_model)):
        roots = [ET.parse(path).getroot() for path in (raw_path, mixed_path)]
        for root in roots:
            root.remove(root.find("asset"))
            for body in root.iter("body"):
                if body.attrib["name"] in selected_links:
                    for geom in list(body.findall("geom")):
                        if geom.attrib.get("contype") == "1":
                            body.remove(geom)
        assert ET.tostring(roots[0]) == ET.tostring(roots[1])

    # Even consistently rehashed metadata cannot relabel a selected precise
    # record as raw and still satisfy the declared compilation policy.
    import hashlib

    selected_records[0]["mode"] = "raw"
    mixed.collision_manifest.write_text(yaml.safe_dump(collision, sort_keys=True, allow_unicode=True))
    manifest = yaml.safe_load(mixed.path.read_text())
    manifest["outputs"]["collision_manifest.yaml"] = hashlib.sha256(mixed.collision_manifest.read_bytes()).hexdigest()
    manifest["manifest_sha256"] = ""
    manifest["manifest_sha256"] = hashlib.sha256(
        yaml.safe_dump(manifest, sort_keys=True, allow_unicode=True).encode("utf-8")
    ).hexdigest()
    mixed.path.write_text(yaml.safe_dump(manifest, sort_keys=True, allow_unicode=True))
    with pytest.raises(ArtifactError, match="collision mode"):
        verify_artifacts(mixed.path, URDF)


@pytest.mark.parametrize("selection", [("missing_link",), ("r_thumb_tip",), "Link5_L"])
def test_invalid_collision_selection_does_not_publish(tmp_path: Path, selection):
    output = tmp_path / "generated"
    output.mkdir()
    sentinel = output / "untouched"
    sentinel.write_bytes(b"previous artifacts")
    with pytest.raises(ArtifactError):
        compile_models(URDF, output, raw_collisions=True, decompose_links=selection)
    assert sentinel.read_bytes() == b"previous artifacts"
    assert not (output / "model_manifest.yaml").exists()


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
