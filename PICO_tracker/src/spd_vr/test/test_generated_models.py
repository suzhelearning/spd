from pathlib import Path

import numpy as np
import pytest

from spd_vr.model_compiler.artifacts import compile_models, verify_artifacts
from spd_vr.manifest import load_manifest
from spd_vr.model_compiler.collision import CollisionArtifact, _canonical_mesh_bytes


URDF = Path(__file__).resolve().parents[4] / "assets" / "tianji_wuji2" / "tianji_wuji2.urdf"


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
    assert all(mujoco.mj_name2id(arm, mujoco.mjtObj.mjOBJ_SITE, name) >= 0 for name in ("l_wrist_target", "r_wrist_target"))
    manifest = yaml.safe_load(result.path.read_text(encoding="utf-8"))
    assert load_manifest(result.path)["dof"] == 54
    assert len(manifest["axis_visuals"]) == 24
    assert len(manifest["collision"]["adjacent_excludes"]) == 79
    assert manifest["source"]["meshes"]
    assert not Path(manifest["source"]["urdf"]).is_absolute()
    assert all(not Path(record["path"]).is_absolute() for record in manifest["visual_meshes"])


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
