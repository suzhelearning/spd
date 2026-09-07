from __future__ import annotations

import hashlib
import json
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import threading
import time

import numpy as np
import pytest
import trimesh

from spd_vr.model_compiler import collision
from spd_vr.model_compiler.collision import (
    CollisionSettings,
    _canonical_mesh_arrays,
    _canonical_mesh_bytes,
    bidirectional_surface_p95,
    decompose_mesh,
)
from spd_vr.model_compiler.urdf_model import MeshGeometry


def _mesh_geometry(tmp_path: Path) -> tuple[MeshGeometry, trimesh.Trimesh]:
    mesh = trimesh.creation.box(extents=(0.04, 0.03, 0.02))
    source = tmp_path / "hand.stl"
    mesh.export(source)
    return MeshGeometry("hand.stl", source), mesh


def _fake_decompose(_mesh, **kwargs):
    assert kwargs["seed"] == 0
    assert kwargs["max_convex_hull"] == 16
    assert kwargs["max_ch_vertex"] == 64
    mesh = trimesh.creation.box(extents=(0.04, 0.03, 0.02))
    return [(mesh.vertices.copy(), mesh.faces.copy())]


def test_decompose_is_deterministic_and_rebuilds_corrupt_cache(tmp_path, monkeypatch):
    geometry, _ = _mesh_geometry(tmp_path)
    monkeypatch.setattr(collision.coacd, "run_coacd", _fake_decompose)
    settings = CollisionSettings(surface_samples=32)

    first = decompose_mesh(geometry, settings, tmp_path / "cache")
    second = decompose_mesh(geometry, settings, tmp_path / "cache")
    assert first.cache_key == second.cache_key
    assert first.pieces == second.pieces
    assert first.piece_sha256 == second.piece_sha256
    assert first.cache_hit is False
    assert second.cache_hit is True
    assert hashlib.sha256(first.pieces[0].read_bytes()).hexdigest() == first.piece_sha256[0]


    manifest_path = first.pieces[0].parent / "manifest.json"
    manifest = json.loads(manifest_path.read_text())
    manifest["pieces"] = []
    manifest_path.write_text(json.dumps(manifest))
    rebuilt_empty = decompose_mesh(geometry, settings, tmp_path / "cache")
    assert rebuilt_empty.cache_hit is False

    rebuilt_empty.pieces[0].write_bytes(b"corrupt")
    rebuilt = decompose_mesh(geometry, settings, tmp_path / "cache")
    assert rebuilt.cache_hit is False
    assert rebuilt.piece_sha256 == first.piece_sha256
    assert rebuilt.pieces[0].read_bytes() != b"corrupt"


def test_canonical_piece_hash_normalizes_cyclic_face_starts():
    mesh = trimesh.creation.box()
    rotated = np.roll(mesh.faces, 1, axis=1)
    assert np.array_equal(_canonical_mesh_arrays(mesh.vertices, mesh.faces)[1],
                          _canonical_mesh_arrays(mesh.vertices, rotated)[1])
    assert _canonical_mesh_bytes((mesh.vertices, mesh.faces)) == _canonical_mesh_bytes(
        (mesh.vertices, rotated)
    )


def test_fixed_coacd_parameters_cannot_be_overridden():
    with pytest.raises(ValueError, match="fixed"):
        CollisionSettings(_extra_coacd_params=(("seed", 7),))


def test_same_key_concurrent_misses_build_once(tmp_path, monkeypatch):
    geometry, _ = _mesh_geometry(tmp_path)
    calls = 0
    calls_lock = threading.Lock()

    def slow_decompose(_mesh, **kwargs):
        nonlocal calls
        with calls_lock:
            calls += 1
        time.sleep(0.05)
        return _fake_decompose(_mesh, **kwargs)

    monkeypatch.setattr(collision.coacd, "run_coacd", slow_decompose)
    settings = CollisionSettings(surface_samples=16)
    with ThreadPoolExecutor(max_workers=2) as pool:
        results = list(pool.map(lambda _: decompose_mesh(geometry, settings, tmp_path / "cache"), range(2)))
    assert calls == 1
    assert {result.cache_hit for result in results} == {False, True}
    assert all(path.exists() for path in results[0].pieces)


def test_decompose_fails_closed_for_bad_output_and_exception(tmp_path, monkeypatch):
    geometry, _ = _mesh_geometry(tmp_path)
    settings = CollisionSettings(surface_samples=16)
    valid = trimesh.creation.box()
    planar = np.array(
        [[[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]],
        dtype=float,
    )

    bad_results = [
        ([], "empty"),
        ([(trimesh.creation.icosphere(subdivisions=2).vertices, valid.faces)], "vertices"),
        ([(planar[0], np.array([[0, 1, 2]], dtype=np.int32))], "volume"),
        ([(np.array([[np.nan, 0.0, 0.0]] * 8), valid.faces)], "non-finite"),
    ]
    for result, message in bad_results:
        monkeypatch.setattr(collision.coacd, "run_coacd", lambda *_a, result=result, **_k: result)
        with pytest.raises(collision.CollisionError, match=message):
            decompose_mesh(geometry, settings, tmp_path / message)

    monkeypatch.setattr(
        collision.coacd,
        "run_coacd",
        lambda *_a, **_k: [(valid.vertices, valid.faces)] * 17,
    )
    with pytest.raises(collision.CollisionError, match="pieces"):
        decompose_mesh(geometry, settings, tmp_path / "pieces")

    monkeypatch.setattr(collision.coacd, "run_coacd", lambda *_a, **_k: (_ for _ in ()).throw(RuntimeError("boom")))
    with pytest.raises(collision.CollisionError, match="CoACD"):
        decompose_mesh(geometry, settings, tmp_path / "exception")


def test_bidirectional_surface_p95_is_deterministic_and_two_way():
    source = trimesh.creation.box(extents=(1.0, 1.0, 1.0))
    same = trimesh.creation.box(extents=(1.0, 1.0, 1.0))
    shifted = trimesh.creation.box(extents=(1.0, 1.0, 1.0))
    shifted.apply_translation([0.01, 0.0, 0.0])

    assert bidirectional_surface_p95(source, [same], samples=64) == pytest.approx(0.0)
    assert bidirectional_surface_p95(source, [shifted], samples=64) > 0.0015
    assert bidirectional_surface_p95(source, [shifted], samples=64) == bidirectional_surface_p95(
        source, [shifted], samples=64
    )


def test_quality_gate_uses_hand_and_arm_limits(tmp_path, monkeypatch):
    geometry, _ = _mesh_geometry(tmp_path)
    shifted = trimesh.creation.box(extents=(0.04, 0.03, 0.02))
    shifted.apply_translation([0.002, 0.0, 0.0])
    monkeypatch.setattr(
        collision.coacd,
        "run_coacd",
        lambda *_a, **_k: [(shifted.vertices, shifted.faces)],
    )
    with pytest.raises(collision.CollisionError, match="surface p95"):
        decompose_mesh(geometry, CollisionSettings(surface_samples=32), tmp_path / "hand")
    arm = CollisionSettings(surface_samples=32, surface_p95_threshold_m=0.003)
    artifact = decompose_mesh(geometry, arm, tmp_path / "arm")
    assert artifact.surface_p95 <= 0.003


def test_real_coacd_small_mesh_smoke(tmp_path):
    geometry, _ = _mesh_geometry(tmp_path)
    settings = CollisionSettings(
        resolution=100,
        preprocess_resolution=20,
        mcts_nodes=2,
        mcts_iterations=2,
        mcts_max_depth=1,
        surface_samples=16,
        surface_p95_threshold_m=0.01,
    )
    artifact = decompose_mesh(geometry, settings, tmp_path / "real")
    assert artifact.pieces
    assert all(path.exists() for path in artifact.pieces)

