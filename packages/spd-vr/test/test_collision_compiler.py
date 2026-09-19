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


def test_source_preparation_welds_stl_without_changing_surface(tmp_path):
    geometry, _ = _mesh_geometry(tmp_path)
    source_bytes = geometry.resolved_path.read_bytes()
    original = trimesh.load_mesh(geometry.resolved_path, process=False)
    prepared, source_hash, _ = collision._source_and_mesh(geometry)

    assert prepared.is_watertight
    np.testing.assert_array_equal(prepared.triangles, original.triangles)
    assert source_hash == hashlib.sha256(source_bytes).hexdigest()
    assert geometry.resolved_path.read_bytes() == source_bytes


def test_source_preparation_does_not_close_real_narrow_gaps():
    left = trimesh.creation.box(extents=(0.01, 0.01, 0.01))
    right = left.copy()
    right.apply_translation((0.01 + 1e-10, 0.0, 0.0))
    triangles = np.concatenate((left.triangles, right.triangles))
    source = trimesh.Trimesh(
        vertices=triangles.reshape(-1, 3),
        faces=np.arange(triangles.size // 3).reshape(-1, 3),
        process=False,
    )
    prepared, _, _ = collision._source_and_mesh(source)

    assert prepared.is_watertight
    assert prepared.body_count == 2
    np.testing.assert_array_equal(prepared.triangles, source.triangles)


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


@pytest.mark.parametrize("width,offset", [(0.5, 0.25), (0.75, 0.125)])
def test_surface_metric_ignores_internal_split_and_overlap_faces(width, offset):
    source = trimesh.creation.box()
    left = trimesh.creation.box(extents=(width, 1.0, 1.0))
    right = left.copy()
    left.apply_translation((-offset, 0.0, 0.0))
    right.apply_translation((offset, 0.0, 0.0))

    assert bidirectional_surface_p95(source, [left, right], samples=256) == pytest.approx(
        0.0, abs=1e-12
    )
    transform = trimesh.transformations.rotation_matrix(0.71, (1.0, 2.0, 3.0))
    transform[:3, 3] = (3.0, -2.0, 5.0)
    for mesh in (source, left, right):
        mesh.apply_transform(transform)
    assert bidirectional_surface_p95(source, [left, right], samples=256) == pytest.approx(
        0.0, abs=1e-12
    )


def test_surface_metric_counts_coincident_exterior_once_and_hides_contained_pieces():
    source = trimesh.creation.box()
    inner = trimesh.creation.box(extents=(0.5, 0.5, 0.5))

    assert bidirectional_surface_p95(source, [source, source, inner], samples=256) == pytest.approx(
        0.0, abs=1e-12
    )
    # A redundant piece must not dilute the area of a genuine exterior defect.
    extra = trimesh.creation.box(extents=(0.5, 0.5, 0.5))
    extra.apply_translation((1.0, 0.0, 0.0))
    assert bidirectional_surface_p95(source, [source] * 15 + [extra], samples=256) > 0.1


def test_surface_metric_detects_exterior_excess_and_missing_volume_in_both_directions():
    source = trimesh.creation.box()
    extra = trimesh.creation.box(extents=(0.5, 0.5, 0.5))
    extra.apply_translation((0.65, 0.0, 0.0))
    assert bidirectional_surface_p95(source, [source, extra], samples=256) > 0.1
    extra.apply_translation((0.35, 0.0, 0.0))

    # Source-to-proxy alone is zero: the added component must fail the reverse.
    assert bidirectional_surface_p95(source, [source, extra], samples=256) > 0.1
    # Proxy-to-source alone is zero: the omitted component must fail the forward.
    larger_source = trimesh.util.concatenate((source, extra))
    assert bidirectional_surface_p95(larger_source, [source], samples=256) > 0.1


def test_surface_metric_rejects_malformed_pieces_even_when_hidden():
    source = trimesh.creation.box()
    inner = trimesh.creation.box(extents=(0.5, 0.5, 0.5))
    open_piece = trimesh.Trimesh(vertices=inner.vertices, faces=inner.faces[:-1], process=False)
    inverted = inner.copy()
    inverted.invert()
    nonconvex = trimesh.creation.icosphere(subdivisions=1, radius=0.2)
    nonconvex.vertices[0] = 0.0

    for malformed in (open_piece, inverted, nonconvex):
        with pytest.raises(collision.CollisionError):
            bidirectional_surface_p95(source, [source, malformed], samples=64)


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

