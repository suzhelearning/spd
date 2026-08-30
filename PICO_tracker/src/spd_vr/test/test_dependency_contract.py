from pathlib import Path


def test_teleop_dependencies_are_pinned():
    text = Path("pixi.toml").read_text(encoding="utf-8")

    assert 'trimesh = ">=4.8,<5"' in text
    assert 'qpoases = ">=3.2,<4"' in text
    assert 'eclipse-zenoh = "==1.10.0"' in text
    assert 'coacd = "==1.0.14"' in text
    assert 'spd-vr = { path = "src/spd_vr", editable = true }' in text
    assert "build-teleop-native" in text
    assert "TIANJI_BUILD_ZENOH_TELEOP=ON" in text
