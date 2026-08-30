from pathlib import Path


def test_python_teleop_dependencies_are_pinned():
    text = Path("pixi.toml").read_text(encoding="utf-8")
    assert 'osqp = ">=1,<2"' in text
    assert 'eclipse-zenoh = "==1.10.0"' in text
    assert 'coacd = "==1.0.14"' in text
    assert 'trimesh = ">=4.8,<5"' in text
    assert 'spd-vr = { path = "src/spd_vr", editable = true }' in text
    assert 'wuji-retargeting = { path = "../wuji-retargeting", editable = true }' in text
    for forbidden in ("qpoases", "zenoh-pico", "build-teleop-native", "TIANJI_BUILD_ZENOH_TELEOP"):
        assert forbidden not in text
