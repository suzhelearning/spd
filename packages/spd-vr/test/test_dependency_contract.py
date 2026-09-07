from pathlib import Path


def test_root_pixi_workspace_uses_only_retained_python_packages():
    text = Path("pixi.toml").read_text(encoding="utf-8")
    assert 'osqp = ">=1,<2"' in text
    assert 'eclipse-zenoh = "==1.10.0"' in text
    assert 'coacd = "==1.0.14"' in text
    assert 'trimesh = ">=4.8,<5"' in text
    assert 'spd-vr = { path = "packages/spd-vr", editable = true }' in text
    assert 'pico-hand-tracking = { path = "packages/pico-hand-tracking", editable = true }' in text
    assert 'wuji-retargeting = { path = "packages/wuji-retargeting", editable = true }' in text
    assert 'spd-teleop = "bash scripts/start_spd_vr.sh"' in text
    for forbidden in (
        "qpoases",
        "zenoh-pico",
        "build-teleop-native",
        "TIANJI_BUILD_ZENOH_TELEOP",
    ):
        assert forbidden not in text
