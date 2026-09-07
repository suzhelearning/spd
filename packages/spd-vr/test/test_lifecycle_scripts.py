import re
import subprocess

from spd_vr.control_cli import DEFAULT_ENDPOINT as CONTROL_ENDPOINT
from spd_vr.model_builder import workspace_root
from spd_vr.preflight import DEFAULT_ENDPOINT as PREFLIGHT_ENDPOINT
from spd_vr.status_cli import DEFAULT_ENDPOINT as STATUS_ENDPOINT


ROOT = workspace_root()
START = ROOT / "scripts" / "start_spd_vr.sh"
STOP = ROOT / "scripts" / "stop_spd_vr.sh"
ZENOH_ENDPOINT = "tcp/127.0.0.1:8888"


def test_start_script_dry_run_lists_pico2_runtime_graph():
    result = subprocess.run([str(START), "--dry-run"], text=True, capture_output=True, check=False)
    assert result.returncode == 0, result.stderr
    modules = re.findall(r"python -m (spd_vr\.[a-z0-9_]+)", result.stdout)
    assert modules == ["spd_vr.pico2_bridge", "spd_vr.arm_ik", "spd_vr.viewer"]
    assert "--source" not in result.stdout


def test_start_script_passes_adb_serial_to_pico2_bridge():
    result = subprocess.run(
        [str(START), "--dry-run", "--serial", "PICO-1"],
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert "python -m spd_vr.pico2_bridge" in result.stdout
    assert "--adb-serial PICO-1" in result.stdout
    assert "--source" not in result.stdout


def test_start_script_uses_adb_serial_only_for_pico2():
    result = subprocess.run([str(START), "--dry-run", "--serial", "TEST"], text=True, capture_output=True, check=False)
    assert result.returncode == 0, result.stderr
    assert "--adb-serial TEST" in result.stdout
    assert "--device-id" not in result.stdout


def test_all_lifecycle_defaults_use_zenoh_8888():
    start = subprocess.run(
        [str(START), "--dry-run"], text=True, capture_output=True, check=False
    )
    stop = subprocess.run(
        [str(STOP), "--dry-run"], text=True, capture_output=True, check=False
    )

    assert start.returncode == stop.returncode == 0
    assert start.stdout.count(f"--endpoint {ZENOH_ENDPOINT}") == 3
    assert f"spd-control shutdown --endpoint {ZENOH_ENDPOINT}" in stop.stdout
    assert CONTROL_ENDPOINT == PREFLIGHT_ENDPOINT == STATUS_ENDPOINT == ZENOH_ENDPOINT


def test_start_script_rejects_option_as_missing_serial_value():
    result = subprocess.run([str(START), "--serial", "--dry-run"], text=True, capture_output=True, check=False)
    assert result.returncode == 2
    assert "session=" not in result.stdout


def test_scripts_use_fixed_session_windows_and_ordered_shutdown():
    start = START.read_text(encoding="utf-8")
    stop = STOP.read_text(encoding="utf-8")
    assert 'session_name="spd-teleop"' in start
    assert all(name in start for name in ("pico2_bridge", "arm_ik", "viewer"))
    assert start.index("pico2_bridge") < start.index("arm_ik") < start.index("viewer")
    assert "spd-control shutdown" in stop
    assert stop.index("spd-control shutdown") < stop.index("viewer") < stop.index("arm_ik") < stop.index("pico2_bridge")
    assert "--source" not in start
    assert "pkill" not in stop
