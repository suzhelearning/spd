from pathlib import Path
import re
import subprocess


ROOT = Path(__file__).resolve().parents[3]
START = ROOT / "scripts" / "start_spd_vr.sh"
STOP = ROOT / "scripts" / "stop_spd_vr.sh"


def test_start_script_dry_run_lists_exact_python_runtime_graph():
    result = subprocess.run([str(START), "--dry-run"], text=True, capture_output=True, check=False)
    assert result.returncode == 0, result.stderr
    modules = re.findall(r"python -m (spd_vr\.[a-z_]+)", result.stdout)
    assert modules == ["spd_vr.pxrea_bridge", "spd_vr.arm_ik", "spd_vr.viewer"]
    assert "ros2" not in result.stdout
    assert "optical_inner" not in result.stdout


def test_start_script_consumes_serial_and_passes_it_to_bridge():
    result = subprocess.run([str(START), "--dry-run", "--serial", "TEST"], text=True, capture_output=True, check=False)
    assert result.returncode == 0, result.stderr
    assert "--device-id TEST" in result.stdout

def test_scripts_use_fixed_session_windows_and_ordered_shutdown():
    start = START.read_text(encoding="utf-8")
    stop = STOP.read_text(encoding="utf-8")
    assert 'session_name="spd-teleop"' in start
    assert all(name in start for name in ("pxrea_bridge", "arm_ik", "viewer"))
    assert start.index("pxrea_bridge") < start.index("arm_ik") < start.index("viewer")
    assert "spd-control shutdown" in stop
    assert stop.index("spd-control shutdown") < stop.index("viewer") < stop.index("arm_ik") < stop.index("pxrea_bridge")
    assert "pkill" not in stop
