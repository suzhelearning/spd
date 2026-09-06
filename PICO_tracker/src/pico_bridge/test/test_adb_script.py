from __future__ import annotations

import os
import signal
import subprocess
import time
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]
ADB_SCRIPT = REPO_ROOT.parent / "adb.sh"


def _fake_adb(tmp_path: Path, devices: str, reverses: str = "") -> tuple[Path, Path]:
    log = tmp_path / "adb.log"
    fake = tmp_path / "adb"
    fake.write_text(
        """#!/usr/bin/env python3
import os
import sys
import time
from pathlib import Path

Path(os.environ["ADB_LOG"]).open("a", encoding="utf-8").write(
    " ".join(sys.argv[1:]) + "\\n"
)
state = Path(os.environ["ADB_REVERSE_STATE"])
args = sys.argv[1:]
if args == ["devices"]:
    print(os.environ["ADB_DEVICES"], end="")
    raise SystemExit(0)
if args[:2] == ["-s", os.environ.get("ADB_EXPECTED_SERIAL", "PICO123")]:
    args = args[2:]
if args == ["get-state"]:
    print("device")
    raise SystemExit(0)
if args[:2] == ["reverse", "--list"]:
    if state.exists():
        print(state.read_text(encoding="utf-8"), end="")
    else:
        print(os.environ.get("ADB_REVERSES", ""), end="")
    raise SystemExit(0)
if args[:2] == ["reverse", "--no-rebind"]:
    pid_file = os.environ.get("ADB_REVERSE_PID_FILE")
    if pid_file:
        Path(pid_file).write_text(str(os.getpid()), encoding="utf-8")
    delay = float(os.environ.get("ADB_REVERSE_DELAY", "0"))
    if delay:
        time.sleep(delay)
    state.parent.mkdir(parents=True, exist_ok=True)
    state.write_text(f"UsbFfs {args[2]} {args[3]}\\n", encoding="utf-8")
    raise SystemExit(0)
if args[:2] == ["reverse", "--remove"]:
    if os.environ.get("ADB_FAIL_REMOVE") == "1":
        raise SystemExit(1)
    state.unlink(missing_ok=True)
    raise SystemExit(0)
if args and args[0] == "reverse":
    raise SystemExit(0)
raise SystemExit(2)
""",
        encoding="utf-8",
    )
    fake.chmod(0o755)
    return fake, log


def _fake_ss(tmp_path: Path) -> Path:
    fake = tmp_path / "ss"
    fake.write_text(
        """#!/usr/bin/env python3
import os
import sys
from pathlib import Path

if sys.argv[1:] == ["-H", "-ltnp"]:
    output_file = os.environ.get("SS_OUTPUT_FILE")
    if output_file:
        print(Path(output_file).read_text(encoding="utf-8"), end="")
    else:
        print(os.environ.get("SS_OUTPUT", ""), end="")
    raise SystemExit(0)
raise SystemExit(2)
""",
        encoding="utf-8",
    )
    fake.chmod(0o755)
    return fake


def _environment(
    tmp_path: Path,
    devices: str,
    reverses: str,
    ss_output: str,
) -> tuple[dict[str, str], Path, Path]:
    fake_adb, log = _fake_adb(tmp_path, devices, reverses)
    fake_ss = _fake_ss(tmp_path)
    state_dir = tmp_path / "state"
    state = state_dir / "reverse.state"
    environment = os.environ.copy()
    environment.update(
        {
            "ADB_BIN": str(fake_adb),
            "ADB_LOG": str(log),
            "ADB_DEVICES": devices,
            "ADB_REVERSES": reverses,
            "ADB_REVERSE_STATE": str(state),
            "ADB_EXPECTED_SERIAL": "PICO123",
            "SS_BIN": str(fake_ss),
            "SS_OUTPUT": ss_output,
            "PICO_ADB_STATE_DIR": str(state_dir),
        }
    )
    return environment, log, state_dir


def _run(
    tmp_path: Path,
    *args: str,
    devices: str = "List of devices attached\nPICO123\tdevice product:pico\n",
    reverses: str = "",
    ss_output: str = "",
) -> subprocess.CompletedProcess[str]:
    environment, _log, _state_dir = _environment(tmp_path, devices, reverses, ss_output)
    return subprocess.run(
        ["bash", str(ADB_SCRIPT), *args],
        cwd=REPO_ROOT,
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )


def test_start_reverses_explicit_device_port_without_kill_server(tmp_path: Path):
    result = _run(
        tmp_path,
        "start",
        "--once",
        "--device-port",
        "63901",
        "--host-port",
        "16391",
    )

    assert result.returncode == 0, result.stderr
    assert "serial=PICO123" in result.stdout
    assert "host_port=16391" in result.stdout
    assert "device_port=63901" in result.stdout
    calls = (tmp_path / "adb.log").read_text(encoding="utf-8").splitlines()
    assert any("reverse --no-rebind tcp:63901 tcp:16391" in call for call in calls)
    assert all("kill-server" not in call for call in calls)


def test_start_requires_serial_when_multiple_devices_are_online(tmp_path: Path):
    devices = "List of devices attached\nPICO123\tdevice\nPICO456\tdevice\n"
    result = _run(tmp_path, "start", "--once", "--device-port", "63901", devices=devices)

    assert result.returncode != 0
    assert "multiple online adb devices" in result.stderr


def test_status_and_stop_touch_only_the_requested_reverse(tmp_path: Path):
    reverses = "PICO123 tcp:63901 tcp:16391\nPICO123 tcp:7000 tcp:17000\n"
    status = _run(
        tmp_path,
        "status",
        "--device-port",
        "63901",
        "--host-port",
        "16391",
        reverses=reverses,
    )
    assert status.returncode == 0, status.stderr
    assert "reversed serial=PICO123 host_port=16391 device_port=63901" in status.stdout

    stop = _run(
        tmp_path,
        "stop",
        "--device-port",
        "63901",
        "--host-port",
        "16391",
        reverses=reverses,
    )
    assert stop.returncode == 0, stop.stderr
    calls = (tmp_path / "adb.log").read_text(encoding="utf-8").splitlines()
    assert any("reverse --remove tcp:63901" in call for call in calls)
    assert all("--remove-all" not in call for call in calls)


def test_status_accepts_adb_transport_label_in_reverse_list(tmp_path: Path):
    reverses = "UsbFfs tcp:63901 tcp:16391\n"
    result = _run(
        tmp_path,
        "status",
        "--device-port",
        "63901",
        "--host-port",
        "16391",
        reverses=reverses,
    )

    assert result.returncode == 0, result.stderr
    assert "reversed serial=PICO123 host_port=16391 device_port=63901" in result.stdout


def test_start_auto_detects_non_loopback_robotics_service_port(tmp_path: Path):
    ss_output = (
        'LISTEN 0 50 0.0.0.0:63901 0.0.0.0:* '
        'users:(("RoboticsService",pid=1,fd=7))\n'
        'LISTEN 0 50 127.0.0.1:60061 0.0.0.0:* '
        'users:(("RoboticsService",pid=1,fd=8))\n'
    )
    result = _run(tmp_path, "start", "--once", ss_output=ss_output)

    assert result.returncode == 0, result.stderr
    assert "host_port=63901" in result.stdout
    assert "device_port=63901" in result.stdout
    calls = (tmp_path / "adb.log").read_text(encoding="utf-8").splitlines()
    assert any("reverse --no-rebind tcp:63901 tcp:63901" in call for call in calls)


def test_missing_pico_is_reported_clearly(tmp_path: Path):
    devices = "List of devices attached\n"
    result = _run(tmp_path, "start", "--once", "--device-port", "63901", devices=devices)

    assert result.returncode != 0
    assert "PICO not connected" in result.stderr


def test_invalid_port_is_rejected_before_adb_access(tmp_path: Path):
    result = _run(tmp_path, "start", "--once", "--device-port", "0")

    assert result.returncode != 0
    assert "port must be an integer in 1..65535" in result.stderr
    assert not (tmp_path / "adb.log").exists()


def test_global_help_works_without_a_subcommand(tmp_path: Path):
    result = _run(tmp_path, "--help")

    assert result.returncode == 0, result.stderr
    assert "Usage:" in result.stdout
    assert not (tmp_path / "adb.log").exists()


def test_default_online_mode_displays_stability_until_ctrl_c(tmp_path: Path):
    ss_output = (
        'LISTEN 0 50 0.0.0.0:63901 0.0.0.0:* '
        'users:(("RoboticsService",pid=1,fd=7))\n'
    )
    environment, log, state_dir = _environment(
        tmp_path,
        "List of devices attached\nPICO123\tdevice\n",
        "",
        ss_output,
    )
    process = subprocess.Popen(
        ["bash", str(ADB_SCRIPT), "--interval", "0.01"],
        cwd=REPO_ROOT,
        env=environment,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    calls = ""
    for _ in range(100):
        if log.exists():
            calls = log.read_text(encoding="utf-8")
            if "get-state" in calls:
                break
        time.sleep(0.01)
    assert "get-state" in calls
    process.send_signal(signal.SIGINT)
    stdout, stderr = process.communicate(timeout=5)

    assert process.returncode == 130, stderr
    assert "stability=" in stdout
    assert "checks=" in stdout
    assert "drops=" in stdout
    assert "stopped" in stdout
    calls = log.read_text(encoding="utf-8").splitlines()
    assert any("reverse --remove tcp:63901" in call for call in calls)


def test_offline_mode_detaches_supervisor_and_stop_cleans_it(tmp_path: Path):
    ss_output = (
        'LISTEN 0 50 0.0.0.0:63901 0.0.0.0:* '
        'users:(("RoboticsService",pid=1,fd=7))\n'
    )
    environment, _log, state_dir = _environment(
        tmp_path,
        "List of devices attached\nPICO123\tdevice\n",
        "",
        ss_output,
    )
    started = subprocess.run(
        [
            "bash",
            str(ADB_SCRIPT),
            "--offline",
            "--interval",
            "30",
            "--device-port",
            "63901",
            "--host-port",
            "16391",
        ],
        cwd=REPO_ROOT,
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )
    assert started.returncode == 0, started.stderr
    pid_file = state_dir / "supervisor.pid"
    for _ in range(50):
        if pid_file.exists():
            break
        time.sleep(0.01)
    assert pid_file.exists()
    pid = int(pid_file.read_text(encoding="utf-8").split()[0])
    assert pid > 0
    supervisor_log = state_dir / "supervisor.log"
    contents = ""
    for _ in range(100):
        if supervisor_log.exists():
            contents = supervisor_log.read_text(encoding="utf-8")
            if "stability=" in contents:
                break
        time.sleep(0.01)
    assert "stability=" in contents

    stop_started = time.monotonic()
    stopped = subprocess.run(
        [
            "bash",
            str(ADB_SCRIPT),
            "stop",
            "--device-port",
            "63901",
            "--host-port",
            "16391",
        ],
        cwd=REPO_ROOT,
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )
    assert time.monotonic() - stop_started < 2
    assert stopped.returncode == 0, stopped.stderr
    assert "stopped" in stopped.stdout
    for _ in range(50):
        if not Path(f"/proc/{pid}").exists():
            break
        time.sleep(0.01)
    assert not Path(f"/proc/{pid}").exists()


def test_offline_mode_keeps_retrying_when_pico_is_disconnected(tmp_path: Path):
    environment, _log, state_dir = _environment(
        tmp_path,
        "List of devices attached\n",
        "",
        "",
    )
    started = subprocess.run(
        [
            "bash",
            str(ADB_SCRIPT),
            "--offline",
            "--device-port",
            "63901",
            "--host-port",
            "16391",
        ],
        cwd=REPO_ROOT,
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )
    assert started.returncode == 0, started.stderr

    pid = int((state_dir / "supervisor.pid").read_text(encoding="utf-8").split()[0])
    supervisor_log = state_dir / "supervisor.log"
    contents = ""
    for _ in range(100):
        if supervisor_log.exists():
            contents = supervisor_log.read_text(encoding="utf-8")
            if "retry_in=1s" in contents:
                break
        time.sleep(0.01)
    assert Path(f"/proc/{pid}").exists()
    assert "PICO not connected" in contents
    assert "retry_in=1s" in contents

    stopped = subprocess.run(
        [
            "bash",
            str(ADB_SCRIPT),
            "stop",
            "--device-port",
            "63901",
            "--host-port",
            "16391",
        ],
        cwd=REPO_ROOT,
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )
    assert stopped.returncode == 0, stopped.stderr
    for _ in range(50):
        if not Path(f"/proc/{pid}").exists():
            break
        time.sleep(0.01)
    assert not Path(f"/proc/{pid}").exists()


def test_foreground_does_not_remove_a_preexisting_reverse(tmp_path: Path):
    environment, log, _state_dir = _environment(
        tmp_path,
        "List of devices attached\nPICO123\tdevice\n",
        "UsbFfs tcp:63901 tcp:16391\n",
        "",
    )
    process = subprocess.Popen(
        [
            "bash",
            str(ADB_SCRIPT),
            "--interval",
            "0.01",
            "--device-port",
            "63901",
            "--host-port",
            "16391",
        ],
        cwd=REPO_ROOT,
        env=environment,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    calls = ""
    for _ in range(100):
        if log.exists():
            calls = log.read_text(encoding="utf-8")
            if "get-state" in calls:
                break
        time.sleep(0.01)
    assert "get-state" in calls

    process.send_signal(signal.SIGINT)
    _stdout, stderr = process.communicate(timeout=5)
    assert process.returncode == 130, stderr
    assert "reverse --remove tcp:63901" not in log.read_text(encoding="utf-8")


def test_stop_does_not_kill_a_process_from_a_stale_pid_file(tmp_path: Path):
    environment, _log, state_dir = _environment(
        tmp_path,
        "List of devices attached\nPICO123\tdevice\n",
        "",
        "",
    )
    sleeper = subprocess.Popen(["sleep", "30"])
    try:
        state_dir.mkdir(parents=True, exist_ok=True)
        (state_dir / "supervisor.pid").write_text(
            f"{sleeper.pid}\n",
            encoding="utf-8",
        )
        stopped = subprocess.run(
            [
                "bash",
                str(ADB_SCRIPT),
                "stop",
                "--device-port",
                "63901",
                "--host-port",
                "16391",
            ],
            cwd=REPO_ROOT,
            env=environment,
            text=True,
            capture_output=True,
            check=False,
        )
        assert stopped.returncode == 0, stopped.stderr
        assert sleeper.poll() is None
    finally:
        sleeper.terminate()
        sleeper.wait(timeout=5)


def test_offline_stop_preserves_a_preexisting_reverse(tmp_path: Path):
    environment, log, state_dir = _environment(
        tmp_path,
        "List of devices attached\nPICO123\tdevice\n",
        "UsbFfs tcp:63901 tcp:16391\n",
        "",
    )
    started = subprocess.run(
        [
            "bash",
            str(ADB_SCRIPT),
            "--offline",
            "--interval",
            "0.01",
            "--device-port",
            "63901",
            "--host-port",
            "16391",
        ],
        cwd=REPO_ROOT,
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )
    assert started.returncode == 0, started.stderr
    supervisor_log = state_dir / "supervisor.log"
    for _ in range(100):
        if supervisor_log.exists() and "state=online" in supervisor_log.read_text(
            encoding="utf-8"
        ):
            break
        time.sleep(0.01)

    stopped = subprocess.run(
        [
            "bash",
            str(ADB_SCRIPT),
            "stop",
            "--device-port",
            "63901",
            "--host-port",
            "16391",
        ],
        cwd=REPO_ROOT,
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )
    assert stopped.returncode == 0, stopped.stderr
    assert "reverse --remove tcp:63901" not in log.read_text(encoding="utf-8")


def test_explicit_stop_reports_reverse_remove_failure(tmp_path: Path):
    environment, _log, _state_dir = _environment(
        tmp_path,
        "List of devices attached\nPICO123\tdevice\n",
        "UsbFfs tcp:63901 tcp:16391\n",
        "",
    )
    environment["ADB_FAIL_REMOVE"] = "1"
    stopped = subprocess.run(
        [
            "bash",
            str(ADB_SCRIPT),
            "stop",
            "--device-port",
            "63901",
            "--host-port",
            "16391",
        ],
        cwd=REPO_ROOT,
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )
    assert stopped.returncode != 0
    assert "reverse --remove failed" in stopped.stderr


def test_monitor_migrates_an_owned_reverse_when_service_port_changes(tmp_path: Path):
    first_listener = (
        'LISTEN 0 50 0.0.0.0:63901 0.0.0.0:* '
        'users:(("RoboticsService",pid=1,fd=7))\n'
    )
    second_listener = (
        'LISTEN 0 50 0.0.0.0:63902 0.0.0.0:* '
        'users:(("RoboticsService",pid=1,fd=7))\n'
    )
    environment, log, state_dir = _environment(
        tmp_path,
        "List of devices attached\nPICO123\tdevice\n",
        "",
        first_listener,
    )
    ss_output_file = tmp_path / "ss.out"
    ss_output_file.write_text(first_listener, encoding="utf-8")
    environment["SS_OUTPUT_FILE"] = str(ss_output_file)
    reverse_state = state_dir / "reverse.state"
    process = subprocess.Popen(
        ["bash", str(ADB_SCRIPT), "--interval", "0.01"],
        cwd=REPO_ROOT,
        env=environment,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    for _ in range(100):
        if reverse_state.exists() and "tcp:63901 tcp:63901" in reverse_state.read_text(
            encoding="utf-8"
        ):
            break
        time.sleep(0.01)
    assert reverse_state.exists()
    ss_output_file.write_text(second_listener, encoding="utf-8")
    for _ in range(200):
        if reverse_state.exists() and "tcp:63902 tcp:63902" in reverse_state.read_text(
            encoding="utf-8"
        ):
            break
        time.sleep(0.01)
    assert "tcp:63902 tcp:63902" in reverse_state.read_text(encoding="utf-8")

    process.send_signal(signal.SIGINT)
    _stdout, stderr = process.communicate(timeout=5)
    assert process.returncode == 130, stderr
    calls = log.read_text(encoding="utf-8")
    assert "reverse --remove tcp:63901" in calls
    assert "reverse --remove tcp:63902" in calls


def test_offline_and_once_are_mutually_exclusive(tmp_path: Path):
    result = _run(
        tmp_path,
        "--offline",
        "--once",
        "--device-port",
        "63901",
        "--host-port",
        "16391",
    )

    assert result.returncode != 0
    assert "mutually exclusive" in result.stderr
    assert not (tmp_path / "state" / "supervisor.pid").exists()


def test_concurrent_offline_starts_publish_only_one_supervisor(tmp_path: Path):
    environment, _log, state_dir = _environment(
        tmp_path,
        "List of devices attached\nPICO123\tdevice\n",
        "",
        "",
    )
    command = [
        "bash",
        str(ADB_SCRIPT),
        "--offline",
        "--device-port",
        "63901",
        "--host-port",
        "16391",
    ]
    first = subprocess.Popen(
        command,
        cwd=REPO_ROOT,
        env=environment,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    second = subprocess.Popen(
        command,
        cwd=REPO_ROOT,
        env=environment,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    first_output = first.communicate(timeout=5)
    second_output = second.communicate(timeout=5)
    try:
        assert sorted((first.returncode, second.returncode)) == [0, 2], (
            first_output,
            second_output,
        )
        pid = int(
            (state_dir / "supervisor.pid").read_text(encoding="utf-8").split()[0]
        )
        assert Path(f"/proc/{pid}").exists()
    finally:
        subprocess.run(
            [
                "bash",
                str(ADB_SCRIPT),
                "stop",
                "--device-port",
                "63901",
                "--host-port",
                "16391",
            ],
            cwd=REPO_ROOT,
            env=environment,
            text=True,
            capture_output=True,
            check=False,
        )


def test_offline_stop_terminates_a_slow_reverse_child(tmp_path: Path):
    environment, _log, state_dir = _environment(
        tmp_path,
        "List of devices attached\nPICO123\tdevice\n",
        "",
        "",
    )
    reverse_pid_file = tmp_path / "reverse.pid"
    environment["ADB_REVERSE_DELAY"] = "30"
    environment["ADB_REVERSE_PID_FILE"] = str(reverse_pid_file)
    started = subprocess.run(
        [
            "bash",
            str(ADB_SCRIPT),
            "--offline",
            "--device-port",
            "63901",
            "--host-port",
            "16391",
        ],
        cwd=REPO_ROOT,
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )
    assert started.returncode == 0, started.stderr

    for _ in range(100):
        if reverse_pid_file.exists() and (state_dir / "supervisor.reverse").exists():
            break
        time.sleep(0.01)
    assert reverse_pid_file.exists()
    reverse_pid = int(reverse_pid_file.read_text(encoding="utf-8"))
    try:
        stop_started = time.monotonic()
        stopped = subprocess.run(
            [
                "bash",
                str(ADB_SCRIPT),
                "stop",
                "--device-port",
                "63901",
                "--host-port",
                "16391",
            ],
            cwd=REPO_ROOT,
            env=environment,
            text=True,
            capture_output=True,
            check=False,
        )
        assert time.monotonic() - stop_started < 2
        assert stopped.returncode == 0, stopped.stderr
        assert not Path(f"/proc/{reverse_pid}").exists()
        assert not (state_dir / "reverse.state").exists()
    finally:
        if Path(f"/proc/{reverse_pid}").exists():
            os.kill(reverse_pid, signal.SIGKILL)
