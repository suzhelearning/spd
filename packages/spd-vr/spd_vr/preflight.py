"""Fail-closed PICO_2 preflight checks for the Python teleoperation lifecycle.

The ADB probe establishes the forward used by the direct PICO_2 hand stream.
"""

from __future__ import annotations

import argparse
import importlib
import json
import os
import shutil
import socket
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Mapping, Sequence

from .defaults import DEFAULT_ZENOH_ENDPOINT
from .model_compiler.artifacts import verify_artifacts
from .model_builder import workspace_root


SESSION_NAME = "spd-teleop"
DEFAULT_ENDPOINT = DEFAULT_ZENOH_ENDPOINT
ARTIFACT_FILES = (
    "model_manifest.yaml",
    "unified_plant.xml",
    "arm_ik.xml",
    "collision_manifest.yaml",
    "actuator_calibration.yaml",
)
DEPENDENCIES = ("mujoco", "osqp", "coacd", "zenoh", "pico_hand_tracking")


@dataclass(frozen=True, slots=True)
class CheckResult:
    """One preflight result suitable for JSON serialization."""

    name: str
    ok: bool
    detail: str

    def as_dict(self) -> dict[str, Any]:
        return {"name": self.name, "ok": self.ok, "detail": self.detail}


RunCommand = Callable[..., subprocess.CompletedProcess[str]]


def _run_command(command: Sequence[str], **kwargs: Any) -> subprocess.CompletedProcess[str]:
    return subprocess.run(command, text=True, capture_output=True, check=False, **kwargs)


def _endpoint_host_port(endpoint: str) -> tuple[str, int]:
    value = str(endpoint)
    if value.startswith("tcp/"):
        value = value[4:]
    host, separator, port = value.rpartition(":")
    if not separator or not host or not port.isdigit():
        raise ValueError(f"unsupported endpoint: {endpoint}")
    return host.strip("[]"), int(port)


def _adb_command(run_command: RunCommand, command: Sequence[str]) -> tuple[bool, str]:
    try:
        result = run_command(command)
    except OSError as exc:
        return False, f"adb unavailable: {exc}"
    if result.returncode != 0:
        return False, (result.stderr or result.stdout or "adb command failed").strip()
    return True, (result.stdout or "").strip()





def _check_pico2_adb(
    run_command: RunCommand,
    *,
    selected_serial: str | None = None,
    local_port: int = 10002,
    device_port: int | None = None,
) -> list[CheckResult]:
    """Check the ADB forward used by the direct PICO_2 hand stream."""
    ok, devices = _adb_command(run_command, ["adb", "devices"])
    if not ok:
        return [
            CheckResult("pico_device", False, devices),
            CheckResult("adb_forward", False, devices),
        ]
    online = [
        line.split()[0]
        for line in devices.splitlines()[1:]
        if len(line.split()) >= 2 and line.split()[1] == "device"
    ]
    serial = (
        selected_serial
        if selected_serial in online
        else (online[0] if len(online) == 1 and selected_serial is None else None)
    )
    device_result = CheckResult(
        "pico_device",
        bool(serial),
        f"online PICO: {serial}"
        if serial
        else "selected PICO is not online or no uniquely selected online PICO",
    )
    if not serial:
        return [device_result, CheckResult("adb_forward", False, "no selected PICO")]
    target_port = local_port if device_port is None else int(device_port)
    ok, detail = _adb_command(
        run_command,
        [
            "adb",
            "-s",
            serial,
            "forward",
            f"tcp:{local_port}",
            f"tcp:{target_port}",
        ],
    )
    return [
        device_result,
        CheckResult(
            "adb_forward",
            ok,
            detail or f"tcp:{local_port} -> tcp:{target_port}",
        ),
    ]




def _check_dependencies(loader: Callable[[str], Any]) -> CheckResult:
    missing: list[str] = []
    for name in DEPENDENCIES:
        try:
            loader(name)
        except Exception as exc:
            missing.append(f"{name}: {exc}")
    if missing:
        return CheckResult("python_dependencies", False, "; ".join(missing))
    return CheckResult("python_dependencies", True, ", ".join(DEPENDENCIES))


def _check_display(environment: Mapping[str, str]) -> CheckResult:
    if environment.get("DISPLAY") or environment.get("WAYLAND_DISPLAY"):
        value = environment.get("DISPLAY") or environment.get("WAYLAND_DISPLAY")
        return CheckResult("display", True, str(value))
    return CheckResult("display", False, "DISPLAY or WAYLAND_DISPLAY is not set")




def _check_port(
    endpoint: str,
    checker: Callable[[str], tuple[bool, str]] | None = None,
) -> CheckResult:
    try:
        ok, detail = checker(endpoint) if checker is not None else _port_is_free(endpoint)
    except Exception as exc:
        return CheckResult("zenoh_endpoint", False, str(exc))
    return CheckResult("zenoh_endpoint", bool(ok), str(detail))


def _port_is_free(endpoint: str) -> tuple[bool, str]:
    host, port = _endpoint_host_port(endpoint)
    family = socket.AF_INET6 if ":" in host else socket.AF_INET
    with socket.socket(family, socket.SOCK_STREAM) as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind((host, port))
        except OSError as exc:
            return False, f"{endpoint} is unavailable: {exc}"
    return True, f"{endpoint} is free"


def _check_session(
    session_name: str,
    run_command: RunCommand,
) -> tuple[bool, str]:
    if shutil.which("tmux") is None:
        return False, "tmux is unavailable"
    try:
        result = run_command(["tmux", "has-session", "-t", session_name])
    except OSError as exc:
        return False, f"tmux unavailable: {exc}"
    if result.returncode == 0:
        return False, f"session already exists: {session_name}"
    return True, f"session is absent: {session_name}"
def _check_artifacts(
    manifest_path: Path,
    urdf_path: Path,
    checker: Callable[[str | Path, str | Path], Any],
) -> CheckResult:
    missing = [name for name in ARTIFACT_FILES if not (manifest_path.parent / name).is_file()]
    if missing and checker is verify_artifacts:
        return CheckResult("artifacts", False, "missing generated artifacts: " + ", ".join(missing))
    if not urdf_path.is_file() and checker is verify_artifacts:
        return CheckResult("artifacts", False, f"missing authoritative URDF: {urdf_path}")
    try:
        result = checker(manifest_path, urdf_path)
        if isinstance(result, tuple) and len(result) == 2:
            ok, detail = bool(result[0]), str(result[1])
            return CheckResult("artifacts", ok, detail)
        if result is False:
            return CheckResult("artifacts", False, "artifact checker rejected generated files")
        return CheckResult("artifacts", True, f"verified {manifest_path.parent}")
    except Exception as exc:
        return CheckResult("artifacts", False, str(exc))


def run_checks(
    *,
    repo_root: str | Path | None = None,
    manifest_path: str | Path | None = None,
    urdf_path: str | Path | None = None,
    endpoint: str = DEFAULT_ENDPOINT,
    session_name: str = SESSION_NAME,
    selected_serial: str | None = None,
    fake_source_path: str | Path | None = None,
    pico2_port: int = 10002,
    pico2_device_port: int | None = None,
    run_command: RunCommand | None = None,
    dependency_loader: Callable[[str], Any] | None = None,
    display_env: Mapping[str, str] | None = None,
    port_checker: Callable[[str], tuple[bool, str]] | None = None,
    session_checker: Callable[[str], tuple[bool, str]] | None = None,
    artifact_checker: Callable[[str | Path, str | Path], Any] | None = None,
) -> list[CheckResult]:
    root = Path(repo_root).resolve() if repo_root is not None else workspace_root()
    package_root = Path(__file__).resolve().parents[1]
    manifest = (
        Path(manifest_path)
        if manifest_path is not None
        else package_root / "generated" / "model_manifest.yaml"
    )
    urdf = (
        Path(urdf_path)
        if urdf_path is not None
        else root / "assets" / "tianji_wuji2" / "tianji_wuji2.urdf"
    )
    command = _run_command if run_command is None else run_command
    load_dependency = importlib.import_module if dependency_loader is None else dependency_loader
    check_session = (lambda name: _check_session(name, command)) if session_checker is None else session_checker
    environment = os.environ if display_env is None else display_env
    check_artifact = verify_artifacts if artifact_checker is None else artifact_checker
    if fake_source_path is None:
        results = _check_pico2_adb(
            command,
            selected_serial=selected_serial,
            local_port=pico2_port,
            device_port=pico2_device_port,
        )
        results.extend((_check_dependencies(load_dependency), _check_display(environment)))
    else:
        fake_source = Path(fake_source_path)
        results = [
            CheckResult(
                "fake_source",
                fake_source.is_file(),
                f"missing fake source: {fake_source}"
                if not fake_source.is_file()
                else f"readable {fake_source}",
            )
        ]
        results.append(_check_dependencies(load_dependency))
    results.extend(
        (_check_artifacts(manifest, urdf, check_artifact), _check_port(endpoint, port_checker))
    )
    try:
        session_ok, session_detail = check_session(session_name)
    except Exception as exc:
        session_ok, session_detail = False, str(exc)
    results.append(CheckResult("session", bool(session_ok), str(session_detail)))
    return results


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=None)
    parser.add_argument("--fake-source", type=Path, default=None)
    parser.add_argument("--manifest", type=Path, default=None)
    parser.add_argument("--urdf", type=Path, default=None)
    parser.add_argument("--endpoint", default=DEFAULT_ENDPOINT)
    parser.add_argument("--serial", default=None)
    parser.add_argument("--pico2-port", type=int, default=10002)
    parser.add_argument("--pico2-device-port", type=int, default=None)
    parser.add_argument("--session", default=SESSION_NAME)
    args = parser.parse_args(argv)
    results = run_checks(
        repo_root=args.repo_root,
        manifest_path=args.manifest,
        urdf_path=args.urdf,
        endpoint=args.endpoint,
        session_name=args.session,
        selected_serial=args.serial,
        fake_source_path=args.fake_source,
        pico2_port=args.pico2_port,
        pico2_device_port=args.pico2_device_port,
    )
    for result in results:
        print(json.dumps(result.as_dict(), sort_keys=True))
    return 0 if all(result.ok for result in results) else 1


__all__ = ["ARTIFACT_FILES", "CheckResult", "DEFAULT_ENDPOINT", "SESSION_NAME", "main", "run_checks"]

if __name__ == "__main__":
    raise SystemExit(main())
