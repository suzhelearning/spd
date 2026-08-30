"""Fail-closed checks for the Python teleoperation lifecycle.

The checks only inspect local state. In particular, the ADB probe uses
``adb reverse --list`` and never creates or tears down a device connection.
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

from .model_compiler.artifacts import verify_artifacts


SESSION_NAME = "spd-teleop"
DEFAULT_ENDPOINT = "tcp/127.0.0.1:7447"
DEFAULT_SDK_LIBRARY = "/opt/apps/roboticsservice/SDK/x64/libPXREARobotSDK.so"
ARTIFACT_FILES = (
    "unified_plant.xml",
    "arm_ik.xml",
    "model_manifest.yaml",
    "collision_manifest.yaml",
    "actuator_calibration.yaml",
)
DEPENDENCIES = ("mujoco", "osqp", "coacd", "zenoh")


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


def _check_adb_reverse(run_command: RunCommand) -> CheckResult:
    try:
        result = run_command(["adb", "reverse", "--list"])
    except OSError as exc:
        return CheckResult("adb_reverse", False, f"adb unavailable: {exc}")
    if result.returncode != 0:
        detail = (result.stderr or result.stdout or "adb reverse --list failed").strip()
        return CheckResult("adb_reverse", False, detail)
    detail = (result.stdout or "").strip() or "ADB reverse is available"
    return CheckResult("adb_reverse", True, detail)


def _check_sdk(path: Path, loader: Callable[[str], Any]) -> CheckResult:
    try:
        client = loader(str(path))
        close = getattr(client, "close", None)
        if close is not None:
            close()
    except Exception as exc:  # SDK errors vary by ctypes/platform loader.
        return CheckResult("sdk", False, f"cannot load {path}: {exc}")
    return CheckResult("sdk", True, f"loaded {path}")


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
        return CheckResult("port_7447", False, str(exc))
    return CheckResult("port_7447", bool(ok), str(detail))


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
    sdk_library: str | Path | None = None,
    manifest_path: str | Path | None = None,
    urdf_path: str | Path | None = None,
    endpoint: str = DEFAULT_ENDPOINT,
    session_name: str = SESSION_NAME,
    run_command: RunCommand | None = None,
    sdk_loader: Callable[[str], Any] | None = None,
    dependency_loader: Callable[[str], Any] | None = None,
    display_env: Mapping[str, str] | None = None,
    port_checker: Callable[[str], tuple[bool, str]] | None = None,
    session_checker: Callable[[str], tuple[bool, str]] | None = None,
    artifact_checker: Callable[[str | Path, str | Path], Any] | None = None,
) -> list[CheckResult]:
    """Run all required checks without starting any physical device."""
    root = Path(repo_root) if repo_root is not None else Path(__file__).resolve().parents[3]
    sdk = Path(sdk_library) if sdk_library is not None else Path(os.environ.get("PXREA_SDK_LIBRARY", DEFAULT_SDK_LIBRARY))
    manifest = Path(manifest_path) if manifest_path is not None else root / "generated" / "model_manifest.yaml"
    urdf = Path(urdf_path) if urdf_path is not None else root.parent / "assets" / "tianji_wuji2" / "tianji_wuji2.urdf"
    command = _run_command if run_command is None else run_command
    load_sdk = sdk_loader
    if load_sdk is None:
        from .pxrea_sdk import PXREAClient

        load_sdk = lambda path: PXREAClient.load_library(path)
    load_dependency = importlib.import_module if dependency_loader is None else dependency_loader
    environment = os.environ if display_env is None else display_env
    check_session = (lambda name: _check_session(name, command)) if session_checker is None else session_checker
    check_artifact = verify_artifacts if artifact_checker is None else artifact_checker

    results = [
        _check_adb_reverse(command),
        _check_sdk(sdk, load_sdk),
        _check_dependencies(load_dependency),
        _check_display(environment),
        _check_artifacts(manifest, urdf, check_artifact),
        _check_port(endpoint, port_checker),
    ]
    try:
        session_ok, session_detail = check_session(session_name)
    except Exception as exc:
        session_ok, session_detail = False, str(exc)
    results.append(CheckResult("session", bool(session_ok), str(session_detail)))
    return results


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=None)
    parser.add_argument("--sdk-library", type=Path, default=None)
    parser.add_argument("--manifest", type=Path, default=None)
    parser.add_argument("--urdf", type=Path, default=None)
    parser.add_argument("--endpoint", default=DEFAULT_ENDPOINT)
    parser.add_argument("--session", default=SESSION_NAME)
    args = parser.parse_args(argv)
    results = run_checks(
        repo_root=args.repo_root,
        sdk_library=args.sdk_library,
        manifest_path=args.manifest,
        urdf_path=args.urdf,
        endpoint=args.endpoint,
        session_name=args.session,
    )
    for result in results:
        print(json.dumps(result.as_dict(), sort_keys=True))
    return 0 if all(result.ok for result in results) else 1


__all__ = ["ARTIFACT_FILES", "CheckResult", "DEFAULT_ENDPOINT", "SESSION_NAME", "main", "run_checks"]

if __name__ == "__main__":
    raise SystemExit(main())
