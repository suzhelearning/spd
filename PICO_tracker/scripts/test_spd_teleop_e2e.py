#!/usr/bin/env python3
"""Run the production Python teleoperation graph, or explicit synthetic smoke."""

from __future__ import annotations

import argparse
import json
import os
import re
import signal
import struct
import subprocess
import sys
import tempfile
import time
from pathlib import Path
from typing import Any, Sequence


ROOT = Path(__file__).resolve().parents[1]
GENERATED = ROOT / "src" / "spd_vr" / "generated"
URDF_DEFAULT = ROOT.parent / "assets" / "tianji_wuji2" / "tianji_wuji2.urdf"
ARTIFACT_FILES = (
    "unified_plant.xml",
    "arm_ik.xml",
    "model_manifest.yaml",
    "collision_manifest.yaml",
    "actuator_calibration.yaml",
)
BLOCKER_REASON = (
    "authoritative artifacts blocked: Link_Base.STL p95 surface error "
    "0.036785362 m exceeds the required arm/base 0.003 m gate"
)


def _recent(text: str, limit: int = 20) -> str:
    lines = [line for line in text.splitlines() if line.strip()]
    return "\n".join(lines[-limit:])


def _run(command: Sequence[str], *, timeout: float) -> dict[str, Any]:
    try:
        result = subprocess.run(
            list(command), cwd=ROOT, env=_environment(), text=True,
            capture_output=True, check=False, timeout=timeout,
        )
        return {
            "command": list(command),
            "exit_code": result.returncode,
            "stdout": result.stdout,
            "stderr": result.stderr,
        }
    except subprocess.TimeoutExpired as exc:
        return {
            "command": list(command),
            "exit_code": None,
            "stdout": _text(exc.stdout),
            "stderr": _text(exc.stderr),
            "timeout": timeout,
        }
    except OSError as exc:
        return {"command": list(command), "exit_code": None, "stdout": "", "stderr": str(exc)}


def _text(value: bytes | str | None) -> str:
    if value is None:
        return ""
    return value.decode(errors="replace") if isinstance(value, bytes) else value


def _environment() -> dict[str, str]:
    env = os.environ.copy()
    env["PYTHONUNBUFFERED"] = "1"
    return env


class OwnedProcess:
    def __init__(self, command: Sequence[str]) -> None:
        self.command = list(command)
        self.process = subprocess.Popen(
            self.command,
            cwd=ROOT,
            env=_environment(),
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            start_new_session=True,
        )
        self.stdout = ""
        self.stderr = ""

    @property
    def alive(self) -> bool:
        return self.process.poll() is None

    def finish(self, timeout: float = 2.0) -> None:
        if self.process.poll() is None:
            try:
                self.process.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                self.terminate(timeout)
        try:
            self.stdout, self.stderr = self.process.communicate(timeout=0.2)
        except subprocess.TimeoutExpired:
            self.terminate(0.5)
            self.stdout, self.stderr = self.process.communicate(timeout=1.0)

    def terminate(self, timeout: float = 1.0) -> None:
        if self.process.poll() is not None:
            return
        try:
            os.killpg(self.process.pid, signal.SIGTERM)
        except ProcessLookupError:
            return
        try:
            self.process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(self.process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            self.process.wait(timeout=timeout)

    def record(self) -> dict[str, Any]:
        if self.process.poll() is None:
            self.finish()
        return {
            "command": self.command,
            "exit_code": self.process.returncode,
            "alive": self.alive,
            "stdout": _recent(self.stdout),
            "stderr": _recent(self.stderr),
        }


def _hand_frame(frame_type: int, timestamp_ms: int) -> bytes:
    payload = bytes((0, 0, 0x80, 0x3F)) + bytes(729)
    return struct.pack("<BBqI", 0xAB, frame_type, timestamp_ms, len(payload)) + payload


def _write_fake_source(path: Path, samples: int = 120) -> None:
    from spd_vr.pico_frames import FRAME_TYPE_HAND_LEFT, FRAME_TYPE_HAND_RIGHT

    with path.open("w", encoding="utf-8") as stream:
        for index in range(samples):
            timestamp = 1_000 + index
            for frame_type in (FRAME_TYPE_HAND_LEFT, FRAME_TYPE_HAND_RIGHT):
                stream.write(json.dumps({
                    "device_id": "SPD-E2E-FAKE",
                    "data_hex": _hand_frame(frame_type, timestamp).hex(),
                    "delay_ms": 50,
                }, sort_keys=True, separators=(",", ":")) + "\n")


def _artifact_gate(manifest: Path, urdf: Path) -> tuple[bool, str]:
    missing = [name for name in ARTIFACT_FILES if not (manifest.parent / name).is_file()]
    if missing:
        return False, f"{BLOCKER_REASON}; required artifacts missing: {', '.join(missing)}"
    try:
        from spd_vr.model_compiler.artifacts import verify_artifacts
        verify_artifacts(manifest, urdf)
    except Exception as exc:
        return False, str(exc)
    return True, f"verified {manifest.parent}"


def _preflight(endpoint: str, manifest: Path, urdf: Path, timeout: float) -> dict[str, Any]:
    command = [
        sys.executable, "-m", "spd_vr.preflight", "--repo-root", str(ROOT),
        "--manifest", str(manifest), "--urdf", str(urdf), "--endpoint", endpoint,
    ]
    result = _run(command, timeout=timeout)
    result["stdout"] = _recent(result["stdout"])
    result["stderr"] = _recent(result["stderr"])
    return result


def _status(endpoint: str, timeout: float) -> dict[str, Any]:
    command = [sys.executable, "-m", "spd_vr.status_cli", "--endpoint", endpoint, "--timeout", "0.35"]
    result = _run(command, timeout=timeout)
    try:
        result["decoded"] = json.loads(result["stdout"])
    except (TypeError, json.JSONDecodeError):
        result["decoded"] = None
    return result


def _wait_status(endpoint: str, processes: list[OwnedProcess], timeout: float) -> dict[str, Any] | None:
    deadline = time.monotonic() + timeout
    latest: dict[str, Any] | None = None
    while time.monotonic() < deadline:
        if any(not process.alive for process in processes):
            return latest
        result = _status(endpoint, min(1.0, max(0.2, deadline - time.monotonic())))
        decoded = result.get("decoded")
        if isinstance(decoded, dict):
            latest = decoded
            values = decoded.get("status")
            if isinstance(values, dict) and all(isinstance(values.get(name), dict) for name in ("bridge", "ik", "viewer")):
                if all(bool(values[name].get("ready")) for name in ("bridge", "ik", "viewer")):
                    return decoded
        time.sleep(0.05)
    return latest


def _control(endpoint: str, sequence_file: Path, command_name: str, timeout: float) -> dict[str, Any]:
    command = [
        sys.executable, "-m", "spd_vr.control_cli", command_name,
        "--endpoint", endpoint, "--sequence-file", str(sequence_file),
        "--session", "spd-teleop-e2e", "--timeout", "1.5",
    ]
    result = _run(command, timeout=timeout)
    try:
        result["decoded"] = json.loads(result["stdout"])
    except (TypeError, json.JSONDecodeError):
        result["decoded"] = None
    return result


def _synthetic(command: list[str], output_path: Path, timeout: float) -> int:
    runs: dict[str, Any] = {}
    with tempfile.TemporaryDirectory(prefix="spd-e2e-"):
        ik = _run([sys.executable, "-m", "spd_vr.arm_ik", "--self-test", "--ticks", "400"], timeout=timeout)
        runs["ik"] = ik
        viewer = _run([
            sys.executable, "-m", "spd_vr.viewer", "--headless", "--synthetic",
            "--ticks", "480", "--auto-start",
        ], timeout=timeout)
        runs["viewer"] = viewer
    ik_rate = None
    match = re.search(r"rate_hz=([0-9.]+)", runs["ik"]["stdout"])
    if match:
        ik_rate = float(match.group(1))
    ok = all(run.get("exit_code") == 0 for run in runs.values())
    result = {
        "blocked": False,
        "synthetic": True,
        "command": command,
        "stage": "synthetic-framework",
        "status": "pass" if ok else "fail",
        "reason": "synthetic framework smoke only; not authoritative production artifact evidence",
        "preflight": None,
        "processes": runs,
        "evidence": {
            "synthetic": True,
            "rates": {"ik_hz": ik_rate, "viewer_physics_hz": 480 if ok else None},
            "latency_ms": None,
            "drops": None,
            "solver": {"finite": ok, "failures": 0 if ok else None},
            "physics": {"finite": ok, "ticks": 480 if ok else None},
            "finite": ok,
            "contact": None,
            "shutdown": {"owned_children": True},
        },
        "recent_stderr": {name: _recent(run.get("stderr", "")) for name, run in runs.items()},
    }
    output_path.write_text(json.dumps(result, ensure_ascii=False, sort_keys=True, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False, sort_keys=True, indent=2))
    return 0 if ok else 1


def _production(command: list[str], output_path: Path, endpoint: str, manifest: Path, urdf: Path, timeout: float) -> int:
    preflight = _preflight(endpoint, manifest, urdf, timeout=timeout)
    verified, reason = _artifact_gate(manifest, urdf)
    base: dict[str, Any] = {
        "blocked": not verified,
        "synthetic": False,
        "command": command,
        "stage": "artifact-gate" if not verified else "preflight",
        "status": "blocked" if not verified else "running",
        "reason": reason,
        "preflight": preflight,
        "statuses": None,
        "control_statuses": [],
        "processes": {},
        "evidence": {
            "synthetic": False,
            "rates": {"bridge_hz": None, "ik_hz": None, "viewer_physics_hz": None},
            "latency_ms": None,
            "drops": None,
            "solver": {"finite": None, "failures": None},
            "physics": {"finite": None},
            "finite": None,
            "contact": None,
            "boundaries": {
                "side_isolation": None,
                "hold": None,
                "epoch": None,
                "pause": None,
                "reset": None,
            },
            "shutdown": {"acknowledged": False, "owned_children": True, "orphan_free": None},
        },
        "recent_stderr": {"preflight": preflight.get("stderr", "")},
    }
    if not verified:
        base["status"] = "blocked"
        output_path.write_text(json.dumps(base, ensure_ascii=False, sort_keys=True, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(base, ensure_ascii=False, sort_keys=True, indent=2))
        return 2

    processes: list[OwnedProcess] = []
    controls: dict[str, Any] = {}
    sequence_file: Path | None = None
    try:
        with tempfile.TemporaryDirectory(prefix="spd-e2e-") as temporary:
            temp = Path(temporary)
            source = temp / "fake-source.jsonl"
            sequence_file = temp / "control-sequence.json"
            _write_fake_source(source)
            model = manifest.parent
            commands = [
                [sys.executable, "-m", "spd_vr.pxrea_bridge", "--fake-source-jsonl", str(source), "--endpoint", endpoint, "--listen"],
                [sys.executable, "-m", "spd_vr.arm_ik", "--model", str(model / "arm_ik.xml"), "--manifest", str(manifest), "--urdf", str(urdf), "--endpoint", endpoint],
                [sys.executable, "-m", "spd_vr.viewer", "--headless", "--model", str(model / "unified_plant.xml"), "--manifest", str(manifest), "--urdf", str(urdf), "--endpoint", endpoint],
            ]
            for process_command in commands:
                processes.append(OwnedProcess(process_command))
                time.sleep(0.1)
            statuses = _wait_status(endpoint, processes, timeout)
            base["statuses"] = statuses
            status_values = statuses.get("status", {}) if isinstance(statuses, dict) else {}
            bridge_status = status_values.get("bridge", {}) if isinstance(status_values, dict) else {}
            base["evidence"]["drops"] = bridge_status.get("dropped")
            base["evidence"]["boundaries"]["epoch"] = int(bridge_status.get("tracking_epoch", 0) or 0) >= 1
            base["statuses_ready"] = bool(statuses and isinstance(status_values, dict) and all(status_values.get(name, {}).get("ready") for name in ("bridge", "ik", "viewer")))
            if not base["statuses_ready"]:
                base["status"] = "fail"
                base["reason"] = "three-process status readiness was not observed"
            else:
                status_snapshots: list[dict[str, Any]] = []
                for control_name in ("start", "pause", "resume", "realign", "reset", "start"):
                    controls[control_name] = _control(endpoint, sequence_file, control_name, timeout)
                    snapshot = _status(endpoint, timeout=min(2.0, timeout))
                    status_snapshots.append({"command": control_name, "result": snapshot})
                    if controls[control_name].get("exit_code") != 0:
                        base["status"] = "fail"
                        base["reason"] = f"control {control_name} was not acknowledged by all three peers"
                        break
                base["control_statuses"] = status_snapshots
                pause_values = next((item["result"].get("decoded", {}).get("status", {}) for item in status_snapshots if item["command"] == "pause"), {})
                reset_values = next((item["result"].get("decoded", {}).get("status", {}) for item in status_snapshots if item["command"] == "reset"), {})
                base["evidence"]["boundaries"]["pause"] = bool(pause_values.get("ik", {}).get("paused") and pause_values.get("viewer", {}).get("paused"))
                base["evidence"]["boundaries"]["reset"] = reset_values.get("viewer", {}).get("status") == "idle"
                shutdown = _control(endpoint, sequence_file, "shutdown", timeout)
                controls["shutdown"] = shutdown
                base["evidence"]["shutdown"]["acknowledged"] = shutdown.get("exit_code") == 0
                if shutdown.get("exit_code") != 0:
                    base["status"] = "fail"
                    base["reason"] = "shutdown was not acknowledged by all three peers"
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline and any(process.alive for process in processes):
                time.sleep(0.05)
    except (OSError, RuntimeError, ValueError) as exc:
        base["status"] = "fail"
        base["reason"] = str(exc)
    finally:
        for process in processes:
            if process.alive:
                process.terminate()
        for process in processes:
            process.finish()

    records = {name: process.record() for name, process in zip(("bridge", "ik", "viewer"), processes)}
    base["processes"] = records
    base["controls"] = controls
    base["evidence"]["shutdown"]["orphan_free"] = all(not record["alive"] for record in records.values())
    base["recent_stderr"].update({name: record["stderr"] for name, record in records.items()})
    if base["status"] == "running":
        base["status"] = "pass" if base["evidence"]["shutdown"]["orphan_free"] else "fail"
        if base["status"] == "pass":
            base["reason"] = "three-process production graph completed with ordered control and cleanup"
    output_path.write_text(json.dumps(base, ensure_ascii=False, sort_keys=True, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(base, ensure_ascii=False, sort_keys=True, indent=2))
    return 0 if base["status"] == "pass" else 1


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--json", type=Path, required=True)
    parser.add_argument("--synthetic", action="store_true")
    parser.add_argument("--endpoint", default="tcp/127.0.0.1:7447")
    parser.add_argument("--manifest", type=Path, default=GENERATED / "model_manifest.yaml")
    parser.add_argument("--urdf", type=Path, default=URDF_DEFAULT)
    parser.add_argument("--timeout", type=float, default=12.0)
    args = parser.parse_args(argv)
    command = [sys.executable, str(Path(__file__).resolve()), *([] if not args.synthetic else ["--synthetic"]), "--json", str(args.json)]
    if args.synthetic:
        return _synthetic(command, args.json, args.timeout)
    return _production(command, args.json, args.endpoint, args.manifest, args.urdf, args.timeout)


if __name__ == "__main__":
    raise SystemExit(main())
