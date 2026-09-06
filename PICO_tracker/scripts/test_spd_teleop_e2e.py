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

from spd_vr.defaults import DEFAULT_ZENOH_ENDPOINT

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


def _recent(text: str, limit: int = 20) -> str:
    lines = [line for line in text.splitlines() if line.strip()]
    return "\n".join(lines[-limit:])


def _run(command: Sequence[str], *, timeout: float) -> dict[str, Any]:
    try:
        owned = OwnedProcess(command)
        timed_out = False
        try:
            owned.process.wait(timeout=max(0.1, timeout))
        except subprocess.TimeoutExpired:
            timed_out = True
            owned.terminate()
        owned.finish()
        result = {
            "command": list(command),
            "exit_code": owned.process.returncode,
            "stdout": owned.stdout,
            "stderr": owned.stderr,
        }
        if timed_out:
            result["timeout"] = timeout
        return result
    except OSError as exc:
        return {"command": list(command), "exit_code": None, "stdout": "", "stderr": str(exc)}



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
        self.forced_termination = False

    @property
    def alive(self) -> bool:
        return self.process.poll() is None
    @property
    def group_alive(self) -> bool:
        try:
            os.killpg(self.process.pid, 0)
        except ProcessLookupError:
            return False
        except PermissionError:
            return True
        return True

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
        if not self.group_alive:
            return
        self.forced_termination = True
        try:
            os.killpg(self.process.pid, signal.SIGTERM)
        except ProcessLookupError:
            return
        deadline = time.monotonic() + max(0.1, timeout)
        while self.group_alive and time.monotonic() < deadline:
            try:
                self.process.wait(timeout=0.05)
            except subprocess.TimeoutExpired:
                pass
            time.sleep(0.01)
        if self.group_alive:
            try:
                os.killpg(self.process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            try:
                self.process.wait(timeout=max(0.1, timeout))
            except subprocess.TimeoutExpired:
                pass
FAKE_PHASES = (
    "baseline",
    "left_wrist_translation",
    "right_wrist_rotation",
    "left_finger_flex",
    "right_finger_flex",
)


def _hand_frame(frame_type: int, timestamp_ms: int, side: str, phase: str) -> bytes:
    from spd_vr.pico_frames import PicoFrame, decode_hand

    if side not in {"left", "right"} or phase not in FAKE_PHASES:
        raise ValueError(f"invalid hand side/phase: {side}/{phase}")
    sign = -1.0 if side == "left" else 1.0
    payload = bytearray(733)
    payload[0] = 1
    struct.pack_into("<f", payload, 1, 1.0)
    for joint in range(26):
        x = sign * (0.20 + 0.002 * joint)
        y = 0.01 * joint
        z = 0.001 * joint
        if phase == "left_wrist_translation" and side == "left" and joint == 1:
            x -= 0.08
        quaternion = (0.0, 0.0, 0.0, 1.0)
        if joint == 1:
            quaternion = (0.0, 0.0, sign * 0.173648, 0.984808)
            if phase == "right_wrist_rotation" and side == "right":
                quaternion = (0.0, 0.258819, 0.0, 0.965926)
        if joint >= 2:
            x += sign * 0.003 * (joint - 1)
            if phase == "left_finger_flex" and side == "left":
                y += 0.06
            if phase == "right_finger_flex" and side == "right":
                y += 0.06
        struct.pack_into("<7f", payload, 5 + 28 * joint, x, y, z, *quaternion)
    frame = PicoFrame(frame_type, timestamp_ms, bytes(payload))
    hand = decode_hand(frame)
    wrist_sign = -1.0 if side == "left" else 1.0
    if float(hand.joints[1, 0]) * wrist_sign <= 0.0 or abs(float(hand.joints[1, 5])) <= 0.0:
        raise AssertionError("fake source must contain side-specific wrist translation/rotation")
    if not hand.active or not any(abs(float(value)) > 0.0 for value in hand.joints[2:, :3].flat):
        raise AssertionError("fake source must contain active, non-zero finger flex")
    return struct.pack("<BBqI", 0xAB, frame_type, timestamp_ms, len(payload)) + payload
def _write_fake_source(path: Path, samples: int = 120) -> dict[str, int]:
    from spd_vr.pico_frames import FRAME_TYPE_HAND_LEFT, FRAME_TYPE_HAND_RIGHT

    phase_count = max(3, samples // len(FAKE_PHASES))
    counts = {phase: 0 for phase in FAKE_PHASES}
    timestamp = 1_000
    with path.open("w", encoding="utf-8") as stream:
        for phase in FAKE_PHASES:
            for _ in range(phase_count):
                for frame_type in (FRAME_TYPE_HAND_LEFT, FRAME_TYPE_HAND_RIGHT):
                    side = "left" if frame_type == FRAME_TYPE_HAND_LEFT else "right"
                    stream.write(json.dumps({
                        "device_id": "SPD-E2E-FAKE",
                        "data_hex": _hand_frame(frame_type, timestamp, side, phase).hex(),
                        "delay_ms": 10,
                    }, sort_keys=True, separators=(",", ":")) + "\n")
                    counts[phase] += 1
                timestamp += 1
    return counts


def _artifact_gate(manifest: Path, urdf: Path) -> tuple[bool, str]:
    missing = [name for name in ARTIFACT_FILES if not (manifest.parent / name).is_file()]
    if missing:
        return False, "required artifacts missing: " + ", ".join(missing)
    try:
        from spd_vr.model_compiler.artifacts import verify_artifacts
        verify_artifacts(manifest, urdf)
    except Exception as exc:
        return False, str(exc)
    return True, f"verified {manifest.parent}"


def _preflight(endpoint: str, manifest: Path, urdf: Path, fake_source: Path, timeout: float) -> dict[str, Any]:
    command = [
        sys.executable, "-m", "spd_vr.preflight", "--repo-root", str(ROOT),
        "--fake-source", str(fake_source), "--manifest", str(manifest),
        "--urdf", str(urdf), "--endpoint", endpoint,
    ]
    result = _run(command, timeout=timeout)
    result["stdout"] = _recent(result["stdout"])
    result["stderr"] = _recent(result["stderr"])
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
    source_temp = tempfile.TemporaryDirectory(prefix="spd-e2e-source-")
    source = Path(source_temp.name) / "fake-source.jsonl"
    try:
        source_counts = _write_fake_source(source)
    except Exception:
        source_temp.cleanup()
        raise
    preflight = _preflight(endpoint, manifest, urdf, source, timeout=timeout)
    verified, reason = _artifact_gate(manifest, urdf)
    preflight_ok = preflight.get("exit_code") == 0
    blocked = not preflight_ok or not verified
    if not verified:
        gate_reason = reason
    elif not preflight_ok:
        gate_reason = f"preflight failed with exit code {preflight.get('exit_code')}"
    else:
        gate_reason = reason
    base: dict[str, Any] = {
        "blocked": blocked,
        "synthetic": False,
        "command": command,
        "stage": "artifact-gate" if not verified else ("preflight" if not preflight_ok else "processes"),
        "status": "blocked" if blocked else "running",
        "reason": gate_reason,
        "preflight": preflight,
        "fake_source_phases": source_counts,
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
            "hand_finger_wrist": None,
            "control_ack": False,
            "boundaries": {
                "side_isolation": None,
                "hold": None,
                "epoch": None,
                "pause": None,
                "reset": None,
            },
            "shutdown": {"acknowledged": False, "natural_exit": False, "owned_children": True, "orphan_free": None},
        },
        "recent_stderr": {"preflight": preflight.get("stderr", "")},
    }
    if blocked:
        source_temp.cleanup()
        output_path.write_text(json.dumps(base, ensure_ascii=False, sort_keys=True, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(base, ensure_ascii=False, sort_keys=True, indent=2))
        return 2

    processes: list[OwnedProcess] = []
    controls: dict[str, Any] = {}
    try:
        with tempfile.TemporaryDirectory(prefix="spd-e2e-") as temporary:
            temp = Path(temporary)
            sequence_file = temp / "control-sequence.json"
            model = manifest.parent
            commands = [
                [sys.executable, "-m", "spd_vr.pxrea_bridge", "--fake-source-jsonl", str(source), "--endpoint", endpoint, "--listen", "--wait-for-shutdown"],
                [sys.executable, "-m", "spd_vr.arm_ik", "--model", str(model / "arm_ik.xml"), "--manifest", str(manifest), "--urdf", str(urdf), "--endpoint", endpoint],
                [sys.executable, "-m", "spd_vr.viewer", "--headless", "--model", str(model / "unified_plant.xml"), "--manifest", str(manifest), "--urdf", str(urdf), "--endpoint", endpoint, "--until-shutdown"],
            ]
            for process_command in commands:
                processes.append(OwnedProcess(process_command))
                time.sleep(0.1)
            statuses = _wait_status(endpoint, processes, timeout)
            base["statuses"] = statuses
            status_values = statuses.get("status", {}) if isinstance(statuses, dict) else {}
            bridge_status = status_values.get("bridge", {}) if isinstance(status_values, dict) else {}
            base["evidence"]["drops"] = bridge_status.get("dropped")
            base["statuses_ready"] = bool(statuses and isinstance(status_values, dict) and all(status_values.get(name, {}).get("ready") for name in ("bridge", "ik", "viewer")))
            if not base["statuses_ready"]:
                base["status"] = "fail"
                base["reason"] = "three-process status readiness was not observed"
            else:
                status_snapshots: list[dict[str, Any]] = []
                for index, control_name in enumerate(("start", "pause", "resume", "realign", "reset", "start"), 1):
                    control_key = f"{index}:{control_name}"
                    controls[control_key] = _control(endpoint, sequence_file, control_name, timeout)
                    snapshot = _status(endpoint, timeout=min(2.0, timeout))
                    status_snapshots.append({"command": control_name, "result": snapshot})
                    if controls[control_key].get("exit_code") != 0:
                        base["status"] = "fail"
                        base["reason"] = f"control {control_name} was not acknowledged by all three peers"
                        break
                base["control_statuses"] = status_snapshots
                decoded_statuses = [
                    item["result"].get("decoded", {}).get("status", {})
                    for item in status_snapshots
                    if isinstance(item["result"].get("decoded"), dict)
                ]
                latest = decoded_statuses[-1] if decoded_statuses else {}
                ik_values = latest.get("ik", {}) if isinstance(latest, dict) else {}
                viewer_values = latest.get("viewer", {}) if isinstance(latest, dict) else {}
                if isinstance(ik_values, dict) and isinstance(viewer_values, dict):
                    target_seen = ik_values.get("target_sequence") is not None and viewer_values.get("target_sequence") is not None
                    base["evidence"]["hand_finger_wrist"] = bool(target_seen and all(value > 0 for value in source_counts.values()))
                    base["evidence"]["boundaries"]["side_isolation"] = viewer_values.get("arm_valid_mask") == 3 and viewer_values.get("hand_valid_mask") == 3
                    base["evidence"]["solver"]["finite"] = ik_values.get("finite") is True
                    base["evidence"]["physics"]["finite"] = viewer_values.get("finite") is True
                    base["evidence"]["finite"] = base["evidence"]["solver"]["finite"] and base["evidence"]["physics"]["finite"]
                    contact = viewer_values.get("contact")
                    base["evidence"]["contact"] = isinstance(contact, int) and contact >= 0
                    base["evidence"]["contact_count"] = contact
                pause_values = next((item["result"].get("decoded", {}).get("status", {}) for item in status_snapshots if item["command"] == "pause"), {})
                reset_values = next((item["result"].get("decoded", {}).get("status", {}) for item in status_snapshots if item["command"] == "reset"), {})
                time.sleep(0.15)
                stale_snapshot = _status(endpoint, timeout=min(2.0, timeout))
                base["control_statuses"].append({"command": "stale-hold", "result": stale_snapshot})
                stale_values = stale_snapshot.get("decoded", {}).get("status", {}) if isinstance(stale_snapshot.get("decoded"), dict) else {}
                stale_ik = stale_values.get("ik", {}) if isinstance(stale_values, dict) else {}
                base["evidence"]["boundaries"]["hold"] = stale_ik.get("left_hold_reason") in {"input_stale", "inactive"} or stale_ik.get("right_hold_reason") in {"input_stale", "inactive"}
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
            if process.alive or process.group_alive:
                process.terminate()
        for process in processes:
            process.finish()
        source_temp.cleanup()

    records = {name: process.record() for name, process in zip(("bridge", "ik", "viewer"), processes)}
    base["processes"] = records
    base["controls"] = controls
    orphan_free = all(not record["group_alive"] for record in records.values())
    natural_exit = all(record["natural_exit"] for record in records.values())
    control_ack = bool(controls) and all(value.get("exit_code") == 0 for value in controls.values())
    base["evidence"]["shutdown"]["orphan_free"] = orphan_free
    base["evidence"]["shutdown"]["natural_exit"] = natural_exit
    base["evidence"]["control_ack"] = control_ack
    base["recent_stderr"].update({name: record["stderr"] for name, record in records.items()})
    invariants = {
        "statuses_ready": bool(base.get("statuses_ready")),
        "control_ack": control_ack,
        "side_isolation": base["evidence"]["boundaries"]["side_isolation"] is True,
        "hand_finger_wrist": base["evidence"]["hand_finger_wrist"] is True,
        "hold": base["evidence"]["boundaries"]["hold"] is True,
        "epoch": base["evidence"]["boundaries"]["epoch"] is True,
        "pause": base["evidence"]["boundaries"]["pause"] is True,
        "reset": base["evidence"]["boundaries"]["reset"] is True,
        "finite": base["evidence"]["finite"] is True,
        "contact": base["evidence"]["contact"] is True,
        "natural_exit": natural_exit,
        "orphan_free": orphan_free,
    }
    base["invariants"] = invariants
    if base["status"] == "running":
        base["status"] = "pass" if all(invariants.values()) else "fail"
        if base["status"] == "pass":
            base["reason"] = "three-process production graph completed with all required invariants"
        else:
            base["reason"] = "production invariants not proven: " + ", ".join(name for name, ok in invariants.items() if not ok)
    output_path.write_text(json.dumps(base, ensure_ascii=False, sort_keys=True, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(base, ensure_ascii=False, sort_keys=True, indent=2))
    return 0 if base["status"] == "pass" else 1


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--json", type=Path, required=True)
    parser.add_argument("--synthetic", action="store_true")
    parser.add_argument("--endpoint", default=DEFAULT_ZENOH_ENDPOINT)
    parser.add_argument("--manifest", type=Path, default=GENERATED / "model_manifest.yaml")
    parser.add_argument("--urdf", type=Path, default=URDF_DEFAULT)
    parser.add_argument("--timeout", type=float, default=12.0)
    args = parser.parse_args(argv)
    args.json.parent.mkdir(parents=True, exist_ok=True)
    command = [sys.executable, str(Path(__file__).resolve()), *([] if not args.synthetic else ["--synthetic"]), "--json", str(args.json)]
    if args.synthetic:
        return _synthetic(command, args.json, args.timeout)
    return _production(command, args.json, args.endpoint, args.manifest, args.urdf, args.timeout)


if __name__ == "__main__":
    raise SystemExit(main())
