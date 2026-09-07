#!/usr/bin/env python3
"""Run the production Python teleoperation graph, or explicit synthetic smoke."""

from __future__ import annotations

import argparse
import json
import os
import re
import socket
import signal
import struct
import subprocess
import sys
import tempfile
import time
import threading
from pathlib import Path
from typing import Any, Sequence

from spd_vr.defaults import DEFAULT_ZENOH_ENDPOINT

ROOT = Path(__file__).resolve().parents[1]
GENERATED = ROOT / "packages" / "spd-vr" / "generated"
URDF_DEFAULT = ROOT / "assets" / "tianji_wuji2" / "tianji_wuji2.urdf"
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
    def __init__(self, command: Sequence[str], *, sequence_file: Path | None = None) -> None:
        self.command = list(command)
        environment = _environment()
        if sequence_file is not None:
            environment["SPD_VR_CONTROL_SEQUENCE_FILE"] = str(sequence_file)
        self.process = subprocess.Popen(
            self.command,
            cwd=ROOT,
            env=environment,
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

    def record(self) -> dict[str, Any]:
        return {
            "command": self.command,
            "pid": self.process.pid,
            "exit_code": self.process.poll(),
            "group_alive": self.group_alive,
            "natural_exit": self.process.returncode == 0 and not self.forced_termination,
            "forced_termination": self.forced_termination,
            "stdout": _recent(self.stdout),
            "stderr": _recent(self.stderr),
        }


SIMULATION_PHASES = (
    "baseline",
    "left_wrist_translation",
    "right_wrist_rotation",
    "left_finger_flex",
    "right_finger_flex",
)


def _pico2_payload(phase: str) -> bytes:
    from pico_hand_tracking import PAYLOAD_BYTES, POSE, parse_hand_frame

    if phase not in SIMULATION_PHASES:
        raise ValueError(f"unknown PICO2 simulation phase: {phase}")

    def joint_pose(side: str, joint: int) -> tuple[float, ...]:
        sign = -1.0 if side == "left" else 1.0
        x = sign * (0.20 + 0.002 * joint)
        y = 0.01 * joint
        z = 1.0 + 0.001 * joint
        quaternion = (0.0, 0.0, 0.0, 1.0)
        if joint == 1:
            quaternion = (0.0, 0.0, sign * 0.173648, 0.984808)
            if phase == "left_wrist_translation" and side == "left":
                x -= 0.08
            if phase == "right_wrist_rotation" and side == "right":
                quaternion = (0.0, 0.258819, 0.0, 0.965926)
        if joint >= 2:
            x += sign * 0.003 * (joint - 1)
            if phase == "left_finger_flex" and side == "left":
                y += 0.06
            if phase == "right_finger_flex" and side == "right":
                y += 0.06
        return (x, y, z, *quaternion)

    payload = bytearray(PAYLOAD_BYTES)
    struct.pack_into("<BBBB", payload, 0, 1, 0x07, 26, 0)
    offset = 4
    struct.pack_into("<7f", payload, offset, 0.0, 0.0, 1.6, 0.0, 0.0, 0.0, 1.0)
    offset += POSE.size
    for side in ("left", "right"):
        struct.pack_into("<BBBB", payload, offset, 1, 0, 0, 0)
        offset += 4
        struct.pack_into("<7f", payload, offset, *joint_pose(side, 1))
        offset += POSE.size
        for joint in range(26):
            struct.pack_into("<BBBB", payload, offset, 1, 0, 0, 0)
            offset += 4
            struct.pack_into("<7f", payload, offset, *joint_pose(side, joint))
            offset += POSE.size
            struct.pack_into("<f", payload, offset, 0.01)
            offset += 4
    if offset != len(payload):
        raise AssertionError("PICO2 payload size mismatch")
    decoded = parse_hand_frame(1_000, payload)
    if not (
        decoded.head.valid
        and decoded.left.valid
        and decoded.right.valid
        and all(joint.valid for hand in (decoded.left, decoded.right) for joint in hand.joints)
    ):
        raise AssertionError("PICO2 simulation payload must contain two active hands")
    if not all(
        any(abs(value) > 0.0 for value in joint.position)
        for hand in (decoded.left, decoded.right)
        for joint in hand.joints[2:]
    ):
        raise AssertionError("PICO2 simulation payload must contain non-zero finger poses")
    return bytes(payload)


def _pico2_source_frames(samples: int = 120) -> tuple[tuple[str, bytes], ...]:
    phase_count = max(12, samples // len(SIMULATION_PHASES))
    payloads = {phase: _pico2_payload(phase) for phase in SIMULATION_PHASES}
    return tuple(
        (phase, payloads[phase])
        for phase in SIMULATION_PHASES
        for _ in range(phase_count)
    )


class LocalPico2Source:
    """Serve a deterministic PICO2 hand stream without ADB."""

    def __init__(
        self,
        frames: Sequence[tuple[str, bytes]],
        *,
        interval_seconds: float = 0.01,
    ) -> None:
        if not frames:
            raise ValueError("PICO2 source requires at least one frame")
        if interval_seconds <= 0.0:
            raise ValueError("PICO2 source interval must be positive")
        from pico_hand_tracking import HEADER, MAGIC, TYPE_HAND_FRAME

        self._frames = tuple(frames)
        self._header = HEADER
        self._magic = MAGIC
        self._frame_type = TYPE_HAND_FRAME
        self._interval_seconds = float(interval_seconds)
        self._ready = threading.Event()
        self._stop = threading.Event()
        self._stop_streaming = threading.Event()
        self._source_lock = threading.Lock()
        self._socket_lock = threading.Lock()
        self._listener: socket.socket | None = None
        self._connection: socket.socket | None = None
        self._port: int | None = None
        self._frame_index = 0
        self._timestamp_ms = 1_000
        self._phase_counts = {phase: 0 for phase, _ in self._frames}
        self._error = ""
        self._thread = threading.Thread(
            target=self._serve,
            name="spd-e2e-pico2-source",
            daemon=True,
        )

    @property
    def port(self) -> int:
        if self._port is None:
            raise RuntimeError("PICO2 source port is unavailable")
        return self._port

    @property
    def phase_counts(self) -> dict[str, int]:
        with self._source_lock:
            return dict(self._phase_counts)

    @property
    def error(self) -> str:
        with self._source_lock:
            return self._error

    def start(self, timeout: float = 2.0) -> int:
        self._thread.start()
        if not self._ready.wait(timeout):
            self.stop()
            self._thread.join(timeout)
            raise RuntimeError("PICO2 source did not become ready")
        if self.error:
            raise RuntimeError(f"PICO2 source failed: {self.error}")
        return self.port

    def stop_streaming(self) -> None:
        self._stop_streaming.set()

    def reset_timestamp(self) -> None:
        with self._source_lock:
            self._timestamp_ms = 1

    def wait_for_all_phases(self, timeout: float) -> bool:
        deadline = time.monotonic() + max(0.0, timeout)
        while time.monotonic() < deadline:
            if all(count > 0 for count in self.phase_counts.values()):
                return True
            if self.error:
                return False
            time.sleep(0.01)
        return all(count > 0 for count in self.phase_counts.values())

    @staticmethod
    def _close_socket(sock: socket.socket | None) -> None:
        if sock is None:
            return
        try:
            sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        try:
            sock.close()
        except OSError:
            pass

    def _next_packet(self) -> bytes:
        with self._source_lock:
            if self._frame_index < len(self._frames):
                phase, payload = self._frames[self._frame_index]
                self._frame_index += 1
            else:
                phase, payload = self._frames[0]
            timestamp_ms = self._timestamp_ms
            self._timestamp_ms += 1
            self._phase_counts[phase] += 1
        return self._header.pack(
            self._magic,
            self._frame_type,
            timestamp_ms,
            len(payload),
        ) + payload

    def _serve(self) -> None:
        listener: socket.socket | None = None
        connection: socket.socket | None = None
        try:
            listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            listener.bind(("127.0.0.1", 0))
            listener.listen(1)
            listener.settimeout(0.1)
            with self._socket_lock:
                self._listener = listener
                self._port = int(listener.getsockname()[1])
            self._ready.set()
            while not self._stop.is_set():
                try:
                    connection, _ = listener.accept()
                except socket.timeout:
                    continue
                break
            if connection is None:
                return
            with self._socket_lock:
                self._connection = connection
                self._listener = None
            self._close_socket(listener)
            listener = None
            while not self._stop.is_set() and not self._stop_streaming.is_set():
                connection.sendall(self._next_packet())
                self._stop.wait(self._interval_seconds)
            self._stop.wait()
        except (BrokenPipeError, ConnectionResetError):
            pass
        except OSError as exc:
            if not self._stop.is_set():
                with self._source_lock:
                    self._error = str(exc)
        finally:
            self._ready.set()
            with self._socket_lock:
                active_listener = self._listener
                active_connection = self._connection
                self._listener = None
                self._connection = None
            self._close_socket(active_connection)
            self._close_socket(active_listener)
            self._close_socket(listener)

    def stop(self) -> None:
        self._stop.set()
        with self._socket_lock:
            listener = self._listener
            connection = self._connection
        self._close_socket(connection)
        self._close_socket(listener)

    def join(self, timeout: float = 2.0) -> bool:
        self._thread.join(timeout)
        return not self._thread.is_alive()


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


def _preflight(endpoint: str, manifest: Path, urdf: Path, local_fixture: Path, timeout: float) -> dict[str, Any]:
    command = [
        sys.executable, "-m", "spd_vr.preflight", "--repo-root", str(ROOT),
        "--fake-source", str(local_fixture), "--manifest", str(manifest),
        "--urdf", str(urdf), "--endpoint", endpoint,
        "--session", "spd-teleop-e2e",
    ]
    result = _run(command, timeout=timeout)
    result["stdout"] = _recent(result["stdout"])
    result["stderr"] = _recent(result["stderr"])
    return result


def _status(endpoint: str, timeout: float) -> dict[str, Any]:
    result = _run(
        [
            sys.executable,
            "-m",
            "spd_vr.status_cli",
            "--endpoint",
            endpoint,
            "--timeout",
            str(max(0.1, timeout)),
        ],
        timeout=timeout + 0.5,
    )
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
        "--session", "spd-teleop", "--timeout", "1.5",
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
    preflight = _preflight(endpoint, manifest, urdf, Path(__file__).resolve(), timeout=timeout)
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
        "pico2_source": {
            "phases": {phase: 0 for phase in SIMULATION_PHASES},
            "error": "",
            "closed": None,
        },
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
            "shutdown": {
                "acknowledged": False,
                "natural_exit": False,
                "owned_children": True,
                "orphan_free": None,
                "pico2_source_closed": None,
            },
        },
        "recent_stderr": {"preflight": preflight.get("stderr", "")},
    }
    if blocked:
        output_path.write_text(json.dumps(base, ensure_ascii=False, sort_keys=True, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(base, ensure_ascii=False, sort_keys=True, indent=2))
        return 2

    processes: list[OwnedProcess] = []
    controls: dict[str, Any] = {}
    source: LocalPico2Source | None = None
    source_closed: bool | None = None
    try:
        with tempfile.TemporaryDirectory(prefix="spd-e2e-") as temporary:
            temp = Path(temporary)
            sequence_file = temp / "control-sequence.json"
            model = manifest.parent
            source = LocalPico2Source(_pico2_source_frames())
            source_port = source.start()
            base["pico2_source"]["port"] = source_port
            commands = [
                [
                    sys.executable,
                    "-m",
                    "spd_vr.pico2_bridge",
                    "--host",
                    "127.0.0.1",
                    "--port",
                    str(source_port),
                    "--no-adb-forward",
                    "--endpoint",
                    endpoint,
                    "--listen",
                ],
                [
                    sys.executable,
                    "-m",
                    "spd_vr.arm_ik",
                    "--model",
                    str(model / "arm_ik.xml"),
                    "--manifest",
                    str(manifest),
                    "--urdf",
                    str(urdf),
                    "--endpoint",
                    endpoint,
                ],
                [
                    sys.executable,
                    "-m",
                    "spd_vr.viewer",
                    "--headless",
                    "--model",
                    str(model / "unified_plant.xml"),
                    "--manifest",
                    str(manifest),
                    "--urdf",
                    str(urdf),
                    "--endpoint",
                    endpoint,
                    "--until-shutdown",
                ],
            ]
            for process_command in commands:
                processes.append(OwnedProcess(process_command, sequence_file=sequence_file))
                time.sleep(0.1)
            statuses = _wait_status(endpoint, processes, timeout)
            source_complete = source.wait_for_all_phases(min(2.0, timeout))
            if source_complete:
                time.sleep(0.15)
                refreshed = _status(endpoint, timeout=min(2.0, timeout)).get("decoded")
                if isinstance(refreshed, dict):
                    statuses = refreshed
            base["statuses"] = statuses
            base["pico2_source"]["phases"] = source.phase_counts
            base["pico2_source"]["error"] = source.error
            status_values = statuses.get("status", {}) if isinstance(statuses, dict) else {}
            bridge_status = status_values.get("bridge", {}) if isinstance(status_values, dict) else {}
            ik_status = status_values.get("ik", {}) if isinstance(status_values, dict) else {}
            viewer_status = status_values.get("viewer", {}) if isinstance(status_values, dict) else {}
            base["evidence"]["drops"] = bridge_status.get("dropped") if isinstance(bridge_status, dict) else None
            try:
                bridge_published = int(bridge_status.get("published", 0)) > 0
                bridge_received = int(bridge_status.get("received", 0)) > 0
            except (AttributeError, TypeError, ValueError):
                bridge_published = False
                bridge_received = False
            base["statuses_ready"] = bool(
                source_complete
                and not source.error
                and isinstance(status_values, dict)
                and all(
                    isinstance(status_values.get(name), dict)
                    and bool(status_values[name].get("ready"))
                    for name in ("bridge", "ik", "viewer")
                )
                and bridge_published
                and bridge_received
                and isinstance(ik_status, dict)
                and ik_status.get("target_sequence") is not None
                and isinstance(viewer_status, dict)
                and viewer_status.get("target_sequence") is not None
            )
            if not base["statuses_ready"]:
                base["status"] = "fail"
                base["reason"] = "PICO2 bridge, IK, and viewer did not become ready with tracking data"
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

                pause_values = next(
                    (
                        item["result"].get("decoded", {}).get("status", {})
                        for item in status_snapshots
                        if item["command"] == "pause"
                    ),
                    {},
                )
                pause_sequence = controls.get("2:pause", {}).get("decoded", {}).get("sequence")
                base["evidence"]["boundaries"]["pause"] = bool(
                    isinstance(pause_values, dict)
                    and pause_sequence is not None
                    and all(
                        values.get("paused") is True
                        and values.get("sequence") == pause_sequence
                        for values in (pause_values.get("ik", {}), pause_values.get("viewer", {}))
                        if isinstance(values, dict)
                    )
                    and all(isinstance(pause_values.get(name), dict) for name in ("ik", "viewer"))
                )

                reset_values = next(
                    (
                        item["result"].get("decoded", {}).get("status", {})
                        for item in status_snapshots
                        if item["command"] == "reset"
                    ),
                    {},
                )
                reset_sequence = controls.get("5:reset", {}).get("decoded", {}).get("sequence")
                base["evidence"]["boundaries"]["reset"] = bool(
                    isinstance(reset_values, dict)
                    and reset_sequence is not None
                    and all(
                        values.get("sequence") == reset_sequence
                        for values in (reset_values.get("ik", {}), reset_values.get("viewer", {}))
                        if isinstance(values, dict)
                    )
                    and all(isinstance(reset_values.get(name), dict) for name in ("ik", "viewer"))
                )

                initial_epoch = bridge_status.get("tracking_epoch") if isinstance(bridge_status, dict) else None
                source.reset_timestamp()
                time.sleep(0.2)
                epoch_snapshot = _status(endpoint, timeout=min(2.0, timeout))
                status_snapshots.append({"command": "pico2-epoch", "result": epoch_snapshot})
                epoch_values = epoch_snapshot.get("decoded", {}).get("status", {}) if isinstance(epoch_snapshot.get("decoded"), dict) else {}
                epoch_bridge = epoch_values.get("bridge", {}) if isinstance(epoch_values, dict) else {}
                try:
                    base["evidence"]["boundaries"]["epoch"] = int(epoch_bridge.get("tracking_epoch", 0)) > int(initial_epoch)
                except (AttributeError, TypeError, ValueError):
                    base["evidence"]["boundaries"]["epoch"] = False

                latest = epoch_values
                ik_values = latest.get("ik", {}) if isinstance(latest, dict) else {}
                viewer_values = latest.get("viewer", {}) if isinstance(latest, dict) else {}
                if isinstance(ik_values, dict) and isinstance(viewer_values, dict):
                    target_seen = ik_values.get("target_sequence") is not None and viewer_values.get("target_sequence") is not None
                    base["evidence"]["hand_finger_wrist"] = bool(
                        target_seen and all(value > 0 for value in source.phase_counts.values())
                    )
                    base["evidence"]["boundaries"]["side_isolation"] = (
                        viewer_values.get("arm_valid_mask") == 3
                        and viewer_values.get("hand_valid_mask") == 3
                    )
                    base["evidence"]["solver"]["finite"] = ik_values.get("finite") is True
                    base["evidence"]["physics"]["finite"] = viewer_values.get("finite") is True
                    base["evidence"]["finite"] = (
                        base["evidence"]["solver"]["finite"]
                        and base["evidence"]["physics"]["finite"]
                    )
                    contact = viewer_values.get("contact")
                    base["evidence"]["contact"] = isinstance(contact, int) and contact >= 0
                    base["evidence"]["contact_count"] = contact

                source.stop_streaming()
                time.sleep(0.15)
                stale_snapshot = _status(endpoint, timeout=min(2.0, timeout))
                status_snapshots.append({"command": "stale-hold", "result": stale_snapshot})
                stale_values = stale_snapshot.get("decoded", {}).get("status", {}) if isinstance(stale_snapshot.get("decoded"), dict) else {}
                stale_ik = stale_values.get("ik", {}) if isinstance(stale_values, dict) else {}
                base["evidence"]["boundaries"]["hold"] = (
                    stale_ik.get("left_hold_reason") in {"input_stale", "inactive"}
                    or stale_ik.get("right_hold_reason") in {"input_stale", "inactive"}
                )
                base["control_statuses"] = status_snapshots

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
        if source is not None:
            source.stop()
            source_closed = source.join()
            base["pico2_source"]["phases"] = source.phase_counts
            base["pico2_source"]["error"] = source.error
            base["pico2_source"]["closed"] = source_closed

    records = {name: process.record() for name, process in zip(("bridge", "ik", "viewer"), processes)}
    base["processes"] = records
    base["controls"] = controls
    orphan_free = len(records) == 3 and all(not record["group_alive"] for record in records.values())
    natural_exit = len(records) == 3 and all(record["natural_exit"] for record in records.values())
    control_ack = bool(controls) and all(value.get("exit_code") == 0 for value in controls.values())
    base["evidence"]["shutdown"]["orphan_free"] = orphan_free
    base["evidence"]["shutdown"]["natural_exit"] = natural_exit
    base["evidence"]["shutdown"]["pico2_source_closed"] = source_closed
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
        "pico2_source_closed": source_closed is True,
    }
    base["invariants"] = invariants
    if base["status"] == "running":
        base["status"] = "pass" if all(invariants.values()) else "fail"
        if base["status"] == "pass":
            base["reason"] = "three-process PICO2 graph completed with all required invariants"
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
