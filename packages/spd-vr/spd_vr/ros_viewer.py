"""External ROS joint targets -> MuJoCo physics, live comparison, and recording."""
from __future__ import annotations

# Load ROS's native extension before simulation/compiler native dependencies.
# Importing rclpy after those libraries can resolve incompatible C++ symbols.
try:
    import rclpy
except ImportError:  # The non-ROS environment can still import simulation modules.
    rclpy = None

import argparse
from concurrent.futures import Future, ThreadPoolExecutor
from pathlib import Path
import queue
import time
from typing import Any

import numpy as np

from .camera import CAMERA_NAMES, MujocoCameraProvider
from .local_control import LocalControlServer
from .recorder import EpisodeRecorder
from .ros_executor import ControlTerminal, RosJointCommandExecutor
from .ros_joint_command import JOINT_NAMES, TOPIC
from .viewer import PlantController
from .scene import build_selected_scene, frame_scene
from .viewer_window import ViewerWindow


class RosViewerApp:
    def __init__(self, args: argparse.Namespace) -> None:
        import rclpy

        self.args = args
        self.stop = False
        self.episode_counter = 0
        self.selected_joint = 0
        self.notice = "Waiting for an external publisher; e enables a fresh aligned candidate"
        self._actions: queue.SimpleQueue[tuple[str, str]] = queue.SimpleQueue()
        self._record_worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix="spd-record-control")
        self._record_job: Future | None = None
        self._record_operation = ""
        self._accept_recording = False
        self.recorder = EpisodeRecorder(args.output, camera_names=CAMERA_NAMES)
        scene_result = build_selected_scene(args.scene, args.task, args.seed)
        args.scene = scene_result.scene if scene_result is not None else "hardware_free"
        args.task = scene_result.task if scene_result is not None else "external_joint_command"
        self.plant = PlantController(
            strict_artifacts=True, command_only=True,
            camera_config_path=Path(__file__).resolve().parents[1] / "config" / "sim_cameras.yaml",
            scene_result=scene_result, scene_output_dir=args.output / "scenes",
        )
        self.camera: MujocoCameraProvider | None = None
        self.window = ViewerWindow(
            self.plant.model,
            self.plant.data,
            headless=args.headless,
            shutdown=self.request_stop,
            recording_control=self.recording_control,
            joint_control=lambda key: self._actions.put(("control", key)),
        )
        rclpy.init()
        self.node = rclpy.create_node("spd_mujoco_joint_command_executor")
        self.executor = RosJointCommandExecutor(
            self.node, self.plant, max_enable_delta_rad=args.max_enable_delta_rad,
        )
        self.control_terminal = ControlTerminal(self.executor)
        self.local_control: LocalControlServer | None = None
        self._started_ns = time.monotonic_ns()
        self._last_received = 0
        self._rate_ns = self._started_ns
        self._receive_hz = 0.0
        self._last_applied = None

    def request_stop(self) -> None:
        self.stop = True

    def recording_control(self, command: str) -> None:
        # MuJoCo's key callback runs on its UI thread; keep all model access here.
        self._actions.put(("record", command))

    def _record_background(self, operation: str, function: Any, *args: Any, **kwargs: Any) -> None:
        self._accept_recording = False
        self._record_operation = operation
        self._record_job = self._record_worker.submit(function, *args, **kwargs)

    def _poll_recording(self) -> None:
        if self._record_job is not None and self._record_job.done():
            try:
                result = self._record_job.result()
                self._accept_recording = self._record_operation == "preparing"
                self.notice = f"Recording {self._record_operation} complete" + (f": {result}" if result else "")
            except Exception as exc:
                self.notice = f"Recording error: {exc}"
                self._accept_recording = False
            self._record_job = None
        if self._record_job is None and self.recorder.error is not None:
            self.notice = f"Recording error: {self.recorder.error}; preserving partial file"
            self._record_background("interrupted", self.recorder.abort_episode, "recording_error")
        if self._accept_recording and not self.executor.mailbox.enabled:
            self._record_background("interrupted", self.recorder.abort_episode, "control_disabled")

    def _process_actions(self) -> None:
        while True:
            try:
                category, command = self._actions.get_nowait()
            except queue.Empty:
                return
            if category == "control":
                if command == "e":
                    enabled = self.executor.mailbox.enabled
                    self.executor.authorize(not enabled)
                    self.notice = self.executor.mailbox.last_reject_reason or ("Control held" if enabled else "Enable requested")
                elif command == "c":
                    self.executor.clear()
                    self.notice = "Mailbox cleared; holding last targets, no HOME/reset"
                else:
                    self.selected_joint = (self.selected_joint + (1 if command == "f9" else -1)) % len(JOINT_NAMES)
                continue
            if self._record_job is not None:
                self.notice = f"Recording busy: {self._record_operation}"
                continue
            try:
                if command == "start" and not self.recorder.is_busy:
                    if not self.executor.mailbox.enabled:
                        self.notice = "Enable aligned external control before recording"
                        continue
                    # Missing calibrated views are reported, never replaced by duplicated images.
                    if self.camera is None:
                        self.camera = MujocoCameraProvider(
                            self.plant.model, self.plant.data,
                            Path(__file__).resolve().parents[1] / "config" / "sim_cameras.yaml",
                        )
                    while True:
                        self.episode_counter += 1
                        stem = f"episode_{self.episode_counter:06d}"
                        if not any((self.args.output / f"{stem}{suffix}").exists() for suffix in (".h5", ".partial.h5")):
                            break
                    self._record_background(
                        "preparing", self.recorder.start_episode, self.episode_counter,
                        {
                            **(self.plant.scene_manifest or {}),
                            "task": self.args.task, "scene": self.args.scene,
                            "seed": self.args.seed, "artifact_hash": self.plant.artifact_hash,
                        },
                    )
                elif command == "success" and self._accept_recording:
                    self._record_background("saved", self.recorder.finish_episode, success=True)
                elif command == "discard" and self._accept_recording:
                    self._record_background("discarded", self.recorder.discard_episode)
            except Exception as exc:
                self.notice = f"Recording unavailable: {exc}"

    def _record_tick(self, step: Any, applied: Any, now: int) -> None:
        if not self._accept_recording:
            return
        try:
            if applied is not None:
                self.recorder.append_command(
                    now, applied.position_rad,
                    sequence=applied.snapshot.sequence,
                    stamp_utc_ns=applied.snapshot.stamp_ns,
                    ready_mask=applied.snapshot.ready_mask,
                    session_id=applied.snapshot.session_id,
                    applied_sim_time_ns=applied.applied_sim_time_ns,
                    hold_mask=applied.hold_mask,
                )
            if step.tick % max(1, self.plant.physics_hz // 120) == 0:
                qpos = self.plant.joint_command_positions()
                stamp = time.monotonic_ns()
                self.recorder.append_arm_qpos(stamp, qpos[:14])
                self.recorder.append_hand_qpos(stamp, qpos[14:])
            if step.tick % max(1, self.plant.physics_hz // 30) == 0:
                frames = self.camera.capture(step.sim_time_ns)
                self.recorder.append_cameras(frames, available_timestamp_ns=time.monotonic_ns())
        except Exception as exc:
            self.notice = f"Recording interrupted: {exc}"
            self._record_background("interrupted", self.recorder.abort_episode, "recording_error")

    def _update_display(self, now: int) -> None:
        mailbox = self.executor.mailbox
        elapsed = (now - self._rate_ns) * 1e-9
        if elapsed >= 1.0:
            self._receive_hz = (mailbox.received - self._last_received) / elapsed
            self._last_received = mailbox.received
            self._rate_ns = now
        actual = self.plant.joint_command_positions()
        targets = self.plant.joint_command_targets()
        candidate = mailbox.latest
        selected = self.selected_joint
        age_ms = (time.time_ns() - candidate.stamp_ns) * 1e-6 if candidate else None
        values = {
            "Control": self.executor.state,
            "Link": "Direct DDS / Fast DDS (domain 120)",
            "RX": f"{mailbox.received} total / {mailbox.accepted} valid / {mailbox.rejected} rejected ({self._receive_hz:.1f} Hz)",
            "Session": (mailbox.authorized_session or (candidate.session_id if candidate else "none"))[:16],
            "Candidate": f"seq={candidate.sequence} age={age_ms:.1f} ms ready={candidate.ready_mask:03b}" if candidate else "none",
            "Applied": f"seq={self._last_applied.snapshot.sequence}" if self._last_applied else "none",
            "HOLD": f"{self.executor.hold_mask:03b} (LH/RH/arms)",
            "Joint F8 / F9": f"{selected + 1}/54 {JOINT_NAMES[selected]}",
            "Position rad": f"target={targets[selected]:+.4f} actual={actual[selected]:+.4f} error={targets[selected] - actual[selected]:+.4f}",
        }
        errors = [
            float(np.max(np.abs(targets[start:end] - actual[start:end])))
            for start, end in ((0, 7), (7, 14), (14, 34), (34, 54))
        ]
        values["Max error LA/RA/LH/RH"] = " / ".join(f"{error:.3f}" for error in errors)
        values["Rejected"] = (mailbox.last_reject_reason or "none")[:90]
        values["Recording"] = self._record_operation if self._record_job else "ACTIVE" if self._accept_recording else "IDLE"
        values["Keys"] = "e enable/hold, c clear, F8/F9 joint, r/s/d record/save/discard, q quit"
        values["Notice"] = self.notice[:90]
        self.window.update_hud(values)
        self.window.update_joint_plot(JOINT_NAMES[selected], (now - self._started_ns) * 1e-9, targets[selected], actual[selected])
        self.window.sync(now)

    def run(self) -> int:
        import rclpy

        period_ns = 1_000_000_000 // self.plant.physics_hz
        deadline = time.monotonic_ns()
        display_deadline = deadline
        try:
            control_path = getattr(self.args, "control_socket", None)
            if control_path is not None:
                self.local_control = LocalControlServer(control_path, self.executor)
            self.window.open()
            handle = self.window.window
            if handle is not None:
                with handle.lock():
                    if self.plant.scene_manifest is not None:
                        frame_scene(handle.cam, handle.opt)
                    else:
                        visual_positions = self.plant.data.geom_xpos[self.plant.model.geom_group == 1]
                        low, high = visual_positions.min(axis=0), visual_positions.max(axis=0)
                        handle.cam.lookat[:] = (low + high) * 0.5
                        handle.cam.distance = max(2.0, float(np.linalg.norm(high - low)) * 2.0)
                        handle.cam.azimuth = 135.0
                        handle.cam.elevation = -20.0
                        handle.opt.geomgroup[0] = 0  # Hide duplicate collision meshes, not dynamics.
            self.control_terminal.start()
            print(f"SPD subscriber ready: {TOPIC}; explicit local enable required", flush=True)
            if self.plant.scene_model_path is not None:
                print(f"Scene: {self.args.scene}/{self.args.task}; seed={self.args.seed}; "
                      f"model={self.plant.scene_model_path}; manifest={self.plant.scene_manifest_path}", flush=True)
            while not self.stop and rclpy.ok() and self.window.is_running():
                rclpy.spin_once(self.node, timeout_sec=0.0)
                self._process_actions()
                if self.local_control is not None:
                    self.local_control.poll()
                self._poll_recording()
                now = time.monotonic_ns()
                applied = self.executor.apply_pending(now_ns=now)
                if applied is not None:
                    self._last_applied = applied
                step = self.plant.physics_tick(now_ns=now)
                self._record_tick(step, applied, now)
                if now >= display_deadline:
                    self._update_display(now)
                    display_deadline = now + 50_000_000
                deadline += period_ns
                remaining = deadline - time.monotonic_ns()
                if remaining > 0:
                    time.sleep(remaining * 1e-9)
                else:
                    deadline = time.monotonic_ns()
        finally:
            if self.local_control is not None:
                self.local_control.close()
            self.control_terminal.close()
            self._record_worker.shutdown(wait=True)
            self.recorder.close()
            self.window.close()
            self.plant.close()
            if self.camera is not None:
                for renderer in self.camera._renderers.values():
                    renderer.close()
            self.node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--scene", help="Scene name (default: hardware_free)")
    parser.add_argument("--task", help="Task name or qualified SCENE/TASK ID")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--control-socket", type=Path,
                        help="Private local PICO control socket (optional; parent directory must be 0700)")
    parser.add_argument("--max-enable-delta-rad", type=float, default=0.15,
                        help="Simulation-only maximum ready-joint target jump allowed at local enable")
    args = parser.parse_args(argv)
    if not np.isfinite(args.max_enable_delta_rad) or args.max_enable_delta_rad <= 0:
        parser.error("--max-enable-delta-rad must be positive and finite")
    return RosViewerApp(args).run()


if __name__ == "__main__":
    raise SystemExit(main())
