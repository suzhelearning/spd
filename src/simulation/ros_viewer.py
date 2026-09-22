"""External ROS joint targets -> MuJoCo physics, live comparison, and recording."""
from __future__ import annotations

# Load ROS's native extension before simulation/compiler native dependencies.
# Importing rclpy after those libraries can resolve incompatible C++ symbols.
try:
    import rclpy
except ImportError:  # The non-ROS environment can still import simulation modules.
    rclpy = None

import argparse
import os
from pathlib import Path
import queue
import signal
import sys
import time

import numpy as np

from data_collector.config import load_collection_config
from data_collector.ros_control import CollectionRosControl
from data_collector.session import CollectionSession
from interfaces.ros_executor import ControlTerminal, RosJointCommandExecutor
from interfaces.ros_joint_command import JOINT_NAMES, TOPIC
from simulation.viewer import PlantController
from simulation.scene import build_selected_scene, frame_scene
from simulation.viewer_window import ViewerWindow
from description.model_builder import config_root


class RosViewerApp:
    def __init__(self, args: argparse.Namespace) -> None:
        import rclpy

        collection_config = load_collection_config(
            args.collection_config, output=args.output, max_frames=args.max_frames,
        )
        args.output = collection_config.data_dir
        scene_result = build_selected_scene(args.scene, args.task, args.seed, args.table_distance)
        self.args = args
        self.stop = False
        self.selected_joint = 0
        self.notice = "Waiting for an external publisher; e enables a fresh aligned candidate"
        self._actions: queue.SimpleQueue[tuple[str, str]] = queue.SimpleQueue()
        args.scene = scene_result.scene if scene_result is not None else "hardware_free"
        args.task = scene_result.task if scene_result is not None else "external_joint_command"
        if scene_result is not None:
            args.table_distance = scene_result.table_near_edge_m
        self.plant = PlantController(
            strict_artifacts=True,
            camera_config_path=config_root() / "sim_cameras.yaml",
            scene_result=scene_result, scene_output_dir=args.output / "scenes",
        )
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
        self.collection = CollectionSession(
            collection_config, self.plant, self.executor,
            {
                **(self.plant.scene_manifest or {}),
                "task": args.task, "scene": args.scene, "seed": args.seed,
                "artifact_hash": self.plant.artifact_hash,
                "collection_config_path": str(args.collection_config.expanduser().resolve()),
            },
        )
        self.collection_ros = CollectionRosControl(self.node, self.collection)
        self.control_terminal = ControlTerminal(self.executor, self.recording_control)
        self._started_ns = time.monotonic_ns()
        self._last_received = 0
        self._rate_ns = self._started_ns
        self._receive_hz = 0.0
        self._last_applied = None

    def request_stop(self) -> None:
        self.stop = True

    def recording_control(self, command: str) -> None:
        # Viewer/stdin callbacks run off-thread; keep all model access here.
        self._actions.put(("record", command))

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
            operation = "save" if command == "success" else command
            _, response = self.collection.request(operation)
            self.notice = response["message"]

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
        if self.plant.scene_manifest is not None:
            values["Table"] = f"near edge X={self.args.table_distance:g} m; fixed"
        errors = [
            float(np.max(np.abs(targets[start:end] - actual[start:end])))
            for start, end in ((0, 7), (7, 14), (14, 34), (34, 54))
        ]
        values["Max error LA/RA/LH/RH"] = " / ".join(f"{error:.3f}" for error in errors)
        values["Rejected"] = (mailbox.last_reject_reason or "none")[:90]
        values["Recording"] = f"{self.collection.state} / states={self.collection.state_frames}"
        values["Collection"] = self.collection.message[:90]
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
        previous_handlers = {
            signum: signal.signal(signum, lambda _signum, _frame: self.request_stop())
            for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)
        }
        try:
            self.window.open()
            handle = self.window.window
            if handle is not None:
                with handle.lock():
                    if self.plant.scene_manifest is not None:
                        frame_scene(handle.cam, handle.opt, self.args.table_distance)
                    else:
                        visual_positions = self.plant.data.geom_xpos[self.plant.model.geom_group == 1]
                        low, high = visual_positions.min(axis=0), visual_positions.max(axis=0)
                        handle.cam.lookat[:] = (low + high) * 0.5
                        handle.cam.distance = max(2.0, float(np.linalg.norm(high - low)) * 2.0)
                        handle.cam.azimuth = 135.0
                        handle.cam.elevation = -20.0
                        handle.opt.geomgroup[0] = 0  # Hide duplicate collision meshes, not dynamics.
                        handle.opt.geomgroup[3] = 0
            self.control_terminal.start()
            print(f"SPD subscriber ready: {TOPIC}; explicit local enable required", flush=True)
            if self.plant.scene_model_path is not None:
                print(f"Scene: {self.args.scene}/{self.args.task}; seed={self.args.seed}; "
                      f"table near edge X={self.args.table_distance:g} m; "
                      f"model={self.plant.scene_model_path}; manifest={self.plant.scene_manifest_path}", flush=True)
            while not self.stop and rclpy.ok() and self.window.is_running():
                rclpy.spin_once(self.node, timeout_sec=0.0)
                self._process_actions()
                self.collection.poll()
                now = time.monotonic_ns()
                applied = self.executor.apply_pending(now_ns=now)
                if applied is not None:
                    self._last_applied = applied
                step = self.plant.physics_tick()
                self.collection.tick(step)
                self.collection_ros.heartbeat()
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
            self.control_terminal.close()
            self.collection.close()
            self.window.close()
            self.plant.close()
            self.node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            for signum, handler in previous_handlers.items():
                signal.signal(signum, handler)
        return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--collection-config", type=Path, default=config_root() / "collect_sim.yaml")
    parser.add_argument("--output", type=Path, default=os.environ.get("SPD_EPISODE_OUTPUT") or None,
                        help="Override collection config data_dir")
    parser.add_argument("--max-frames", type=int, help="Override state sample limit (0: unlimited)")
    parser.add_argument("--scene", help="Scene name (default: hardware_free)")
    parser.add_argument("--task", help="Task name or qualified SCENE/TASK ID")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--table-distance", type=float,
                        help="Base origin to near table edge along +X, metres; prompts for table scenes")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--max-enable-delta-rad", type=float, default=0.15,
                        help="Simulation-only maximum ready-joint target jump allowed at local enable")
    args = parser.parse_args(argv)
    if not np.isfinite(args.max_enable_delta_rad) or args.max_enable_delta_rad <= 0:
        parser.error("--max-enable-delta-rad must be positive and finite")
    try:
        app = RosViewerApp(args)
    except ValueError as exc:
        parser.error(str(exc))
    except (EOFError, KeyboardInterrupt):
        print("\n已取消，未打开场景。", file=sys.stderr)
        return 130
    return app.run()


if __name__ == "__main__":
    raise SystemExit(main())
