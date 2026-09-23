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
from interfaces.three_key_control import ThreeKeyControl
from simulation.viewer import PlantController
from simulation.scene import EpisodeTasks, build_selected_scene, frame_scene
from simulation.viewer_window import ViewerWindow
from description.model_builder import config_root


class RosViewerApp:
    def __init__(self, args: argparse.Namespace) -> None:
        import rclpy

        collection_config = load_collection_config(
            args.collection_config, output=args.output, max_frames=args.max_frames,
        )
        args.output = collection_config.data_dir
        self._task_sequence = EpisodeTasks(args.scene, args.task, args.seed)
        args.scene, args.task, args.seed = self._task_sequence.next()
        self._scene_episode_count = 0
        scene_result = build_selected_scene(args.scene, args.task, args.seed, args.table_distance)
        self.args = args
        self.stop = False
        self.selected_joint = 0
        self.notice = "Start upstream with r then s; local r begins recording"
        self._actions: queue.SimpleQueue[tuple[str, str]] = queue.SimpleQueue()
        args.scene = scene_result.scene if scene_result is not None else "hardware_free"
        args.task = scene_result.task if scene_result is not None else "external_joint_command"
        if scene_result is not None:
            args.table_distance = scene_result.table_near_edge_m
        self.task_title, self.task_goal = self._task_text(args.scene, args.task)
        self.plant = self._create_plant(scene_result)
        self.window = self._create_window(self.plant)
        rclpy.init()
        self.node = rclpy.create_node("spd_mujoco_joint_command_executor")
        self.executor = RosJointCommandExecutor(
            self.node, self.plant,
        )
        self.collection = CollectionSession(
            collection_config, self.plant, self.executor,
            self._task_manifest(self.plant, args.scene, args.task, args.seed),
        )
        self.collection_ros = CollectionRosControl(
            self.node, self.collection, allow_requests=False,
        )
        self.control_terminal = ControlTerminal(self.joint_control, self.recording_control)
        self.three_key = ThreeKeyControl(self.collection, self.executor)
        self._started_ns = time.monotonic_ns()
        self._last_received = 0
        self._rate_ns = self._started_ns
        self._receive_hz = 0.0
        self._last_applied = None

    @staticmethod
    def _task_text(scene: str, task: str) -> tuple[str, str]:
        if scene == "hardware_free":
            return "自由仿真", "等待外部关节命令，无预设物体操作任务。"
        from spd_envs.registry import get_task

        spec = get_task(scene, task)
        return spec.title_zh, spec.goal_zh

    def _task_manifest(self, plant, scene: str, task: str, seed: int) -> dict:
        title, goal = self._task_text(scene, task)
        return {
            **(plant.scene_manifest or {}),
            "task": task, "scene": scene, "seed": seed,
            "task_title_zh": title, "task_goal_zh": goal,
            "artifact_hash": plant.artifact_hash,
            "collection_config_path": str(self.args.collection_config.expanduser().resolve()),
        }

    def _create_plant(self, result):
        return PlantController(
            strict_artifacts=True, camera_config_path=config_root() / "sim_cameras.yaml",
            scene_result=result, scene_output_dir=self.args.output / "scenes",
        )

    def _create_window(self, plant):
        window = ViewerWindow(
            plant.model, plant.data, headless=self.args.headless,
            shutdown=self.request_stop, recording_control=self.recording_control,
            joint_control=self.joint_control,
        )
        window.set_task(self.task_title, self.task_goal)
        return window

    def _open_window(self) -> None:
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
                    handle.opt.geomgroup[0] = 0
                    handle.opt.geomgroup[3] = 0

    def _announce_task(self) -> None:
        mode = "每段随机" if self._task_sequence.randomized else "固定任务"
        print(f"任务：{self.task_title}；目标：{self.task_goal}", flush=True)
        print(f"Scene: {self.args.scene}/{self.args.task}; seed={self.args.seed}; {mode}", flush=True)

    def _task_change_pending(self) -> bool:
        return (self._task_sequence.randomized
                and self.collection.completed_episodes != self._scene_episode_count)

    def _maybe_rotate_task(self) -> None:
        if not self._task_change_pending() or self.collection.state != "idle" or self.stop:
            return
        if self.three_key.stage != "idle":
            return
        print("正在随机切换下一段任务；清除授权，保留机器人姿态，重置任务物体。", flush=True)
        self.executor.clear()
        scene, task, seed = self._task_sequence.next()
        result = build_selected_scene(scene, task, seed, self.args.table_distance)
        plant = self._create_plant(result)
        executor = None
        try:
            plant.inherit_robot_state(self.plant)
            executor = RosJointCommandExecutor(
                self.node, plant,
            )
            self.collection.replace_scene(plant, executor, self._task_manifest(plant, scene, task, seed))
        except BaseException:
            if executor is not None:
                self.node.destroy_subscription(executor.subscription)
            plant.close()
            raise
        self.window.close()
        self.node.destroy_subscription(self.executor.subscription)
        self.plant.close()
        self.plant, self.executor = plant, executor
        self.args.scene, self.args.task, self.args.seed = scene, task, seed
        self.task_title, self.task_goal = self._task_text(scene, task)
        self.window = self._create_window(plant)
        self.three_key = ThreeKeyControl(self.collection, executor)
        self.three_key.notice = "Next task ready; r starts a new episode and checkpoint 0"
        self._scene_episode_count = self.collection.completed_episodes
        self._last_applied = None
        self._last_received = 0
        self._receive_hz = 0.0
        self._rate_ns = time.monotonic_ns()
        self._discard_scene_actions()
        if not self.stop:
            self._open_window()
            self._announce_task()
        self.collection_ros.publish()

    def _discard_scene_actions(self) -> None:
        # Input queued against the previous scene cannot authorize this one.
        while not self._actions.empty():
            category, command = self._actions.get_nowait()
            if category == "control" and command == "q":
                self.request_stop()

    def request_stop(self) -> None:
        self.stop = True

    def recording_control(self, command: str) -> None:
        # Viewer/stdin callbacks run off-thread; queue all model access.
        if self.three_key.stage == "blending":
            return  # Never replay transition-time key presses at the endpoint.
        self._actions.put(("record", command))

    def joint_control(self, key: str) -> None:
        self._actions.put(("control", key))

    def _process_actions(self) -> None:
        while True:
            try:
                category, command = self._actions.get_nowait()
            except queue.Empty:
                return
            if category == "control":
                if command == "q":
                    self.request_stop()
                elif command in {"f8", "f9"}:
                    self.three_key.cancel_confirmation()
                    self.selected_joint = (self.selected_joint + (1 if command == "f9" else -1)) % len(JOINT_NAMES)
                continue
            key = {"checkpoint": "r", "pause_toggle": "s", "revert": "d"}.get(command)
            if key is None:
                continue
            previous_stage, previous_notice = self.three_key.stage, self.three_key.notice
            self.three_key.key(key)
            if self.three_key.stage == previous_stage and self.three_key.notice != previous_notice:
                print(f"SPD keys [{self.three_key.stage}]: {self.three_key.notice}", flush=True)

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
        status = self.collection.snapshot()
        values["Recording"] = f"{self.collection.state} / states={self.collection.state_frames}"
        values["Physics"] = "paused" if self.collection.physics_paused else "running"
        checkpoint_frames = status["checkpoint_frames"]
        values["Checkpoint"] = f"states={checkpoint_frames}" if checkpoint_frames is not None else "none"
        values["Operation"] = f"{status['operation']} / {status['operation_id']}"
        values["Collection"] = self.collection.message[:90]
        values["Keys"] = "r start/checkpoint; s pause/resume; paused d rewind; q exit"
        values["Three-key"] = self.three_key.stage + " / " + self.three_key.notice
        values["Save"] = "While paused: r then r confirms success; no discard"
        ghost = None
        ghost_label = ""
        if self.three_key.stage in {"paused", "reverting", "rewind_wait"}:
            ghost = self.collection.checkpoint_targets
            values["Hand ghost"] = "checkpoint target"
            ghost_label = "检查点目标"
        elif self.three_key.stage == "blending":
            if candidate is not None and 0 <= time.time_ns() - candidate.stamp_ns <= 100_000_000:
                ghost = candidate.position_rad
            values["Hand ghost"] = "live upstream target / 1 second transition"
            ghost_label = "实时目标（1 秒接入）"
        self.window.set_hand_ghost(ghost, label=ghost_label)
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
            self._open_window()
            self.control_terminal.start()
            print(f"SPD subscriber ready: {TOPIC}; upstream remains independent", flush=True)
            print("r starts/checkpoints; s pauses/resumes; paused d rewinds and auto-resumes. "
                  "All entries blend for 1 second; r/s/d ignored during blending. "
                  "Paused r then r saves success. q/Ctrl+C exits without saving.", flush=True)
            self._announce_task()
            while not self.stop and rclpy.ok() and self.window.is_running():
                rclpy.spin_once(self.node, timeout_sec=0.0)
                self.three_key.poll()
                self._maybe_rotate_task()
                if self._task_change_pending():
                    self._discard_scene_actions()
                else:
                    self._process_actions()
                self.collection.poll()
                now = time.monotonic_ns()
                if not self.collection.physics_paused and not self._task_change_pending():
                    applied = self.executor.apply_pending(now_ns=now)
                    if applied is not None:
                        self._last_applied = applied
                    # A physics-time validation failure must not advance or
                    # record even one uncontrolled step before the next poll.
                    if not self.executor.mailbox.enabled or self.executor.hold_mask & 1:
                        self.three_key.poll()
                        continue
                    step = self.plant.physics_tick()
                    self.collection.tick(step, recovery=self.three_key.recovery)
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
            self.three_key.close()
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
    parser.add_argument("--scene", help="Restrict random tasks to this scene; hardware_free disables tasks")
    parser.add_argument("--task", help="Fixed task SCENE/TASK; omitted: random task for each episode")
    parser.add_argument("--seed", type=int, help="Reproduce random task/scene sequence; fixed tasks default to 0")
    parser.add_argument("--table-distance", type=float,
                        help="Base origin to near table edge along +X, metres; prompts for table scenes")
    parser.add_argument("--headless", action="store_true")
    args = parser.parse_args(argv)
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
