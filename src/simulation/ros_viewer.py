"""Native ROS execution with task, collection status and hand-target feedback."""
from __future__ import annotations

from _spd_native import RosJointCommandExecutor, ThreeKeyControl, run_loop

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
from interfaces.keyboard_control import ControlTerminal
from interfaces.ros_joint_command import TOPIC
from simulation.viewer import PlantController
from simulation.scene import EpisodeTasks, build_selected_scene, frame_scene
from simulation.viewer_window import ViewerWindow
from description.model_builder import config_root

_STATE_ZH = {
    "idle": "待开始", "preparing": "准备中", "preparation_failed": "准备失败",
    "recording": "录制中", "paused": "已暂停", "blending": "接入中",
    "reverting": "回退中", "rewind_wait": "等待恢复目标", "saving": "保存中",
    "aborting": "保留未完成数据", "discarding": "结束中", "error": "异常",
}
_NOTICE_ZH = {
    "Upstream r calibrates / s follows; local r starts an episode": "上游 r 标定、s 跟随；本窗口按 r 开始",
    "Next task ready; r starts a new episode and checkpoint 0": "新任务已就绪；按 r 开始并建立初始检查点",
    "Opening episode with checkpoint 0; no motion until ready": "正在准备采集和初始检查点，请稍候",
    "1 second live-target transition; r/s/d ignored; recovery samples labelled": "正在接入（1 秒），暂不接受 r/s/d",
    "r checkpoint; s pause; d is available only while paused": "r 存检查点；s 暂停",
    "Paused: checkpoint hand ghost shown; s resumes, d rewinds, r then r saves": "s 继续；d 回退；连按两次 r 保存",
    "Press r again to confirm successful completion; s resumes, d rewinds": "再次按 r 确认成功保存；s 继续；d 回退",
    "Saving successful episode; next task waits for r": "正在保存，下一任务等待按 r 开始",
    "Restoring checkpoint; automatic 1 second recovery follows": "正在恢复检查点，随后自动接入",
    "Checkpoint restored; waiting for fresh target to auto-resume": "检查点已恢复，等待新目标后自动继续",
    "Waiting for upstream targets: calibrate with r and start with s upstream": "等待上游目标：请先在上游按 r 标定、s 跟随",
    "Upstream arm targets are not ready": "上游双臂目标尚未就绪",
    "Upstream joint candidate is stale": "上游关节目标已过期",
    "Motion authorization was revoked": "运动授权已撤销",
    "Upstream session changed": "上游会话已变更",
    "Arm targets are held": "双臂目标已保持，等待重新授权",
    "Upstream session changed during rewind; explicit s required": "回退期间上游会话变更，请按 s 重新接入",
    "No fresh target after rewind; explicit s required": "回退后未收到新目标，请按 s 重新接入",
    "Collector is not ready for the transition": "采集器尚未就绪，无法接入",
    "Local collector stopping; upstream continues independently": "本地采集正在停止，上游继续独立运行",
    "checkpoint accepted; completion is reported on status": "检查点已更新；r 再次更新，s 暂停",
    "Checkpoint rejected: a hand is in contact with a task object": "无法更新检查点：手仍接触任务物体",
    "No checkpoint in this episode": "本段还没有检查点",
    "Waiting for the first actual whole-scene trajectory sample": "等待首帧场景数据",
    "no legal command candidate": "尚无合法关节目标",
    "candidate has no ready groups": "目标的控制分组尚未就绪",
    "command is older than 100 ms": "关节目标已过期（超过 100 毫秒）",
    "command is more than 5 ms in the future": "目标时间戳超前超过 5 毫秒，请检查时钟同步",
    "session changed; explicit authorization required": "上游会话已变更，需要重新授权",
    "ready groups changed during transition; explicit authorization required": "接入期间控制分组变更，需要重新授权",
    "ready groups changed during transition": "接入期间控制分组变更",
    "transition lost its authorized session": "接入期间原会话授权失效",
    "position_rad must contain 54 finite values": "关节目标必须包含 54 个有限数值",
    "joint_names do not match the canonical 54-DoF order": "关节名称或顺序不符合 54 关节约定",
    "sequence is not strictly increasing": "命令序号未严格递增",
    "stamp_ns rolled back": "命令时间戳发生回退",
}


def _notice_zh(text: str) -> str:
    """Localize operator messages; preserve unrecognized diagnostic details."""
    if text in _NOTICE_ZH:
        return _NOTICE_ZH[text]
    for suffix, translated in (
        ("; explicit s required after recovery", "；恢复后按 s 重新接入"),
        ("; r starts the next episode", "；按 r 开始下一段"),
        ("; frozen, restart required", "；已冻结，请重启"),
        ("; partial episode preserved", "；未完成数据已保留"),
        ("; preserving partial episode", "；正在保留未完成数据"),
    ):
        if text.endswith(suffix):
            return _notice_zh(text[:-len(suffix)]) + translated
    if text.startswith("Saved episode: "):
        return "本段已保存"
    for prefix, translated in (
        ("Rewind did not complete: ", "回退未完成："),
        ("Collector is ", "采集状态："),
    ):
        if text.startswith(prefix):
            detail = text[len(prefix):]
            return translated + (_STATE_ZH.get(detail) or _notice_zh(detail))
    return text


class RosViewerApp:
    def __init__(self, args: argparse.Namespace) -> None:
        collection_config = load_collection_config(
            args.collection_config, output=args.output, max_frames=args.max_frames,
        )
        args.output = collection_config.data_dir
        self._task_sequence = EpisodeTasks(args.scene, args.task, args.seed)
        args.scene, args.task, args.seed = self._task_sequence.next()
        self._scene_episode_count = 0
        self._table_distance_override = args.table_distance
        scene_result = build_selected_scene(args.scene, args.task, args.seed, args.table_distance)
        self.args = args
        self.stop = False
        self._actions: queue.SimpleQueue[tuple[str, str]] = queue.SimpleQueue()
        args.scene = scene_result.scene if scene_result is not None else "hardware_free"
        args.task = scene_result.task if scene_result is not None else "external_joint_command"
        if scene_result is not None:
            args.table_distance = scene_result.table_near_edge_m
        self.task_title, self.task_goal = self._task_text(args.scene, args.task)
        self.plant = self._create_plant(scene_result)
        self.window = self._create_window(self.plant)
        self.executor = RosJointCommandExecutor(self.plant)
        self.collection = CollectionSession(
            collection_config, self.plant, self.executor,
            self._task_manifest(self.plant, args.scene, args.task, args.seed),
        )
        self.collection_ros = CollectionRosControl(self.executor, self.collection)
        self.control_terminal = ControlTerminal(self.joint_control, self.recording_control)
        self.three_key = ThreeKeyControl(self.collection, self.executor)

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
        if self.plant.scene_manifest is not None:
            table = self.plant.scene_manifest["table"]
            print(f"桌高={table['top_z_m']:.3f} m；桌沿 X={table['near_edge_x_m']:.3f} m；本场景固定。", flush=True)

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
        result = build_selected_scene(scene, task, seed, self._table_distance_override)
        plant = self._create_plant(result)
        executor = None
        try:
            plant.inherit_robot_state(self.plant)
            executor = RosJointCommandExecutor(plant)
            self.collection.replace_scene(plant, executor, self._task_manifest(plant, scene, task, seed))
        except BaseException:
            if executor is not None:
                executor.close()
            plant.close()
            raise
        self.window.close()
        self.executor.close()
        self.plant.close()
        self.plant, self.executor = plant, executor
        self.collection_ros.executor = executor
        self.args.scene, self.args.task, self.args.seed = scene, task, seed
        self.args.table_distance = result.table_near_edge_m
        self.task_title, self.task_goal = self._task_text(scene, task)
        self.window = self._create_window(plant)
        self.three_key = ThreeKeyControl(self.collection, executor)
        self.three_key.notice = "Next task ready; r starts a new episode and checkpoint 0"
        self._scene_episode_count = self.collection.completed_episodes
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
        candidate = mailbox.latest
        status = self.collection.snapshot()
        checkpoint_frames = status["checkpoint_frames"]
        stage = self.three_key.stage
        state = _STATE_ZH.get(stage, stage)
        if self.collection.physics_paused and stage == "recording":
            state = "已暂停"
        checkpoint = str(checkpoint_frames) if checkpoint_frames is not None else "无"
        values = {
            "状态": f"{state}    帧数：{self.collection.state_frames}    检查点：{checkpoint}",
            "提示": _notice_zh(self.three_key.notice),
        }
        error = self.collection.error or mailbox.last_reject_reason
        if error:
            values["异常"] = _notice_zh(error)
        ghost = None
        ghost_label = ""
        if self.three_key.stage in {"paused", "reverting", "rewind_wait"}:
            ghost = self.collection.checkpoint_targets
            ghost_label = "检查点目标"
        elif self.three_key.stage == "blending":
            if candidate is not None and 0 <= time.time_ns() - candidate.stamp_ns <= 100_000_000:
                ghost = candidate.position_rad
            ghost_label = "实时目标（1 秒接入）"
        self.window.set_hand_ghost(ghost, label=ghost_label)
        self.window.update_hud(values)
        self.window.sync(now)

    def run(self) -> int:
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
            run_loop(self)
        finally:
            self.three_key.close()
            self.control_terminal.close()
            self.collection.close()
            self.window.close()
            self.plant.close()
            self.executor.close()
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
                        help="Override sampled near table edge distance along +X, metres (default: random 0.10–0.30)")
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
