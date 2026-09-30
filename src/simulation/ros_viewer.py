"""Native ROS execution with task, collection status and hand-target feedback."""
from __future__ import annotations

from _spd_native import RosJointCommandExecutor, run_loop

import argparse
from contextlib import ExitStack
import os
from pathlib import Path
import queue
import signal
import sys
import time

from data_collector.config import load_collection_config
from data_collector.ros_control import CollectionRosControl
from data_collector.session import CollectionSession
from interfaces.keyboard_control import ControlTerminal
from interfaces.ros_joint_command import TOPIC
from simulation.collection_control import CollectionControl
from simulation.viewer import PlantController
from simulation.scene import EpisodeTasks, build_selected_scene
from simulation.viewer_window import ViewerWindow
from description.model_builder import config_root


_STATE_ZH = {
    "idle": "待开始", "preparing": "准备中", "preparation_failed": "准备失败",
    "recording": "录制中", "paused": "已暂停", "blending": "接入中",
    "reverting": "回退中", "saving": "保存中",
    "aborting": "保留未完成数据", "discarding": "结束中", "error": "异常",
    "binding": "等待稳定跟踪并绑定", "rebinding": "冻结重绑定",
    "auto_paused": "跟踪丢失，等待 r 重新接手",
}
_NOTICE_ZH = {
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
        self.args = args
        self.stop = False
        self.teleop = None
        self._closed = False
        self._scene_generation = 0
        self._actions: queue.SimpleQueue[tuple[int, str]] = queue.SimpleQueue()
        collection_config = load_collection_config(
            args.collection_config, output=args.output, max_frames=args.max_frames,
        )
        args.output = collection_config.data_dir
        self._task_sequence = EpisodeTasks(args.scene, args.task, args.seed)
        args.scene, args.task, args.seed = self._task_sequence.next()
        self._scene_episode_count = 0
        scene_result = build_selected_scene(args.scene, args.task, args.seed, args.table_distance)
        args.scene = scene_result.scene if scene_result is not None else "hardware_free"
        args.task = scene_result.task if scene_result is not None else "external_joint_command"
        if scene_result is not None:
            args.table_distance = scene_result.table_near_edge_m
        try:
            self.plant = self._create_plant(scene_result)
            self.home_targets = self.plant._command_home.copy()
            self.task_title, self.task_goal = self._task_text(args.scene, args.task, self.plant.scene_manifest)
            self.window = self._create_window(self.plant)
            height_m = getattr(args, "height_m", None)
            if height_m is not None:
                from pico2_hands.collection_session import TeleopSession

                self.teleop = TeleopSession(
                    height_m, host=getattr(args, "host", "127.0.0.1"), port=getattr(args, "port", 10002))
            self.executor = RosJointCommandExecutor(self.plant, subscribe=self.teleop is None)
            self.collection = CollectionSession(
                collection_config, self.plant, self.executor,
                self._task_manifest(self.plant, args.scene, args.task, args.seed),
            )
            self.collection_ros = CollectionRosControl(self.executor, self.collection)
            self.control_terminal = ControlTerminal(self.joint_control)
            self.three_key = CollectionControl(self)
        except BaseException:
            self.close()
            raise

    @staticmethod
    def _task_text(scene: str, task: str, scene_manifest: dict | None) -> tuple[str, str]:
        if scene == "hardware_free":
            return "自由仿真", "无预设物体操作任务；按统一采集按键开始。"
        from spd_envs.registry import get_task

        spec = get_task(scene, task)
        sampled = (scene_manifest or {}).get("sampled_values", {})
        return spec.title_zh, sampled.get("task_goal_zh", spec.goal_zh)

    def _task_manifest(self, plant, scene: str, task: str, seed: int) -> dict:
        title, goal = self._task_text(scene, task, plant.scene_manifest)
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

    def _create_window(self, plant, task_text=None, *, generation=None):
        generation = self._scene_generation if generation is None else generation
        window = ViewerWindow(
            plant.model, plant.data, headless=self.args.headless, split_view=True,
            shutdown=self.request_stop,
            joint_control=lambda key: self._queue_control(key, generation),
        )
        window.set_task(*(task_text or (self.task_title, self.task_goal)))
        return window

    def _open_window(self) -> None:
        self.window.open()
        self.window.frame(self.args.table_distance if self.plant.scene_manifest is not None else None)

    def _announce_task(self) -> None:
        mode = "随机任务；结束后全目录重抽" if self._task_sequence.randomized else "首条指定任务；结束后全目录随机"
        print(f"任务：{self.task_title}；目标：{self.task_goal}", flush=True)
        print(f"Scene: {self.args.scene}/{self.args.task}; seed={self.args.seed}; {mode}", flush=True)
        if self.plant.scene_manifest is not None:
            table = self.plant.scene_manifest["table"]
            print(f"桌高={table['top_z_m']:.3f} m；桌沿 X={table['near_edge_x_m']:.3f} m；本场景固定。", flush=True)


    def _replace_scene(self, result, scene: str, task: str, seed: int) -> None:
        self.collection.control_paused = True
        self.executor.clear()
        title, goal = self._task_text(scene, task, result.manifest() if result else None)
        with ExitStack() as rollback:
            plant = self._create_plant(result)
            rollback.callback(plant.close)
            executor = RosJointCommandExecutor(plant, subscribe=self.teleop is None)
            rollback.callback(executor.close)
            window = self._create_window(plant, (title, goal), generation=self._scene_generation + 1)
            rollback.callback(window.close)
            # GLFW initialization/termination is process-global. Join the old
            # renderer before creating the next window or its teardown destroys
            # the new context (BadWindow). Plant state remains owned until commit.
            self.window.close()
            if not self.stop:
                window.open()
                window.frame(result.table_near_edge_m if result is not None else None)
            manifest = self._task_manifest(plant, scene, task, seed)
            self.collection.replace_scene(plant, executor, manifest)
            old_plant, old_executor, old_window = self.plant, self.executor, self.window
            self.plant, self.executor, self.window = plant, executor, window
            self.collection_ros.executor = executor
            self.task_title, self.task_goal = title, goal
            self._scene_generation += 1
            self.home_targets = plant._command_home.copy()
            rollback.pop_all()
        # Release renderer and command users before destroying their plant. Every
        # cleanup runs even if an earlier resource reports an error.
        with ExitStack() as cleanup:
            cleanup.callback(old_plant.close)
            cleanup.callback(old_executor.close)
            cleanup.callback(old_window.close)
        self._discard_scene_actions()
        self.collection_ros.publish()

    def next_task_after_episode(self) -> None:
        """Replace a completed save or discard with a fresh random Home task."""
        if (self.collection.state != "idle" or self.collection.last_outcome not in {"saved", "discarded"}
                or self.collection.completed_episodes <= self._scene_episode_count):
            raise RuntimeError("next task requires a newly completed, closed episode")
        self._load_task(*self._task_sequence.next(), None)

    def _load_task(self, scene, task, seed, table_distance):
        result = build_selected_scene(scene, task, seed, table_distance)
        scene = result.scene if result is not None else "hardware_free"
        task = result.task if result is not None else "external_joint_command"
        self._replace_scene(result, scene, task, seed)
        self.args.scene, self.args.task, self.args.seed = scene, task, seed
        self.args.table_distance = result.table_near_edge_m if result is not None else table_distance
        self._scene_episode_count = self.collection.completed_episodes
        if not self.stop:
            self._announce_task()

    def _discard_scene_actions(self) -> None:
        # Input queued against the previous scene cannot authorize this one.
        while True:
            try:
                _, key = self._actions.get_nowait()
            except queue.Empty:
                return
            if key == "q":
                self.request_stop()

    def request_stop(self) -> None:
        self.stop = True

    def _queue_control(self, key: str, generation: int) -> None:
        # Callbacks may race disk completion or scene replacement. Do not carry
        # busy-state presses or an old window's authority into the next scene.
        control = getattr(self, "three_key", None)
        if key != "q" and (generation != self._scene_generation or
                          (control is not None and control.stage in {"saving", "discarding", "reverting", "error"})):
            return
        self._actions.put((generation, key))

    def joint_control(self, key: str) -> None:
        self._queue_control(key, self._scene_generation)

    def _process_actions(self) -> None:
        while True:
            try:
                generation, key = self._actions.get_nowait()
            except queue.Empty:
                return
            if key != "q" and generation != self._scene_generation:
                continue
            previous_stage, previous_notice = self.three_key.stage, self.three_key.notice
            self.three_key.key(key)
            if self.stop:
                return
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
        flags = self.three_key.control_flags
        local = self.teleop.snapshot() if self.teleop is not None else None
        left_mode, right_mode = local.finger_modes if local is not None else ("live", "live")
        finger_flags = local.control_flags if local is not None else flags
        auto_checkpoint = status["auto_checkpoint_frames"]
        values = {
            "状态": f"{state}    帧数：{self.collection.state_frames}    检查点：{checkpoint}",
            "提示": _notice_zh(self.three_key.notice),
            "双手": (("右手：" + ("等待有效输入" if right_mode == "waiting" else "短时保持"
                                if finger_flags & 2 else "平滑接入" if right_mode == "blend" else "就绪")
                      + "；左手：" + ("等待有效输入" if left_mode == "waiting" else "短时保持"
                                   if finger_flags & 4 else "平滑接入" if left_mode == "blend" else "就绪"))
                     if local is not None else "由外部源控制；本地无法判断手指输入有效性"),
            "跟踪": ("双臂输入降级" if flags & 1 else "本地相对绑定"
                     if self.teleop is not None else "外部订阅；无本地绑定/手指重接入控制"),
        }
        if stage == "idle":
            values["双手"] = "机器人停在 Home；双手放在腰间，按 r 重新绑定并开始"
        if auto_checkpoint is not None:
            values["失跟踪现场"] = f"帧 {auto_checkpoint}；r 仅重新接手，不回退、不更新保存点"
        error = self.collection.error or mailbox.last_reject_reason
        if local is not None:
            error = error or local.fault
        if error:
            values["异常"] = _notice_zh(error)
        ghost = None
        ghost_label = ""
        if stage == "auto_paused":
            ghost = self.collection.auto_checkpoint_targets
            ghost_label = "停住的机器人姿态"
        elif stage in {"paused", "reverting"}:
            ghost = self.collection.checkpoint_targets
            ghost_label = "当前保存点"
        elif (stage in {"binding", "rebinding"}
              or (stage == "recording" and local is not None and local.finger_modes != ("live", "live"))):
            if local is not None:
                ghost = local.position_rad
            elif candidate is not None and 0 <= time.time_ns() - candidate.stamp_ns <= 100_000_000:
                ghost = candidate.position_rad
            ghost_label = "保持／平滑接入目标"
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
            if self.teleop is None:
                print(f"SPD subscriber ready: {TOPIC}; 外部源独立运行，不提供本地相对绑定保证。", flush=True)
            else:
                print("SPD local collection ready: 本进程拥有 PICO 接收、双臂/双手求解与采集控制。", flush=True)
            print("待开始/失跟踪：r 开始或重新接手；运动：s 暂停，r 存检查点，d 回退并自动续采；"
                  "人工暂停：s 重新绑定继续，r 保存整条，d 丢弃整条。s 继续和 d 回退均无需额外按 r。"
                  "单键按下立即生效；q/Esc/Ctrl+C 退出但不保存。"
                  "保存或丢弃完成后直接随机新任务、Home 等待 r。", flush=True)
            self._announce_task()
            run_loop(self)
        finally:
            try:
                self.close()
            finally:
                for signum, handler in previous_handlers.items():
                    signal.signal(signum, handler)
        return 0

    def close(self) -> None:
        """Close all owners, including a local backend, even after partial startup."""
        if self._closed:
            return
        self._closed = True
        self.stop = True
        with ExitStack() as cleanup:
            for name in ("plant", "executor", "window", "collection", "teleop",
                         "control_terminal", "three_key"):
                resource = getattr(self, name, None)
                if resource is not None:
                    cleanup.callback(resource.close)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--collection-config", type=Path, default=config_root() / "collect_sim.yaml")
    parser.add_argument("--output", type=Path, default=os.environ.get("SPD_EPISODE_OUTPUT") or None,
                        help="Override collection config data_dir")
    parser.add_argument("--max-frames", type=int, help="Override state sample limit (0: unlimited)")
    parser.add_argument("--scene", help="Select the first task's scene; later saves sample all scenes")
    parser.add_argument("--task", help="Select the first task SCENE/TASK; later saves sample all tasks")
    parser.add_argument("--seed", type=int, help="Reproduce the task/scene sequence; explicit first tasks default to 0")
    parser.add_argument("--table-distance", type=float,
                        help="Override first task's near table edge distance; later tasks sample 0.10–0.30 m")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--height-m", type=float,
                        help="Own local PICO teleoperation in this process, with user height in metres")
    parser.add_argument("--host", default="127.0.0.1", help="Headset TCP host for local teleop")
    parser.add_argument("--port", type=int, default=10002, help="Headset TCP port for local teleop")
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
