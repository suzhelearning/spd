"""Launch a deterministic task scene with the verified robot; no autonomous policy."""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import random
import sys
import time
from typing import Any


def resolve_table_distance(value: float | None) -> float:
    """Resolve the near table edge before starting the simulation viewer."""
    if value is not None:
        if not math.isfinite(value) or value < 0:
            raise ValueError("桌沿距离必须是有限的非负数（单位 m）。")
        return value
    if not sys.stdin.isatty():
        raise ValueError("带桌子的场景需要交互输入；非交互启动请指定 --table-distance METRES。")
    while True:
        print(
            "桌子近侧边缘距离机器人底座原点多少 m？沿 +X 测量，"
            "桌上物体一起平移，进入后固定。[回车 = 0.10 m]\n> ",
            end="", file=sys.stderr, flush=True,
        )
        selected = input().strip()
        try:
            distance = float(selected) if selected else 0.10
            return resolve_table_distance(distance)
        except ValueError:
            print("请输入有限的非负数，例如 0.25；Ctrl+C 取消。", file=sys.stderr)


class EpisodeTasks:
    """Choose a task and scene seed per episode, or retain an explicit task."""

    def __init__(self, scene: str | None, task: str | None, seed: int | None) -> None:
        from spd_envs.registry import TASKS

        self.randomized = task is None and scene != "hardware_free"
        self._fixed = (scene, task, 0 if seed is None else seed)
        self._rng = random.Random(seed)
        self._tasks = tuple(spec for spec in TASKS if scene is None or spec.scene == scene)
        if self.randomized and not self._tasks:
            raise ValueError(f"unknown task scene: {scene}")

    def next(self) -> tuple[str | None, str | None, int]:
        if not self.randomized:
            return self._fixed
        spec = self._rng.choice(self._tasks)
        return spec.scene, spec.name, self._rng.randrange(2**31)


def build_selected_scene(
    scene: str | None, task: str | None, seed: int,
    table_near_edge_m: float | None = 0.10,
) -> Any:
    """Accept a task and table edge; None prompts only for procedural scenes."""
    from spd_envs.registry import get_task

    if task is not None and "/" in task:
        task_scene, task = task.split("/", 1)
        if scene is not None and scene != task_scene:
            raise ValueError("--scene conflicts with the qualified --task")
        scene = task_scene
    if scene is None or scene == "hardware_free":
        if task not in (None, "external_joint_command"):
            raise ValueError("use --task SCENE/TASK or supply --scene with a task name")
        return None
    try:
        spec = get_task(scene, task)
    except KeyError as exc:
        raise ValueError(str(exc)) from exc
    distance = resolve_table_distance(table_near_edge_m)
    return spec.build(seed).with_table_near_edge(distance)


def frame_scene(camera: Any, options: Any, table_near_edge_m: float = 0.10) -> None:
    """Show the tabletop and both arms, hiding duplicate robot collision meshes."""
    shift = table_near_edge_m - 0.10
    camera.lookat[:] = (0.40 + shift * 0.5, 0.0, 0.85)
    camera.distance = 2.0 + abs(shift)
    camera.azimuth = 135.0
    camera.elevation = -30.0
    options.geomgroup[0] = 0
    options.geomgroup[3] = 0  # Scene collision proxies stay physical, not visible.


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--task", required=True, help="Qualified task ID, e.g. dishes/rack_dishes")
    parser.add_argument("--scene", help="Scene name when --task is not qualified")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--table-distance", type=float,
                        help="Base origin to near table edge along +X, metres; prompts when omitted")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--duration", type=float, help="Simulation seconds; required in headless mode")
    parser.add_argument("--output", type=Path, help="Save SCENE/TASK/seed_N/{scene.xml,scene_manifest.json,final.png,state.json}")
    parser.add_argument("--screenshot", type=Path, help="Save the final view here (overrides output's final.png)")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=960)
    args = parser.parse_args(argv)
    if args.duration is not None and (not math.isfinite(args.duration) or args.duration < 0):
        parser.error("--duration must be finite and nonnegative")
    if args.headless and args.duration is None:
        parser.error("--headless requires a finite --duration")
    if args.width <= 0 or args.height <= 0:
        parser.error("image dimensions must be positive")
    # MuJoCo chooses its offscreen backend at import time. Respect explicit overrides.
    if args.headless:
        os.environ.setdefault("MUJOCO_GL", "egl")
    try:
        result = build_selected_scene(args.scene, args.task, args.seed, args.table_distance)
    except ValueError as exc:
        parser.error(str(exc))
    except (EOFError, KeyboardInterrupt):
        print("\n已取消，未打开场景。", file=sys.stderr)
        return 130
    if result is None:
        parser.error("spd-scene requires a procedural task")
    print(f"桌沿 X={result.table_near_edge_m:g} m；桌面与物体已同步定位，进入后固定。", flush=True)

    import mujoco
    import numpy as np
    from PIL import Image

    from simulation.viewer import PlantController
    from simulation.viewer_window import ViewerWindow
    from spd_envs.registry import get_task

    plant = PlantController(
        strict_artifacts=True,
        scene_result=result, scene_output_dir=args.output,
    )
    window = ViewerWindow(plant.model, plant.data, headless=args.headless)
    spec = get_task(result.scene, result.task)
    window.set_task(spec.title_zh, spec.goal_zh)
    summary: dict[str, Any] = {}
    try:
        window.open()
        if window.window is not None:
            with window.window.lock():
                frame_scene(window.window.cam, window.window.opt, result.table_near_edge_m)
        print(f"任务：{spec.title_zh}；目标：{spec.goal_zh}", flush=True)
        timestep = float(plant.model.opt.timestep)
        steps = None if args.duration is None else math.ceil(args.duration / timestep)
        start = time.monotonic()
        render_every = max(1, round(plant.physics_hz / plant.render_hz))
        while window.is_running() and (steps is None or plant.tick < steps):
            plant.physics_tick()
            if not np.isfinite(plant.data.qpos).all() or not np.isfinite(plant.data.qvel).all():
                raise RuntimeError("scene physics produced non-finite state")
            if not args.headless:
                if plant.tick % render_every == 0:
                    window.sync()
                time.sleep(max(0.0, start + plant.tick * timestep - time.monotonic()))
        mujoco.mj_forward(plant.model, plant.data)
        destination = plant.scene_model_path.parent
        screenshot = args.screenshot
        if screenshot is None and args.output is not None:
            screenshot = destination / "final.png"
        if screenshot is not None:
            plant.model.vis.global_.offwidth = max(int(plant.model.vis.global_.offwidth), args.width)
            plant.model.vis.global_.offheight = max(int(plant.model.vis.global_.offheight), args.height)
            camera, options = mujoco.MjvCamera(), mujoco.MjvOption()
            mujoco.mjv_defaultCamera(camera)
            frame_scene(camera, options, result.table_near_edge_m)
            with mujoco.Renderer(plant.model, height=args.height, width=args.width) as renderer:
                renderer.update_scene(plant.data, camera=camera, scene_option=options)
                screenshot = screenshot.resolve()
                screenshot.parent.mkdir(parents=True, exist_ok=True)
                Image.fromarray(renderer.render()).save(screenshot)
        summary = {
            "scene": result.scene, "task": result.task, "seed": result.seed,
            "steps": plant.tick, "sim_time_s": float(plant.data.time),
            "robot_joints": len(plant.joints), "nq": plant.model.nq, "nv": plant.model.nv,
            "artifact_hash": plant.artifact_hash,
            "model": str(plant.scene_model_path) if args.output is not None else None,
            "manifest": str(plant.scene_manifest_path) if args.output is not None else None,
            "screenshot": str(screenshot) if screenshot is not None else None,
            "object_positions": {
                obj.name: plant.data.body(obj.name).xpos.tolist() for obj in result.objects
            },
        }
        if args.output is not None:
            state_path = destination / "state.json"
            summary["state"] = str(state_path)
            state_path.write_text(json.dumps({
                **summary, "qpos": plant.data.qpos.tolist(), "qvel": plant.data.qvel.tolist(),
            }, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    finally:
        window.close()
        plant.close()
    print(json.dumps(summary, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
