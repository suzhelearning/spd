"""SPD's 17 Table 2 tasks plus the Figure 4 / A.4 Jenga playing task."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Iterable

from .scene_builder import ProceduralSceneBuilder, SceneBuildResult


@dataclass(frozen=True)
class TaskSpec:
    name: str
    prompt: str
    title_zh: str
    goal_zh: str
    target_duration_s: float | None
    build: Callable[[int], SceneBuildResult]
    reset: Callable[[int], SceneBuildResult]
    score_debug: Callable[[SceneBuildResult], dict[str, float | int | str]]
    scene: str
    table2_episodes: int | None
    table2_minutes: int | None

    @property
    def qualified_name(self) -> str:
        return f"{self.scene}/{self.name}"


# Source of truth: SPD paper Table 2.  The per-episode target is exactly
# 60 seconds/minute × total minutes / episode count.
TABLE2_STATS = {
    ("jenga", "hollow_tower"): (92, 567),
    ("jenga", "tower"): (87, 473),
    ("jenga", "dominos"): (107, 471),
    ("jenga", "criss_cross"): (103, 386),
    ("jenga", "handover_lr"): (96, 172),
    ("jenga", "handover_rl"): (72, 109),
    ("spelling_blocks", "spelling"): (168, 587),
    ("spelling_blocks", "sort_and_unload"): (25, 144),
    ("spelling_blocks", "pyramid"): (32, 136),
    ("spelling_blocks", "vowel_consonant_sort"): (50, 109),
    ("mugs", "hang_mug"): (406, 491),
    ("dishes", "rack_dishes"): (129, 285),
    ("dishes", "plate_dishes"): (79, 109),
    ("cups", "pyramid"): (44, 109),
    ("cups", "stack_two_threes"): (46, 67),
    ("cups", "unstack"): (30, 47),
    ("bottles", "toss_in_bin"): (350, 253),
}
PROMPTS = {
    ("jenga", "hollow_tower"): "Build a hollow Jenga tower.",
    ("jenga", "tower"): "Build a stable Jenga tower.",
    ("jenga", "dominos"): "Arrange the blocks as a domino chain.",
    ("jenga", "criss_cross"): "Build a criss-cross Jenga tower.",
    ("jenga", "handover_lr"): "Hand the block from the left hand to the right.",
    ("jenga", "handover_rl"): "Hand the block from the right hand to the left.",
    ("jenga", "playing"): "Push a middle block out, pull it free without collapsing the tower, and place it on top.",
    ("spelling_blocks", "spelling"): "Spell the sampled target word with the letter blocks.",
    ("spelling_blocks", "sort_and_unload"): "Open the drawers, sort the letter blocks, and unload them onto the table.",
    ("spelling_blocks", "pyramid"): "Build a pyramid from the letter blocks.",
    ("spelling_blocks", "vowel_consonant_sort"): "Sort letters into vowels and consonants.",
    ("mugs", "hang_mug"): "Hang the mug on the mug tree.",
    ("dishes", "rack_dishes"): "Place the plates into the dish rack.",
    ("dishes", "plate_dishes"): "Arrange the plates into a dish stack.",
    ("cups", "pyramid"): "Build a pyramid from the cups.",
    ("cups", "stack_two_threes"): "Build two stacks of three cups.",
    ("cups", "unstack"): "Unstack the nested cups.",
    ("bottles", "toss_in_bin"): "Toss the bottle into the bin.",
}
TASK_TEXT_ZH = {
    ("jenga", "hollow_tower"): ("空心积木塔", "用积木搭建一座空心塔。"),
    ("jenga", "tower"): ("搭建积木塔", "用积木搭建一座稳固的塔。"),
    ("jenga", "dominos"): ("多米诺骨牌", "将积木排列成一条多米诺骨牌链。"),
    ("jenga", "criss_cross"): ("交错积木塔", "将积木交错堆叠成塔。"),
    ("jenga", "handover_lr"): ("左手交给右手", "将积木从左手传递到右手。"),
    ("jenga", "handover_rl"): ("右手交给左手", "将积木从右手传递到左手。"),
    ("jenga", "playing"): (
        "抽取并叠放积木",
        "先推出塔中间的一块积木，再将其完整抽出，保持塔不倒塌，最后把积木放到塔顶。",
    ),
    ("spelling_blocks", "spelling"): ("字母拼词", "用字母积木拼出本次指定的单词。"),
    ("spelling_blocks", "sort_and_unload"): ("字母积木分拣", "拉开抽屉，将其中的字母积木分类并取出放到桌上。"),
    ("spelling_blocks", "pyramid"): ("字母积木金字塔", "用字母积木搭建金字塔。"),
    ("spelling_blocks", "vowel_consonant_sort"): ("元音辅音分类", "将字母积木按元音和辅音分成两组。"),
    ("mugs", "hang_mug"): ("悬挂马克杯", "将马克杯挂到杯架上。"),
    ("dishes", "rack_dishes"): ("餐盘入架", "将餐盘放入沥水架。"),
    ("dishes", "plate_dishes"): ("叠放餐盘", "将餐盘整齐叠成一摞。"),
    ("cups", "pyramid"): ("杯子金字塔", "用杯子搭建金字塔。"),
    ("cups", "stack_two_threes"): ("两组三杯叠放", "将杯子叠成两组，每组三个。"),
    ("cups", "unstack"): ("拆分套叠杯", "将套叠在一起的杯子逐个分开。"),
    ("bottles", "toss_in_bin"): ("投瓶入箱", "将瓶子投入收纳箱。"),
}


def _build(scene: str, task: str, seed: int) -> SceneBuildResult:
    return ProceduralSceneBuilder(scene, task, seed).build()


def _score(result: SceneBuildResult) -> dict[str, float | int | str]:
    return {
        "scene": result.scene,
        "task": result.task,
        "objects": len(result.objects),
        "candidate": result.candidate,
        "contact_group_count": len({item.contact_group for item in result.objects}),
    }


def _task(scene: str, task: str) -> TaskSpec:
    statistics = TABLE2_STATS.get((scene, task))
    episodes, minutes = statistics if statistics is not None else (None, None)
    duration = 60.0 * float(minutes) / float(episodes) if statistics is not None else None
    builder = lambda seed: _build(scene, task, seed)
    return TaskSpec(
        name=task,
        prompt=PROMPTS[(scene, task)],
        title_zh=TASK_TEXT_ZH[(scene, task)][0],
        goal_zh=TASK_TEXT_ZH[(scene, task)][1],
        target_duration_s=duration,
        build=builder,
        reset=builder,
        score_debug=_score,
        scene=scene,
        table2_episodes=episodes,
        table2_minutes=minutes,
    )


# A.4 evaluates playing, but Table 2 does not report a playing dataset row.
# None denotes unreported statistics/duration, rather than invented counts.
TASKS: tuple[TaskSpec, ...] = tuple(_task(scene, task) for scene, task in TABLE2_STATS) + (_task("jenga", "playing"),)
TASK_REGISTRY = {spec.qualified_name: spec for spec in TASKS}
SCENES = tuple(dict.fromkeys(spec.scene for spec in TASKS))


def get_task(scene: str, task: str | None = None) -> TaskSpec:
    if task is None and "/" in scene:
        scene, task = scene.split("/", 1)
    if task is None:
        matches = [spec for spec in TASKS if spec.scene == scene]
        if len(matches) != 1:
            raise KeyError(f"task is required for scene {scene!r}")
        return matches[0]
    try:
        return TASK_REGISTRY[f"{scene}/{task}"]
    except KeyError as exc:
        raise KeyError(f"unknown SPD task: {scene}/{task}") from exc


def iter_tasks() -> Iterable[TaskSpec]:
    return iter(TASKS)


__all__ = [
    "PROMPTS",
    "SCENES",
    "TABLE2_STATS",
    "TASKS",
    "TASK_REGISTRY",
    "TaskSpec",
    "get_task",
    "iter_tasks",
]
