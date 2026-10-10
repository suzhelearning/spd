"""Physics-thread friction drafts and atomic runtime/recording-policy publication."""
from __future__ import annotations

import math
from decimal import Decimal
from pathlib import Path
import mujoco
import numpy as np

from _spd_native import material_forward
from data_collector.trajectory import TrajectorySource
from spd_envs.physical_materials import (
    MATERIAL_IDS, PAIR_KEYS, material_manifest, model_material_pairs,
    material_numeric_values, save_task_material_coefficients,
    task_material_config_path, validate_material_coefficients,
)


_MATERIAL_NAMES = {
    "wood": "木材", "polyethylene": "聚乙烯", "ceramic_glaze": "釉面陶瓷",
    "unglazed_ceramic": "无釉陶瓷", "bare_iron": "裸铁",
    "polyester_woven_fabric": "涤纶织物", "silicone": "硅胶",
}


class MaterialEditor:
    """Call key/reset_scene only on the model's owning physics thread.

    Drafts never mutate the model. Apply first prepares native contacts and an
    owned MJB snapshot, then persists the config; publication cannot fail after
    the atomic file replacement. Existing TrajectorySource snapshots stay owned.
    """

    def __init__(self, app):
        self.app = app
        self.opened = False
        self._selected = 0
        self._draft = {}
        self._error = ""
        self._notice = ""
        self._policy_error = ""
        self.reset_scene()

    def _numeric(self):
        model = self.app.plant.model
        index = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_NUMERIC, "spd_material_friction")
        if index < 0:
            raise ValueError("此模型没有材质策略；不能编辑／应用摩擦")
        start, size = int(model.numeric_adr[index]), int(model.numeric_size[index])
        if size != 65:
            raise ValueError("模型材质策略格式不受支持；不能应用")
        # Keep this view and its underlying address: native Physics caches it.
        return model.numeric_data[start:start + size]

    def _current(self):
        values = self._numeric()
        if values[0] != 2:
            raise ValueError("模型材质策略版本不受支持；不能应用")
        coefficients = validate_material_coefficients({
            pair: float(values[1 + 8 * MATERIAL_IDS[pair[0]] + MATERIAL_IDS[pair[1]]])
            for pair in PAIR_KEYS
        })
        if not np.array_equal(values, material_numeric_values(coefficients)):
            raise ValueError("模型材质矩阵非对称或包含未批准材质对；不能应用")
        return coefficients

    def reset_scene(self) -> None:
        """Discard the draft, including after a parent-owned scene replacement."""
        self.opened = False
        self._selected = 0
        self._error = self._notice = self._policy_error = ""
        self._draft = {}
        self._pairs = model_material_pairs(self.app.plant.model)
        self._scene = self._task = ""
        self._profile_path = None
        self._profile_error = ""
        try:
            scene = self.app.plant.scene_manifest or {}
            task = self.app.collection.task_manifest or {}
            profile = (scene.get("physical_materials") or {}).get("task_profile")
            if profile is None:
                profile = (task.get("physical_materials") or {}).get("task_profile") or {}
            self._scene = profile.get("scene", scene.get("scene", task.get("scene", "")))
            self._task = profile.get("task", scene.get("task", task.get("task", "")))
            self._profile_path = (
                Path(profile["path"]) if profile.get("path")
                else task_material_config_path(self._scene, self._task)
            )
        except (OSError, TypeError, ValueError) as exc:
            self._profile_error = f"任务配置身份不可用：{exc}"
        try:
            self._draft = self._current()
        except ValueError as exc:
            self._policy_error = str(exc)

    def _readonly_reason(self) -> str:
        if self._policy_error:
            return self._policy_error
        if self._profile_error:
            return self._profile_error
        if not self._pairs:
            return "此模型没有可调材质对；不能应用"
        collection = self.app.collection
        if (self.app.three_key.stage != "idle" or collection.state != "idle"
                or collection._closed or collection._job is not None
                or collection.recorder.is_busy):
            return "只读：仅待开始 idle 且无活动 episode 时可修改／应用"
        return ""

    def key(self, key: str) -> bool:
        key = key.lower()
        if key == "m":
            if self.opened:
                self.reset_scene()
            else:
                self.reset_scene()
                self.opened = True
            return True
        if not self.opened:
            return False
        if key in {"r", "s", "d"}:
            return True  # Never defer episode controls until after panel close.
        if key == "up":
            if self._pairs:
                self._selected = (self._selected - 1) % len(self._pairs)
            return True
        if key == "down":
            if self._pairs:
                self._selected = (self._selected + 1) % len(self._pairs)
            return True
        if key not in {"left", "right", "enter"}:
            return False
        reason = self._readonly_reason()
        if reason:
            self._error = reason
            return True
        if key == "enter":
            self._apply()
        else:
            pair = self._pairs[self._selected]
            delta = Decimal("0.05") if key == "right" else Decimal("-0.05")
            value = float(max(Decimal(0), Decimal(str(self._draft[pair])) + delta))
            if not math.isfinite(value):
                self._error = "系数必须为有限非负数"
            else:
                self._draft[pair] = value
                self._error = self._notice = ""
        return True

    def _apply(self) -> None:
        plant, collection = self.app.plant, self.app.collection
        old_scene = plant.scene_manifest
        old_manifest, old_source = collection.task_manifest, collection.source
        numeric = None
        old_numeric = old_data = None
        try:
            coefficients = validate_material_coefficients(self._draft)
            # Reject externally replaced/malformed models before touching them.
            self._current()
            numeric = self._numeric()
            old_numeric = numeric.copy()
            old_data = mujoco.MjData(plant.model)
            mujoco.mj_copyData(old_data, plant.model, plant.data)
            numeric[:] = material_numeric_values(coefficients)
            material_forward(plant.model, plant.data)  # No stepping or resampling.
            policy = {
                **material_manifest(coefficients),
                "task_profile": {
                    "scene": self._scene, "task": self._task, "path": str(self._profile_path),
                },
            }
            plant.scene_manifest = {**(old_scene or {}), "physical_materials": policy}
            manifest = {
                **self.app._task_manifest(
                    plant, self._scene, self._task,
                    (old_scene or {}).get("seed", (old_manifest or {}).get("seed")),
                ),
                "collection_config": collection.config.as_dict(),
            }
            source = TrajectorySource(plant, manifest)
            # This is the last fallible operation. No episode can start mid-key.
            save_task_material_coefficients(
                self._scene, self._task, coefficients, path=self._profile_path,
            )
        except Exception as exc:
            if old_numeric is not None:
                numeric[:] = old_numeric
            if old_data is not None:
                mujoco.mj_copyData(plant.data, plant.model, old_data)
            plant.scene_manifest = old_scene
            collection.task_manifest, collection.source = old_manifest, old_source
            self._error = f"应用失败，未改变生效策略／录制快照：{exc}"
            self._notice = ""
            return
        collection.task_manifest, collection.source = manifest, source
        self._draft = coefficients.copy()
        self._error = ""
        self._notice = "已应用并保存；当前接触与下一条录制快照已同步"

    def hud(self) -> dict[str, str]:
        if not self.opened:
            return {"摩擦": "M 编辑材质摩擦（仅待开始时可应用）"}
        path_label = "不可用"
        if self._profile_path is not None:
            try:
                path_label = str(self._profile_path.relative_to(Path.cwd()))
            except ValueError:
                path_label = str(self._profile_path)
        lines = {
            "摩擦编辑": f"{self._selected + 1 if self._pairs else 0}/{len(self._pairs)} 可调　当前 → 待应用",
            "当前任务": f"{self._scene}/{self._task}",
            "保存文件": path_label,
            "操作": "↑↓ 选择　←→ ±0.05（最小0，无上限）　Enter 应用保存　M 取消关闭",
            "采集快捷键": "面板打开时 r / s / d 已隔离；关闭后需重新按键",
        }
        try:
            current = self._current()
            self._policy_error = ""
        except ValueError as exc:
            current = {}
            self._policy_error = str(exc)
        reason = self._readonly_reason()
        lines["权限"] = reason or "可编辑：仅应用时修改物理；取消不会写入配置"
        if self._error:
            lines["失败"] = self._error
        elif self._notice:
            lines["结果"] = self._notice
        if self._pairs:
            pair = self._pairs[self._selected]
            name = "／".join(_MATERIAL_NAMES[material] for material in pair)
            active = str(current[pair]) if pair in current else "未生效"
            draft = str(self._draft[pair]) if pair in self._draft else "不可用"
            lines[f"▶ {self._selected + 1:02d} {name}"] = f"{active} → {draft}"
        return lines
