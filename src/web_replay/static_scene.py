"""Load one embedded scene model, without indexing or reading trajectory frames."""
from __future__ import annotations

from copy import deepcopy
from pathlib import Path
from typing import Any

import h5py
import mujoco

from data_collector.trajectory import TrajectoryError, load_model

from .dataset import (
    ReplayError,
    _camera_mounts,
    _episode_labels,
    _load_editor_camera_config,
    _model_metadata,
    _source_identity,
    _text,
)
from .scene import SceneError, export_scene


class StaticScene:
    """An immutable model/GLB snapshot at compiled qpos0; no frame API."""

    def __init__(self, path: Path) -> None:
        self.path = Path(path).expanduser().resolve(strict=True)
        self._identity = _source_identity(self.path)
        try:
            with h5py.File(self.path, "r") as handle:
                metadata = _model_metadata(handle)
                model_bytes = handle["model/mjb"][:].tobytes()
                task, scene, title = _episode_labels(metadata, _text(handle.attrs["task"], "task"))
            self._assert_unchanged()
            model = load_model(model_bytes, metadata, expected_model_sha256=metadata["model_sha256"])
            camera_config, configs = _load_editor_camera_config()
            mounts, parent_ids = _camera_mounts(model, configs)
            data = mujoco.MjData(model)
            # MjData starts from compiled qpos0. Never read trajectory/qpos or step.
            mujoco.mj_kinematics(model, data)
            glb, body_ids, center = export_scene(model, data, metadata, extra_body_ids=parent_ids)
            self._info = {
                "id": "static-scene",
                "mode": "scene",
                "title": title,
                "task": task,
                "scene": scene,
                "source_path": str(self.path),
                "pose_source": "compiled_qpos0",
                "body_ids": body_ids,
                "body_names": [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)
                               or f"body_{body_id}" for body_id in body_ids],
                "center": center,
                "camera_config": camera_config,
                "camera_mounts": mounts,
                "model_sha256": metadata["model_sha256"],
            }
            self._glb = glb
            self._assert_unchanged()
        except ReplayError:
            raise
        except (TrajectoryError, SceneError, OSError, RuntimeError, TypeError, ValueError, KeyError) as exc:
            raise ReplayError(f"无法加载静态场景模型: {exc}") from exc

    def _assert_unchanged(self) -> None:
        if _source_identity(self.path) != self._identity:
            raise ReplayError("场景模型文件已改变，请重启服务")

    @property
    def info(self) -> dict[str, Any]:
        self._assert_unchanged()
        return deepcopy(self._info)

    @property
    def scene_glb(self) -> bytes:
        self._assert_unchanged()
        return self._glb
