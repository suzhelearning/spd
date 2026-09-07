"""Replaceable policy-camera providers for SPD VR episodes."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any, Mapping

import numpy as np
import yaml

CAMERA_NAMES = ("top", "left_wrist", "right_wrist")
IMAGE_HEIGHT = 168
IMAGE_WIDTH = 224
CALIBRATION_REVISION = "provisional-v1"


class CameraError(RuntimeError):
    """Raised when camera configuration or render output is unsafe to record."""


@dataclass(frozen=True)
class CameraConfig:
    name: str
    parent: str
    position: tuple[float, float, float]
    look_at: tuple[float, float, float]


@dataclass(frozen=True)
class CameraFrame:
    name: str
    timestamp_ns: int
    rgb: np.ndarray
    segmentation: np.ndarray
    calibration_revision: str


def _tuple3(value: Any, name: str) -> tuple[float, float, float]:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise CameraError(f"{name} must be a length-3 vector")
    result = tuple(float(item) for item in value)
    if not all(math.isfinite(item) for item in result):
        raise CameraError(f"{name} must contain finite values")
    return result  # type: ignore[return-value]


def load_camera_config(path: str | Path) -> tuple[dict[str, Any], dict[str, CameraConfig]]:
    path = Path(path)
    if not path.is_file():
        raise FileNotFoundError(path)
    document = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if not isinstance(document, dict) or document.get("version") != 1:
        raise CameraError("camera config version must be 1")
    revision = document.get("calibration_revision")
    if not isinstance(revision, str) or not revision:
        raise CameraError("calibration_revision must be a non-empty string")
    width = int(document.get("width", 0))
    height = int(document.get("height", 0))
    if (width, height) != (IMAGE_WIDTH, IMAGE_HEIGHT):
        raise CameraError(f"camera resolution must be {IMAGE_WIDTH}x{IMAGE_HEIGHT}")
    fovy = float(document.get("fovy_deg", 0.0))
    near = float(document.get("near_m", 0.0))
    far = float(document.get("far_m", 0.0))
    if not (math.isfinite(fovy) and fovy == 70.0):
        raise CameraError("camera fovy_deg must be 70.0")
    if not (math.isfinite(near) and near == 0.01):
        raise CameraError("camera near_m must be 0.01")
    if not (math.isfinite(far) and far == 3.0):
        raise CameraError("camera far_m must be 3.0")
    entries = document.get("cameras")
    if not isinstance(entries, dict) or set(entries) != set(CAMERA_NAMES):
        raise CameraError(f"camera names must be exactly {CAMERA_NAMES}")
    configs: dict[str, CameraConfig] = {}
    for name in CAMERA_NAMES:
        entry = entries[name]
        if not isinstance(entry, dict):
            raise CameraError(f"camera {name} must be a mapping")
        parent = entry.get("parent")
        if not isinstance(parent, str) or not parent:
            raise CameraError(f"camera {name} parent must be a non-empty string")
        configs[name] = CameraConfig(
            name=name,
            parent=parent,
            position=_tuple3(entry.get("position"), f"{name}.position"),
            look_at=_tuple3(entry.get("look_at"), f"{name}.look_at"),
        )
    return document, configs


def _normalize(vector: np.ndarray, name: str) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    if not math.isfinite(norm) or norm <= 1e-12:
        raise CameraError(f"{name} must have non-zero length")
    return vector / norm


def look_at_rotation(position: tuple[float, float, float], look_at: tuple[float, float, float]) -> np.ndarray:
    """Return camera-local-to-parent rotation; MuJoCo camera looks down -Z."""
    position_array = np.asarray(position, dtype=np.float64)
    forward = _normalize(np.asarray(look_at, dtype=np.float64) - position_array, "look_at-position")
    world_up = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    if abs(float(np.dot(forward, world_up))) > 0.98:
        world_up = np.array([0.0, 1.0, 0.0], dtype=np.float64)
    right = _normalize(np.cross(forward, world_up), "camera right")
    up = _normalize(np.cross(right, forward), "camera up")
    # Columns are local x/right, local y/up, local z/backward.
    return np.column_stack((right, up, -forward))


def rotation_to_mujoco_quat(rotation: np.ndarray) -> tuple[float, float, float, float]:
    """Convert a proper rotation matrix to MuJoCo's wxyz quaternion."""
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        x = (rotation[2, 1] - rotation[1, 2]) / scale
        y = (rotation[0, 2] - rotation[2, 0]) / scale
        z = (rotation[1, 0] - rotation[0, 1]) / scale
        w = 0.25 * scale
    else:
        diagonal = np.diag(rotation)
        index = int(np.argmax(diagonal))
        next_index = (index + 1) % 3
        last_index = (index + 2) % 3
        scale = math.sqrt(max(1e-15, 1.0 + rotation[index, index] - rotation[next_index, next_index] - rotation[last_index, last_index])) * 2.0
        values = [0.0, 0.0, 0.0]
        values[index] = 0.25 * scale
        w = (rotation[last_index, next_index] - rotation[next_index, last_index]) / scale
        values[next_index] = (rotation[next_index, index] + rotation[index, next_index]) / scale
        values[last_index] = (rotation[last_index, index] + rotation[index, last_index]) / scale
        x, y, z = values
    result = np.asarray((w, x, y, z), dtype=np.float64)
    result /= np.linalg.norm(result)
    return tuple(float(item) for item in result)  # type: ignore[return-value]


def validate_frame(frame: CameraFrame, expected_timestamp_ns: int | None = None) -> None:
    if frame.name not in CAMERA_NAMES:
        raise CameraError(f"unknown logical camera name: {frame.name}")
    if expected_timestamp_ns is not None and frame.timestamp_ns != expected_timestamp_ns:
        raise CameraError(
            f"camera {frame.name} timestamp {frame.timestamp_ns} != {expected_timestamp_ns}"
        )
    if frame.rgb.dtype != np.uint8 or frame.rgb.shape != (IMAGE_HEIGHT, IMAGE_WIDTH, 3):
        raise CameraError(f"camera {frame.name} RGB must be uint8[168,224,3]")
    if frame.segmentation.dtype != np.int32 or frame.segmentation.shape != (IMAGE_HEIGHT, IMAGE_WIDTH, 2):
        raise CameraError(f"camera {frame.name} segmentation must be int32[168,224,2]")
    if not isinstance(frame.calibration_revision, str) or not frame.calibration_revision:
        raise CameraError(f"camera {frame.name} calibration revision is empty")


def validate_frame_mapping(
    frames: Mapping[str, CameraFrame],
    expected_timestamp_ns: int | None = None,
) -> dict[str, CameraFrame]:
    if set(frames) != set(CAMERA_NAMES):
        raise CameraError(f"camera frame names must be exactly {CAMERA_NAMES}")
    result: dict[str, CameraFrame] = {}
    for name in CAMERA_NAMES:
        frame = frames[name]
        if frame.name != name:
            raise CameraError(f"camera mapping key/name mismatch: {name}/{frame.name}")
        validate_frame(frame, expected_timestamp_ns)
        result[name] = frame
    return result


class SyntheticCameraProvider:
    """Deterministic zero-image provider for headless/replay smoke tests."""

    def __init__(self, calibration_revision: str = CALIBRATION_REVISION) -> None:
        self.calibration_revision = calibration_revision
        self._last_timestamp_ns = -1
    def reset(self) -> None:
        self._last_timestamp_ns = -1

    def capture(self, sim_time_ns: int) -> Mapping[str, CameraFrame]:
        if sim_time_ns < self._last_timestamp_ns:
            raise CameraError("render timestamp moved backwards")
        self._last_timestamp_ns = int(sim_time_ns)
        return validate_frame_mapping({
            name: CameraFrame(
                name=name,
                timestamp_ns=sim_time_ns,
                rgb=np.zeros((IMAGE_HEIGHT, IMAGE_WIDTH, 3), dtype=np.uint8),
                segmentation=np.full((IMAGE_HEIGHT, IMAGE_WIDTH, 2), -1, dtype=np.int32),
                calibration_revision=self.calibration_revision,
            )
            for name in CAMERA_NAMES
        }, sim_time_ns)


class MujocoCameraProvider:
    """Render exactly the three policy views from one MuJoCo model/data pair."""

    def __init__(
        self,
        model: Any,
        data: Any,
        config_path: str | Path,
    ) -> None:
        self.model = model
        self.data = data
        self.document, self.configs = load_camera_config(config_path)
        try:
            import mujoco
        except ImportError as exc:  # pragma: no cover
            raise ImportError("mujoco is required for MujocoCameraProvider") from exc
        self._mujoco = mujoco
        self._renderers: dict[str, Any] = {}
        for name in CAMERA_NAMES:
            camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, name)
            if camera_id < 0:
                raise CameraError(f"model is missing logical camera: {name}")
            try:
                self._renderers[name] = mujoco.Renderer(
                    model, height=IMAGE_HEIGHT, width=IMAGE_WIDTH
                )
            except Exception as exc:
                raise CameraError(f"cannot create renderer for {name}: {exc}") from exc
        self._last_timestamp_ns = -1
        self._worker_data = mujoco.MjData(model)

    def reset(self) -> None:
        self._last_timestamp_ns = -1

    def _capture_with_data(self, sim_time_ns: int, data: Any) -> Mapping[str, CameraFrame]:
        if sim_time_ns < self._last_timestamp_ns:
            raise CameraError("render timestamp moved backwards")
        self._last_timestamp_ns = int(sim_time_ns)
        frames: dict[str, CameraFrame] = {}
        for name in CAMERA_NAMES:
            renderer = self._renderers[name]
            renderer.update_scene(data, camera=name)
            rgb = np.asarray(renderer.render()).copy()
            renderer.enable_segmentation_rendering()
            try:
                segmentation = np.asarray(renderer.render()).copy()
            finally:
                renderer.disable_segmentation_rendering()
            frames[name] = CameraFrame(
                name=name,
                timestamp_ns=sim_time_ns,
                rgb=rgb,
                segmentation=segmentation,
                calibration_revision=self.document["calibration_revision"],
            )
        return validate_frame_mapping(frames, sim_time_ns)

    def capture(self, sim_time_ns: int) -> Mapping[str, CameraFrame]:
        return self._capture_with_data(sim_time_ns, self.data)

    def capture_snapshot(
        self, sim_time_ns: int, qpos: np.ndarray, qvel: np.ndarray
    ) -> Mapping[str, CameraFrame]:
        qpos = np.asarray(qpos, dtype=np.float64)
        qvel = np.asarray(qvel, dtype=np.float64)
        if qpos.shape != (self.model.nq,) or qvel.shape != (self.model.nv,):
            raise CameraError("camera snapshot qpos/qvel shape does not match model")
        self._worker_data.qpos[:] = qpos
        self._worker_data.qvel[:] = qvel
        self._mujoco.mj_forward(self.model, self._worker_data)
        return self._capture_with_data(sim_time_ns, self._worker_data)


__all__ = [
    "CALIBRATION_REVISION",
    "CAMERA_NAMES",
    "CameraConfig",
    "CameraError",
    "CameraFrame",
    "IMAGE_HEIGHT",
    "IMAGE_WIDTH",
    "MujocoCameraProvider",
    "SyntheticCameraProvider",
    "load_camera_config",
    "look_at_rotation",
    "rotation_to_mujoco_quat",
    "validate_frame",
    "validate_frame_mapping",
]
