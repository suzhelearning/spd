"""Replaceable policy-camera providers for SPD VR episodes."""

from __future__ import annotations

from dataclasses import dataclass
import math
from numbers import Real
from pathlib import Path
from typing import Any, Mapping

import numpy as np
import yaml

CAMERA_NAMES = ("top", "left_wrist", "right_wrist")
IMAGE_HEIGHT = 720
IMAGE_WIDTH = 1280
CALIBRATION_REVISION = "provisional-v2"


class CameraError(RuntimeError):
    """Raised when camera configuration or render output is unsafe to record."""


@dataclass(frozen=True)
class CameraConfig:
    name: str
    parent: str
    position: tuple[float, float, float]
    quaternion: tuple[float, float, float, float]


@dataclass(frozen=True)
class CameraFrame:
    name: str
    timestamp_ns: int
    rgb: np.ndarray
    calibration_revision: str


def _tuple3(value: Any, name: str) -> tuple[float, float, float]:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise CameraError(f"{name} must be a length-3 vector")
    if any(isinstance(item, bool) or not isinstance(item, Real) for item in value):
        raise CameraError(f"{name} must contain numeric values")
    result = tuple(float(item) for item in value)
    if not all(math.isfinite(item) for item in result):
        raise CameraError(f"{name} must contain finite values")
    return result  # type: ignore[return-value]


def _finite_float(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise CameraError(f"{name} must be a finite number")
    result = float(value)
    if not math.isfinite(result):
        raise CameraError(f"{name} must be a finite number")
    return result


def _rpy_deg_to_mujoco_quat(
    rpy_deg: tuple[float, float, float],
) -> tuple[float, float, float, float]:
    """Return the wxyz camera-to-parent quaternion for Rz(yaw) Ry(pitch) Rx(roll)."""
    roll, pitch, yaw = (math.radians(value) for value in rpy_deg)
    half_roll, half_pitch, half_yaw = roll / 2.0, pitch / 2.0, yaw / 2.0
    cr, sr = math.cos(half_roll), math.sin(half_roll)
    cp, sp = math.cos(half_pitch), math.sin(half_pitch)
    cy, sy = math.cos(half_yaw), math.sin(half_yaw)
    quaternion = (
        cy * cp * cr + sy * sp * sr,
        cy * cp * sr - sy * sp * cr,
        cy * sp * cr + sy * cp * sr,
        sy * cp * cr - cy * sp * sr,
    )
    norm = math.sqrt(sum(component * component for component in quaternion))
    return tuple(component / norm for component in quaternion)  # type: ignore[return-value]


def _parse_camera_document(
    document: Any, *, allow_legacy: bool
) -> tuple[dict[str, CameraConfig], float]:
    if not isinstance(document, dict):
        raise CameraError("camera config must be a mapping")
    version = document.get("version")
    legacy = type(version) is int and version == 1
    current = type(version) is int and version == 2
    if not current and not (allow_legacy and legacy):
        expected = "1 or 2" if allow_legacy else "2"
        raise CameraError(f"camera config version must be {expected}")
    revision = document.get("calibration_revision")
    if not isinstance(revision, str) or not revision:
        raise CameraError("calibration_revision must be a non-empty string")
    width = _finite_float(document.get("width"), "camera width")
    height = _finite_float(document.get("height"), "camera height")
    if (width, height) != (IMAGE_WIDTH, IMAGE_HEIGHT):
        raise CameraError(f"camera resolution must be {IMAGE_WIDTH}x{IMAGE_HEIGHT}")
    fovy = _finite_float(document.get("fovy_deg"), "camera fovy_deg")
    near = _finite_float(document.get("near_m"), "camera near_m")
    far = _finite_float(document.get("far_m"), "camera far_m")
    if fovy != 70.0:
        raise CameraError("camera fovy_deg must be 70.0")
    if near != 0.01:
        raise CameraError("camera near_m must be 0.01")
    if far != 3.0:
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
        if current and parent == "world":
            raise CameraError(f"camera {name} parent must be a non-world body")
        position = _tuple3(entry.get("position"), f"{name}.position")
        if legacy:
            quaternion = rotation_to_mujoco_quat(
                look_at_rotation(position, _tuple3(entry.get("look_at"), f"{name}.look_at"))
            )
        else:
            if "look_at" in entry:
                raise CameraError(f"camera {name} must use rpy_deg instead of look_at")
            quaternion = _rpy_deg_to_mujoco_quat(
                _tuple3(entry.get("rpy_deg"), f"{name}.rpy_deg")
            )
        configs[name] = CameraConfig(
            name=name,
            parent=parent,
            position=position,
            quaternion=quaternion,
        )
    return configs, fovy


def parse_camera_config(
    document: Any, *, allow_legacy: bool = False
) -> dict[str, CameraConfig]:
    """Validate camera metadata and return fixed local camera mounts."""
    return _parse_camera_document(document, allow_legacy=allow_legacy)[0]


def load_camera_config(path: str | Path) -> tuple[dict[str, Any], dict[str, CameraConfig]]:
    path = Path(path)
    if not path.is_file():
        raise FileNotFoundError(path)
    document = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    configs = parse_camera_config(document)
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
        raise CameraError(f"camera {frame.name} RGB must be uint8[{IMAGE_HEIGHT},{IMAGE_WIDTH},3]")
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


def apply_camera_config(model: Any, document: Any) -> None:
    """Apply a complete version-2 camera mount document without changing dynamics."""
    import mujoco

    configs, fovy = _parse_camera_document(document, allow_legacy=False)
    try:
        fixed_mode = int(mujoco.mjtCamLight.mjCAMLIGHT_FIXED)
    except AttributeError as exc:  # pragma: no cover - fixed in supported MuJoCo versions
        raise CameraError("MuJoCo does not expose fixed camera mode") from exc
    updates: list[tuple[int, int, CameraConfig]] = []
    try:
        for name in CAMERA_NAMES:
            config = configs[name]
            camera_id = int(
                mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, name)
            )
            if camera_id < 0:
                raise CameraError(f"model is missing logical camera: {name}")
            body_id = int(
                mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, config.parent)
            )
            if body_id <= 0:
                raise CameraError(f"camera {name} parent is missing: {config.parent}")
            if (np.shape(model.cam_pos[camera_id]) != (3,)
                    or np.shape(model.cam_quat[camera_id]) != (4,)):
                raise CameraError(f"model camera arrays are invalid: {name}")
            _ = model.cam_mode[camera_id]
            _ = model.cam_targetbodyid[camera_id]
            _ = model.cam_bodyid[camera_id]
            _ = model.cam_fovy[camera_id]
            updates.append((camera_id, body_id, config))
    except CameraError:
        raise
    except (AttributeError, IndexError, TypeError, ValueError) as exc:
        raise CameraError(f"model camera arrays are invalid: {exc}") from exc

    for camera_id, body_id, config in updates:
        model.cam_mode[camera_id] = fixed_mode
        model.cam_targetbodyid[camera_id] = -1
        model.cam_bodyid[camera_id] = body_id
        model.cam_pos[camera_id] = config.position
        model.cam_quat[camera_id] = config.quaternion
        model.cam_fovy[camera_id] = fovy


def load_camera_model(model_path: str | Path, config_path: str | Path) -> Any:
    """Add configured, body-attached RGB views without changing robot dynamics."""
    import mujoco

    document, configs = load_camera_config(config_path)
    spec = mujoco.MjSpec.from_file(str(model_path))
    mounts: list[tuple[str, CameraConfig, Any]] = []
    for name in CAMERA_NAMES:
        config = configs[name]
        if spec.camera(name) is not None:
            raise CameraError(f"model already defines camera {name}; refusing ambiguous configuration")
        parent = spec.body(config.parent)
        if parent is None:
            raise CameraError(f"camera {name} parent is missing: {config.parent}")
        mounts.append((name, config, parent))
    for name, config, parent in mounts:
        parent.add_camera(
            name=name,
            pos=config.position,
            quat=config.quaternion,
            fovy=float(document["fovy_deg"]),
        )
    model = spec.compile()
    model.vis.global_.offwidth = max(int(model.vis.global_.offwidth), IMAGE_WIDTH)
    model.vis.global_.offheight = max(int(model.vis.global_.offheight), IMAGE_HEIGHT)
    return model


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
        self._scene_option = mujoco.MjvOption()
        self._scene_option.geomgroup[0] = 0
        self._scene_option.geomgroup[3] = 0
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

    def reset(self) -> None:
        self._last_timestamp_ns = -1

    def _capture_with_data(self, sim_time_ns: int, data: Any) -> Mapping[str, CameraFrame]:
        if sim_time_ns < self._last_timestamp_ns:
            raise CameraError("render timestamp moved backwards")
        self._last_timestamp_ns = int(sim_time_ns)
        frames: dict[str, CameraFrame] = {}
        for name in CAMERA_NAMES:
            renderer = self._renderers[name]
            renderer.update_scene(data, camera=name, scene_option=self._scene_option)
            rgb = np.asarray(renderer.render()).copy()
            frames[name] = CameraFrame(
                name=name,
                timestamp_ns=sim_time_ns,
                rgb=rgb,
                calibration_revision=self.document["calibration_revision"],
            )
        return validate_frame_mapping(frames, sim_time_ns)

    def capture(self, sim_time_ns: int) -> Mapping[str, CameraFrame]:
        return self._capture_with_data(sim_time_ns, self.data)



__all__ = [
    "CALIBRATION_REVISION",
    "CAMERA_NAMES",
    "CameraConfig",
    "CameraError",
    "CameraFrame",
    "apply_camera_config",
    "IMAGE_HEIGHT",
    "IMAGE_WIDTH",
    "MujocoCameraProvider",
    "load_camera_config",
    "parse_camera_config",
    "look_at_rotation",
    "rotation_to_mujoco_quat",
    "validate_frame",
    "validate_frame_mapping",
]
