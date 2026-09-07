"""Independent PICO-wrist neutral alignment for one arm side."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any

import numpy as np


_IDENTITY = np.eye(4, dtype=float)
PICO_TO_ROBOT_ROTATION = np.asarray(
    ((0.0, 0.0, -1.0), (-1.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
    dtype=float,
)


def _pose_matrix(value: Any) -> np.ndarray:
    """Return a validated homogeneous transform from a matrix or pose object."""
    if hasattr(value, "position") and hasattr(value, "quaternion_xyzw"):
        value = (*np.asarray(value.position, dtype=float), *np.asarray(value.quaternion_xyzw, dtype=float))
    array = np.asarray(value, dtype=float)
    if array.shape == (7,):
        position = array[:3]
        quaternion = array[3:]
        norm = float(np.linalg.norm(quaternion))
        if not np.isfinite(norm) or norm <= 1.0e-12:
            raise ValueError("pose quaternion must be finite and non-zero")
        x, y, z, w = quaternion / norm
        rotation = np.array(
            [
                [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
                [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
                [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
            ]
        )
        result = np.eye(4, dtype=float)
        result[:3, :3] = rotation
        result[:3, 3] = position
        array = result
    if array.shape != (4, 4) or not np.all(np.isfinite(array)):
        raise ValueError("pose must be a finite 4x4 transform or 7-vector")
    if not np.allclose(array[3], (0.0, 0.0, 0.0, 1.0), atol=1.0e-8):
        raise ValueError("pose must be homogeneous")
    rotation = array[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1.0e-6) or not math.isclose(float(np.linalg.det(rotation)), 1.0, abs_tol=1.0e-6):
        raise ValueError("pose rotation must be proper")
    return np.array(array, dtype=float, copy=True)


def _rotation_distance(first: np.ndarray, second: np.ndarray) -> float:
    cosine = float(np.clip((np.trace(first.T @ second) - 1.0) * 0.5, -1.0, 1.0))
    return float(math.acos(cosine))


def _rotation_vector(rotation: np.ndarray) -> np.ndarray:
    """Return the shortest axis-angle vector represented by a rotation."""
    cosine = float(np.clip((np.trace(rotation) - 1.0) * 0.5, -1.0, 1.0))
    angle = float(math.acos(cosine))
    skew = np.asarray(
        (
            rotation[2, 1] - rotation[1, 2],
            rotation[0, 2] - rotation[2, 0],
            rotation[1, 0] - rotation[0, 1],
        ),
        dtype=float,
    )
    if angle < 1.0e-8:
        return 0.5 * skew
    sine = math.sin(angle)
    if abs(sine) < 1.0e-7:
        diagonal = np.maximum(np.diag(rotation) + 1.0, 0.0)
        axis = np.sqrt(diagonal * 0.5)
        if axis[0] > 1.0e-6:
            axis[1] = (rotation[0, 1] + rotation[1, 0]) / (4.0 * axis[0])
            axis[2] = (rotation[0, 2] + rotation[2, 0]) / (4.0 * axis[0])
        elif axis[1] > 1.0e-6:
            axis[2] = (rotation[1, 2] + rotation[2, 1]) / (4.0 * axis[1])
        else:
            axis = np.asarray((0.0, 0.0, 1.0), dtype=float)
        norm = float(np.linalg.norm(axis))
        return angle * axis / max(norm, 1.0e-12)
    return angle * skew / (2.0 * sine)


@dataclass(frozen=True, slots=True)
class AlignedPose:
    """Output of one alignment update; target is retained while held."""

    target_pose: np.ndarray
    aligned: bool
    hold_reason: str | None
    stable_count: int

    def __post_init__(self) -> None:
        target = np.array(self.target_pose, dtype=float, copy=True)
        target.setflags(write=False)
        object.__setattr__(self, "target_pose", target)

    @property
    def pose(self) -> np.ndarray:
        return self.target_pose

    @property
    def valid(self) -> bool:
        return self.aligned and self.hold_reason is None

    @property
    def hold(self) -> bool:
        return self.hold_reason is not None


class SideAlignment:
    """Neutral alignment plus a zero-lag stationary wrist-jitter hold."""

    def __init__(
        self,
        *,
        neutral_robot: Any | None = None,
        stable_frames: int = 10,
        max_translation_step_m: float = 0.02,
        max_rotation_step_rad: float = 0.15,
        stale_after_ns: int = 50_000_000,
        position_scale: float = 1.0,
        pico_to_robot_rotation: Any | None = None,
        stationary_hold_dwell_s: float = 0.15,
        translation_hold_enter_velocity_m_s: float = 0.06,
        translation_hold_exit_velocity_m_s: float = 0.12,
        translation_hold_exit_position_error_m: float = 0.020,
        orientation_hold_enter_velocity_rad_s: float = 0.12,
        orientation_hold_exit_velocity_rad_s: float = 0.25,
        orientation_hold_exit_error_rad: float = 0.035,
        twist_lowpass_cutoff_hz: float = 0.8,
    ) -> None:
        if int(stable_frames) <= 0:
            raise ValueError("stable_frames must be positive")
        if not math.isfinite(max_translation_step_m) or max_translation_step_m <= 0.0:
            raise ValueError("max_translation_step_m must be finite and positive")
        if not math.isfinite(max_rotation_step_rad) or max_rotation_step_rad <= 0.0:
            raise ValueError("max_rotation_step_rad must be finite and positive")
        if int(stale_after_ns) < 0:
            raise ValueError("stale_after_ns must be non-negative")
        if not math.isfinite(position_scale) or position_scale <= 0.0:
            raise ValueError("position_scale must be finite and positive")
        positive_parameters = {
            "stationary_hold_dwell_s": stationary_hold_dwell_s,
            "translation_hold_enter_velocity_m_s": translation_hold_enter_velocity_m_s,
            "translation_hold_exit_velocity_m_s": translation_hold_exit_velocity_m_s,
            "translation_hold_exit_position_error_m": translation_hold_exit_position_error_m,
            "orientation_hold_enter_velocity_rad_s": orientation_hold_enter_velocity_rad_s,
            "orientation_hold_exit_velocity_rad_s": orientation_hold_exit_velocity_rad_s,
            "orientation_hold_exit_error_rad": orientation_hold_exit_error_rad,
            "twist_lowpass_cutoff_hz": twist_lowpass_cutoff_hz,
        }
        if any(not math.isfinite(value) or value <= 0.0 for value in positive_parameters.values()):
            raise ValueError("stationary hold parameters must be finite and positive")
        if translation_hold_exit_velocity_m_s <= translation_hold_enter_velocity_m_s:
            raise ValueError("translation hold exit velocity must exceed enter velocity")
        if orientation_hold_exit_velocity_rad_s <= orientation_hold_enter_velocity_rad_s:
            raise ValueError("orientation hold exit velocity must exceed enter velocity")
        self.stable_frames = int(stable_frames)
        self.max_translation_step_m = float(max_translation_step_m)
        self.max_rotation_step_rad = float(max_rotation_step_rad)
        self.stale_after_ns = int(stale_after_ns)
        self.position_scale = float(position_scale)
        # These are the proven stationary-hold thresholds from the senior
        # Tianji PICO teleoperation profile.  They freeze only settled jitter;
        # deliberate motion is not passed through a pose low-pass filter.
        self.stationary_hold_dwell_ns = int(stationary_hold_dwell_s * 1.0e9)
        self.translation_hold_enter_velocity_m_s = float(translation_hold_enter_velocity_m_s)
        self.translation_hold_exit_velocity_m_s = float(translation_hold_exit_velocity_m_s)
        self.translation_hold_exit_position_error_m = float(translation_hold_exit_position_error_m)
        self.orientation_hold_enter_velocity_rad_s = float(orientation_hold_enter_velocity_rad_s)
        self.orientation_hold_exit_velocity_rad_s = float(orientation_hold_exit_velocity_rad_s)
        self.orientation_hold_exit_error_rad = float(orientation_hold_exit_error_rad)
        self.twist_lowpass_cutoff_hz = float(twist_lowpass_cutoff_hz)
        self.neutral_robot = _pose_matrix(_IDENTITY if neutral_robot is None else neutral_robot)
        rotation = np.asarray(
            np.eye(3) if pico_to_robot_rotation is None else pico_to_robot_rotation,
            dtype=float,
        )
        if (
            rotation.shape != (3, 3)
            or not np.all(np.isfinite(rotation))
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1.0e-6)
            or not math.isclose(float(np.linalg.det(rotation)), 1.0, abs_tol=1.0e-6)
        ):
            raise ValueError("pico_to_robot_rotation must be a proper 3x3 rotation")
        self.pico_to_robot_rotation = rotation.copy()
        self._epoch: int | None = None
        self._last_timestamp_ns: int | None = None
        self._candidate: np.ndarray | None = None
        self._stable_count = 0
        self._aligned = False
        self._transform: np.ndarray | None = None
        self._pico_neutral: np.ndarray | None = None
        self._last_target: np.ndarray | None = None
        self._motion_pose: np.ndarray | None = None
        self._motion_timestamp_ns: int | None = None
        self._filtered_linear_velocity = np.zeros(3, dtype=float)
        self._filtered_angular_velocity = np.zeros(3, dtype=float)
        self._stationary_since_ns: int | None = None
        self._stationary_target: np.ndarray | None = None

    @property
    def aligned(self) -> bool:
        return self._aligned

    @property
    def stable_count(self) -> int:
        return self._stable_count

    @property
    def epoch(self) -> int | None:
        return self._epoch

    @property
    def transform(self) -> np.ndarray | None:
        return None if self._transform is None else self._transform.copy()

    @property
    def last_target(self) -> np.ndarray | None:
        return None if self._last_target is None else self._last_target.copy()

    def _clear_alignment(self) -> None:
        self._candidate = None
        self._stable_count = 0
        self._aligned = False
        self._transform = None
        self._pico_neutral = None
        self._motion_pose = None
        self._motion_timestamp_ns = None
        self._filtered_linear_velocity.fill(0.0)
        self._filtered_angular_velocity.fill(0.0)
        self._stationary_since_ns = None
        self._stationary_target = None

    def _stationary_hold(
        self,
        current: np.ndarray,
        target: np.ndarray,
        timestamp_ns: int,
    ) -> np.ndarray:
        """Freeze settled optical jitter and release immediately on real motion."""
        if self._motion_pose is None or self._motion_timestamp_ns is None:
            self._motion_pose = current.copy()
            self._motion_timestamp_ns = timestamp_ns
            self._stationary_since_ns = timestamp_ns
            return target

        dt = (timestamp_ns - self._motion_timestamp_ns) * 1.0e-9
        if dt <= 0.0:
            return self._stationary_target.copy() if self._stationary_target is not None else target
        linear_velocity = (current[:3, 3] - self._motion_pose[:3, 3]) / dt
        angular_velocity = _rotation_vector(
            current[:3, :3] @ self._motion_pose[:3, :3].T
        ) / dt
        alpha = 1.0 - math.exp(-2.0 * math.pi * self.twist_lowpass_cutoff_hz * dt)
        self._filtered_linear_velocity += alpha * (
            linear_velocity - self._filtered_linear_velocity
        )
        self._filtered_angular_velocity += alpha * (
            angular_velocity - self._filtered_angular_velocity
        )
        self._motion_pose = current.copy()
        self._motion_timestamp_ns = timestamp_ns

        linear_speed = float(np.linalg.norm(self._filtered_linear_velocity))
        angular_speed = float(np.linalg.norm(self._filtered_angular_velocity))
        if self._stationary_target is not None:
            position_error = float(
                np.linalg.norm(target[:3, 3] - self._stationary_target[:3, 3])
            )
            orientation_error = _rotation_distance(
                self._stationary_target[:3, :3], target[:3, :3]
            )
            if (
                linear_speed > self.translation_hold_exit_velocity_m_s
                or angular_speed > self.orientation_hold_exit_velocity_rad_s
                or position_error > self.translation_hold_exit_position_error_m
                or orientation_error > self.orientation_hold_exit_error_rad
            ):
                self._stationary_target = None
                self._stationary_since_ns = None
                return target
            return self._stationary_target.copy()

        if (
            linear_speed <= self.translation_hold_enter_velocity_m_s
            and angular_speed <= self.orientation_hold_enter_velocity_rad_s
        ):
            if self._stationary_since_ns is None:
                self._stationary_since_ns = timestamp_ns
            elif timestamp_ns - self._stationary_since_ns >= self.stationary_hold_dwell_ns:
                self._stationary_target = target.copy()
                return self._stationary_target.copy()
        else:
            self._stationary_since_ns = None
        return target

    def _target_or_neutral(self) -> np.ndarray:
        return self._last_target if self._last_target is not None else self.neutral_robot

    def _held(self, reason: str) -> AlignedPose:
        return AlignedPose(self._target_or_neutral(), False, reason, self._stable_count)

    def accept(
        self,
        wrist_pose: Any,
        active: bool,
        epoch: int,
        timestamp_ns: int,
        *,
        now_ns: int | None = None,
    ) -> AlignedPose:
        timestamp = int(timestamp_ns)
        epoch = int(epoch)
        if timestamp < 0 or epoch < 0:
            raise ValueError("epoch and timestamp must be non-negative")
        if self._epoch is None:
            self._epoch = epoch
        elif epoch != self._epoch:
            self._epoch = epoch
            self._last_timestamp_ns = timestamp
            self._clear_alignment()
            return self._held("epoch_change")
        duplicate = self._last_timestamp_ns is not None and timestamp == self._last_timestamp_ns
        if self._last_timestamp_ns is not None and timestamp < self._last_timestamp_ns:
            self._last_timestamp_ns = timestamp
            self._clear_alignment()
            return self._held("timestamp_rollback")
        if now_ns is not None and int(now_ns) - timestamp > self.stale_after_ns:
            self._last_timestamp_ns = timestamp
            self._clear_alignment()
            return self._held("stale")
        if not active:
            self._last_timestamp_ns = timestamp
            self._clear_alignment()
            return self._held("inactive")
        if duplicate:
            return AlignedPose(
                self._target_or_neutral(),
                self._aligned,
                None if self._aligned else "aligning",
                self._stable_count,
            )
        self._last_timestamp_ns = timestamp
        try:
            current = _pose_matrix(wrist_pose)
        except (TypeError, ValueError):
            self._clear_alignment()
            return self._held("invalid_pose")

        if self._aligned:
            assert self._candidate is not None
            assert self._pico_neutral is not None
            previous = self._candidate
            translation_step = float(np.linalg.norm(current[:3, 3] - previous[:3, 3]))
            rotation_step = _rotation_distance(previous[:3, :3], current[:3, :3])
            if translation_step > self.max_translation_step_m or rotation_step > self.max_rotation_step_rad:
                self._clear_alignment()
                self._candidate = current
                self._stable_count = 1
                return self._held("aligning")
            basis = self.pico_to_robot_rotation
            target = self.neutral_robot.copy()
            target[:3, 3] += self.position_scale * basis @ (
                current[:3, 3] - self._pico_neutral[:3, 3]
            )
            target[:3, :3] = (
                basis
                @ current[:3, :3]
                @ self._pico_neutral[:3, :3].T
                @ basis.T
                @ self.neutral_robot[:3, :3]
            )
            target = self._stationary_hold(current, target, timestamp)
            self._candidate = current
            self._last_target = target
            return AlignedPose(target, True, None, self._stable_count)

        if self._candidate is None:
            self._candidate = current
            self._stable_count = 1
        else:
            translation_step = float(np.linalg.norm(current[:3, 3] - self._candidate[:3, 3]))
            rotation_step = _rotation_distance(self._candidate[:3, :3], current[:3, :3])
            if translation_step > self.max_translation_step_m or rotation_step > self.max_rotation_step_rad:
                self._candidate = current
                self._stable_count = 1
            else:
                self._candidate = current
                self._stable_count += 1
        if self._stable_count >= self.stable_frames:
            self._candidate = current
            self._transform = self.neutral_robot @ np.linalg.inv(current)
            self._pico_neutral = current.copy()
            self._aligned = True
            target = self.neutral_robot.copy()
            self._motion_pose = current.copy()
            self._motion_timestamp_ns = timestamp
            self._stationary_since_ns = timestamp
            self._last_target = target
            return AlignedPose(target, True, None, self._stable_count)
        return self._held("aligning")

    def stale(self, now_ns: int) -> AlignedPose:
        if self._last_timestamp_ns is None or int(now_ns) - self._last_timestamp_ns <= self.stale_after_ns:
            return AlignedPose(self._target_or_neutral(), self._aligned, None if self._aligned else "aligning", self._stable_count)
        return self._held("stale")

    def realign(self) -> None:
        self._clear_alignment()

    def reset(self) -> None:
        self._clear_alignment()
        self._epoch = None
        self._last_timestamp_ns = None
        self._last_target = None


__all__ = ["AlignedPose", "PICO_TO_ROBOT_ROTATION", "SideAlignment"]
