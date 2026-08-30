"""Independent PICO-wrist neutral alignment for one arm side."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any

import numpy as np


_IDENTITY = np.eye(4, dtype=float)


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
    """Ten-frame, reset-on-jump neutral alignment for one wrist."""

    def __init__(
        self,
        *,
        neutral_robot: Any | None = None,
        stable_frames: int = 10,
        max_translation_step_m: float = 0.02,
        max_rotation_step_rad: float = 0.15,
        stale_after_ns: int = 50_000_000,
        position_scale: float = 1.0,
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
        self.stable_frames = int(stable_frames)
        self.max_translation_step_m = float(max_translation_step_m)
        self.max_rotation_step_rad = float(max_rotation_step_rad)
        self.stale_after_ns = int(stale_after_ns)
        self.position_scale = float(position_scale)
        self.neutral_robot = _pose_matrix(_IDENTITY if neutral_robot is None else neutral_robot)
        self._epoch: int | None = None
        self._last_timestamp_ns: int | None = None
        self._candidate: np.ndarray | None = None
        self._stable_count = 0
        self._aligned = False
        self._transform: np.ndarray | None = None
        self._last_target: np.ndarray | None = None

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
        if self._last_timestamp_ns is not None and timestamp < self._last_timestamp_ns:
            self._last_timestamp_ns = timestamp
            self._clear_alignment()
            return self._held("timestamp_rollback")
        self._last_timestamp_ns = timestamp
        if now_ns is not None and int(now_ns) - timestamp > self.stale_after_ns:
            return self._held("stale")
        if not active:
            self._clear_alignment()
            return self._held("inactive")
        try:
            current = _pose_matrix(wrist_pose)
        except (TypeError, ValueError):
            self._clear_alignment()
            return self._held("invalid_pose")

        if self._aligned:
            assert self._candidate is not None
            translation_step = float(np.linalg.norm(current[:3, 3] - self._candidate[:3, 3]))
            rotation_step = _rotation_distance(self._candidate[:3, :3], current[:3, :3])
            if translation_step > self.max_translation_step_m or rotation_step > self.max_rotation_step_rad:
                self._clear_alignment()
                self._candidate = current
                self._stable_count = 1
                return self._held("aligning")
            delta = np.linalg.inv(self._candidate) @ current
            target = self._transform @ current if self._transform is not None else self.neutral_robot @ delta
            target[:3, 3] = self.neutral_robot[:3, 3] + self.position_scale * (target[:3, 3] - self.neutral_robot[:3, 3])
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
            self._aligned = True
            target = self.neutral_robot.copy()
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


__all__ = ["AlignedPose", "SideAlignment"]
