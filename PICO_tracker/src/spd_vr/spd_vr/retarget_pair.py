"""Simulation-only atomic Wuji Hand 2 retargeting for both sides."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Any, Callable, Sequence

import numpy as np

from .pico_hands import PicoHandFrame, PicoHandsInput

try:
    from wuji_retargeting import Retargeter
except ImportError as exc:  # pragma: no cover - packaging failure path
    raise ImportError(
        "spd_vr requires the editable local wuji-retargeting dependency"
    ) from exc


class HandHoldReason(str, Enum):
    NONE = "none"
    INACTIVE = "inactive"
    SOLVER_FAILURE = "solver_failure"
    STALE = "stale"
    INVALID = "invalid"


@dataclass(frozen=True)
class RetargetedHands:
    tracking_epoch: int
    sequence_id: int
    left_qpos: np.ndarray
    right_qpos: np.ndarray
    left_valid: bool
    right_valid: bool
    left_hold_reason: HandHoldReason
    right_hold_reason: HandHoldReason


def qpos_reorder_perm(
    src_joint_names: Sequence[str], dst_joint_names: Sequence[str]
) -> np.ndarray:
    """Return strict source→destination indices; never silently use identity."""
    src = list(src_joint_names)
    dst = list(dst_joint_names)
    if not src or not dst:
        raise ValueError("source and destination joint names must not be empty")
    if len(src) != len(dst):
        raise ValueError(f"joint count mismatch: source={len(src)}, destination={len(dst)}")
    if len(set(src)) != len(src) or len(set(dst)) != len(dst):
        raise ValueError("joint names must be unique on both sides")
    if set(src) != set(dst):
        missing = sorted(set(dst) - set(src))
        extra = sorted(set(src) - set(dst))
        raise ValueError(f"joint-name mapping failed: missing={missing}, extra={extra}")
    source_index = {name: index for index, name in enumerate(src)}
    return np.asarray([source_index[name] for name in dst], dtype=np.int64)


def _resolve_path(config: dict[str, Any], key: str) -> Path:
    raw = (config.get("optimizer") or {}).get(key)
    if not raw:
        raise ValueError(f"optimizer.{key} is required for strict Wuji mapping")
    path = Path(raw)
    if not path.is_absolute():
        yaml_dir = config.get("__yaml_dir")
        if not yaml_dir:
            raise ValueError(f"optimizer.{key} is relative but __yaml_dir is missing")
        path = Path(yaml_dir) / path
    path = path.resolve()
    if not path.is_file():
        raise FileNotFoundError(f"optimizer.{key} not found: {path}")
    return path


def mjcf_actuator_joint_names(path: str | Path) -> list[str]:
    """Resolve official MJCF actuator order by name, including every actuator."""
    try:
        import mujoco
    except ImportError as exc:  # pragma: no cover - dependency setup
        raise ImportError("mujoco is required to resolve Hand2 actuator order") from exc
    model = mujoco.MjModel.from_xml_path(str(path))
    names: list[str] = []
    for actuator_id in range(model.nu):
        joint_id = int(model.actuator_trnid[actuator_id, 0])
        if joint_id < 0:
            raise ValueError(f"MJCF actuator {actuator_id} is not joint-backed")
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
        if not name:
            raise ValueError(f"MJCF actuator {actuator_id} has an unnamed joint")
        names.append(name)
    if len(set(names)) != len(names):
        raise ValueError("MJCF actuator joint names are duplicated")
    return names


def _retargeter_from(value: Any, side: str, factory: Callable[..., Any]) -> Any:
    if hasattr(value, "retarget") and hasattr(value, "reset_filter"):
        return value
    return factory(str(value), hand_side=side)


class WujiRetargetPair:
    """Own two Retargeters and process one atomic PICO frame at a time."""

    def __init__(
        self,
        left: Any,
        right: Any,
        *,
        left_actuator_joint_names: Sequence[str] | None = None,
        right_actuator_joint_names: Sequence[str] | None = None,
        retargeter_factory: Callable[..., Any] = Retargeter.from_yaml,
    ) -> None:
        self.left_retargeter = _retargeter_from(left, "left", retargeter_factory)
        self.right_retargeter = _retargeter_from(right, "right", retargeter_factory)
        self._left_perm = self._build_perm(
            self.left_retargeter, left_actuator_joint_names, "left"
        )
        self._right_perm = self._build_perm(
            self.right_retargeter, right_actuator_joint_names, "right"
        )
        self._left_target = self._initial_target(self.left_retargeter)
        self._right_target = self._initial_target(self.right_retargeter)
        if self._left_target.shape != self._right_target.shape:
            raise ValueError("left/right retargeters must both expose 20 joints")
        if self._left_target.size != 20:
            raise ValueError(f"Wuji Hand 2 target must have 20 joints, got {self._left_target.size}")
        self._epoch: int | None = None
        self._last_sequence: int | None = None

    @staticmethod
    def _initial_target(retargeter: Any) -> np.ndarray:
        robot = getattr(getattr(retargeter, "optimizer", None), "robot", None)
        limits = getattr(robot, "joint_limits", None)
        if limits is None:
            return np.zeros(20, dtype=np.float64)
        limits_array = np.asarray(limits, dtype=np.float64)
        if limits_array.shape != (20, 2):
            raise ValueError(f"Wuji Hand 2 joint limits must have shape (20,2), got {limits_array.shape}")
        return limits_array.mean(axis=1)

    @staticmethod
    def _source_names(retargeter: Any, side: str) -> list[str]:
        names = getattr(getattr(retargeter, "optimizer", None), "robot", None)
        names = getattr(names, "dof_joint_names", None)
        if names is None:
            raise ValueError(f"{side} retargeter does not expose URDF joint names")
        names = list(names)
        if len(names) != 20:
            raise ValueError(f"{side} retargeter must expose 20 URDF joints, got {len(names)}")
        return names

    def _build_perm(
        self,
        retargeter: Any,
        actuator_names: Sequence[str] | None,
        side: str,
    ) -> np.ndarray:
        source_names = self._source_names(retargeter, side)
        if actuator_names is None:
            config = getattr(retargeter, "config", None)
            if not isinstance(config, dict):
                raise ValueError(f"{side} retargeter config is unavailable for MJCF mapping")
            mjcf_path = _resolve_path(config, "mjcf_path")
            actuator_names = mjcf_actuator_joint_names(mjcf_path)
        return qpos_reorder_perm(source_names, actuator_names)

    def reset_filter(self, tracking_epoch: int | None = None) -> None:
        self.left_retargeter.reset_filter()
        self.right_retargeter.reset_filter()
        if tracking_epoch is not None:
            self._epoch = int(tracking_epoch)
        self._last_sequence = None

    @staticmethod
    def _mapped_target(retargeter: Any, points: np.ndarray, perm: np.ndarray) -> np.ndarray:
        value = np.asarray(retargeter.retarget(points), dtype=np.float64)
        if value.shape != (20,):
            raise ValueError(f"retargeter returned {value.shape}, expected (20,)")
        if not np.all(np.isfinite(value)):
            raise ValueError("retargeter returned non-finite joint target")
        mapped = value[perm]
        if not np.all(np.isfinite(mapped)):
            raise ValueError("mapped joint target is non-finite")
        return mapped

    def retarget(self, frame: PicoHandFrame | dict[str, Any] | Any) -> RetargetedHands:
        """Retarget both active sides while holding only failed/inactive sides."""
        input_frame = PicoHandsInput(frame)
        source = input_frame.frame
        epoch = int(source.tracking_epoch)
        sequence = int(source.sequence_id)
        if self._epoch != epoch:
            self.reset_filter(epoch)
        elif self._last_sequence is not None and sequence <= self._last_sequence:
            return RetargetedHands(
                epoch, sequence, self._left_target.copy(), self._right_target.copy(),
                False, False, HandHoldReason.STALE, HandHoldReason.STALE,
            )
        self._last_sequence = sequence
        left_valid = False
        right_valid = False
        left_reason = HandHoldReason.INACTIVE
        right_reason = HandHoldReason.INACTIVE
        if source.left_active:
            try:
                self._left_target = self._mapped_target(
                    self.left_retargeter,
                    input_frame.get_side_fingers_data("left"),
                    self._left_perm,
                )
                left_valid = True
                left_reason = HandHoldReason.NONE
            except Exception:
                left_reason = HandHoldReason.SOLVER_FAILURE
        if source.right_active:
            try:
                self._right_target = self._mapped_target(
                    self.right_retargeter,
                    input_frame.get_side_fingers_data("right"),
                    self._right_perm,
                )
                right_valid = True
                right_reason = HandHoldReason.NONE
            except Exception:
                right_reason = HandHoldReason.SOLVER_FAILURE

        return RetargetedHands(
            tracking_epoch=epoch,
            sequence_id=sequence,
            left_qpos=self._left_target.copy(),
            right_qpos=self._right_target.copy(),
            left_valid=left_valid,
            right_valid=right_valid,
            left_hold_reason=left_reason,
            right_hold_reason=right_reason,
        )


__all__ = [
    "HandHoldReason",
    "RetargetedHands",
    "WujiRetargetPair",
    "mjcf_actuator_joint_names",
    "qpos_reorder_perm",
]
