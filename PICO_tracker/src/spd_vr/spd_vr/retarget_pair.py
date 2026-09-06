"""Simulation-only atomic Wuji Hand 2 retargeting for both sides."""

from __future__ import annotations
from copy import deepcopy
from dataclasses import dataclass
from enum import Enum
import hashlib
from pathlib import Path
from typing import Any, Callable, Sequence

import numpy as np
import yaml

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
    return factory(value, hand_side=side)


def _config_from(value: str | Path | dict[str, Any]) -> dict[str, Any]:
    if isinstance(value, dict):
        config = deepcopy(value)
    else:
        path = Path(value).resolve()
        config = yaml.safe_load(path.read_text(encoding="utf-8"))
        if not isinstance(config, dict):
            raise ValueError(f"retarget config must be a mapping: {path}")
        config["__yaml_dir"] = str(path.parent)
    return config


def _manifest_hand_contract(
    manifest_path: str | Path, urdf_path: str | Path
) -> tuple[dict[str, Any], list[str], list[str]]:
    from .manifest import load_manifest

    manifest_file = Path(manifest_path).resolve()
    document = load_manifest(manifest_file)
    source = document.get("source")
    if not isinstance(source, dict):
        raise ValueError("manifest source metadata is missing")
    urdf = Path(urdf_path).resolve()
    if not urdf.is_file():
        raise FileNotFoundError(urdf)
    declared = source.get("urdf")
    if declared and Path(str(declared)).name != urdf.name:
        raise ValueError(f"manifest URDF path mismatch: declared={declared!r}, actual={urdf}")
    expected_hash = source.get("urdf_sha256")
    actual_hash = hashlib.sha256(urdf.read_bytes()).hexdigest()
    if not isinstance(expected_hash, str) or actual_hash != expected_hash:
        raise ValueError("authoritative URDF hash mismatch")
    expected_manifest_hash = document.get("manifest_sha256")
    normalized = dict(document)
    normalized["manifest_sha256"] = ""
    actual_manifest_hash = hashlib.sha256(
        yaml.safe_dump(normalized, sort_keys=True, allow_unicode=True).encode("utf-8")
    ).hexdigest()
    if not isinstance(expected_manifest_hash, str) or actual_manifest_hash != expected_manifest_hash:
        raise ValueError("model manifest hash mismatch")
    hand_order = document.get("hand_joint_order")
    if not isinstance(hand_order, dict):
        raise ValueError("manifest hand_joint_order is missing")
    entries = document["joints"]
    actuator_order = document.get("actuator_order")
    by_actuator = {entry["actuator"]: entry for entry in entries}
    def side_order(side: str) -> list[str]:
        names = hand_order.get(side)
        if not isinstance(names, list) or len(names) != 20 or len(set(names)) != 20:
            raise ValueError(f"manifest {side} hand_joint_order must contain 20 unique joints")
        actuators = [
            by_actuator[actuator]["joint"]
            for actuator in actuator_order
            if by_actuator[actuator].get("group") == "hand"
            and by_actuator[actuator].get("side") == side
        ]
        if set(actuators) != set(names):
            raise ValueError(f"manifest {side} actuator order disagrees with hand order")
        return actuators

    return document, side_order("left"), side_order("right")


class WujiRetargetPair:
    """Own two Retargeters and process one atomic PICO frame at a time."""

    @classmethod
    def from_manifest(
        cls,
        left_config: str | Path | dict[str, Any],
        right_config: str | Path | dict[str, Any],
        manifest_path: str | Path,
        urdf_path: str | Path,
    ) -> "WujiRetargetPair":
        _, left_names, right_names = _manifest_hand_contract(manifest_path, urdf_path)
        left = _config_from(left_config)
        right = _config_from(right_config)
        for config, side, names in (
            (left, "left", left_names),
            (right, "right", right_names),
        ):
            optimizer = config.setdefault("optimizer", {})
            optimizer["hand_side"] = side
            optimizer["urdf_path"] = str(Path(urdf_path).resolve())
            optimizer["active_joint_names"] = names
            optimizer.pop("mjcf_path", None)
        factory = lambda config, *, hand_side: Retargeter.from_config(config, hand_side=hand_side)
        return cls(
            left,
            right,
            left_actuator_joint_names=left_names,
            right_actuator_joint_names=right_names,
            retargeter_factory=factory,
        )

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
        self._left_limits = self._source_limits(self.left_retargeter)
        self._right_limits = self._source_limits(self.right_retargeter)
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

    @staticmethod
    def _source_limits(retargeter: Any) -> np.ndarray:
        robot = getattr(getattr(retargeter, "optimizer", None), "robot", None)
        limits = getattr(robot, "joint_limits", None)
        if limits is None:
            return np.full((20, 2), [-np.inf, np.inf], dtype=np.float64)
        limits = np.asarray(limits, dtype=np.float64)
        if limits.shape != (20, 2) or not np.all(np.isfinite(limits)):
            raise ValueError(f"retargeter joint limits must be finite with shape (20,2), got {limits.shape}")
        if np.any(limits[:, 0] > limits[:, 1]):
            raise ValueError("retargeter joint limits are inverted")
        return limits

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
    def _mapped_target(
        retargeter: Any, points: np.ndarray, perm: np.ndarray, limits: np.ndarray
    ) -> np.ndarray:
        value = np.asarray(retargeter.retarget(points), dtype=np.float64)
        if value.shape != (20,) or not np.all(np.isfinite(value)):
            raise ValueError("retargeter returned a non-finite target with shape other than (20,)")
        value = np.clip(value, limits[:, 0], limits[:, 1])
        mapped = value[perm]
        if not np.all(np.isfinite(mapped)):
            raise ValueError("mapped joint target is non-finite")
        return mapped

    @staticmethod
    def _resilient_input(frame: PicoHandFrame | dict[str, Any] | Any) -> PicoHandsInput:
        """Validate each side independently without retaining malformed data."""
        if not isinstance(frame, (PicoHandFrame, dict)):
            return PicoHandsInput(frame)
        raw = frame if isinstance(frame, PicoHandFrame) else PicoHandFrame(
            frame["left_hand"],
            frame["right_hand"],
            bool(frame.get("left_active", True)),
            bool(frame.get("right_active", True)),
            int(frame.get("tracking_epoch", 0)),
            int(frame.get("sequence_id", 0)),
            int(frame.get("timestamp_ns", 0)),
            frame.get("left_scale", 1.0),
            frame.get("right_scale", 1.0),
        )

        def checked(value: Any, name: str) -> tuple[np.ndarray, bool]:
            try:
                return PicoHandsInput._array(value, name), True
            except (TypeError, ValueError):
                safe = np.zeros((26, 7), dtype=np.float64)
                safe[:, 6] = 1.0
                return safe, False

        left, left_ok = checked(raw.left_hand, "left_hand")
        right, right_ok = checked(raw.right_hand, "right_hand")
        try:
            left_scale = PicoHandsInput._scale(raw.left_scale, "left_scale")
            left_scale_ok = True
        except (TypeError, ValueError):
            left_scale, left_scale_ok = 1.0, False
        try:
            right_scale = PicoHandsInput._scale(raw.right_scale, "right_scale")
            right_scale_ok = True
        except (TypeError, ValueError):
            right_scale, right_scale_ok = 1.0, False
        safe = PicoHandFrame(
            left,
            right,
            bool(raw.left_active and left_ok and left_scale_ok),
            bool(raw.right_active and right_ok and right_scale_ok),
            int(raw.tracking_epoch),
            int(raw.sequence_id),
            int(raw.timestamp_ns),
            left_scale,
            right_scale,
        )
        return PicoHandsInput(safe)

    def retarget(self, frame: PicoHandFrame | dict[str, Any] | Any) -> RetargetedHands:
        """Retarget both active sides while holding only failed/inactive sides."""
        input_frame = self._resilient_input(frame)
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
                    input_frame.get_side_mediapipe_landmarks("left"),
                    self._left_perm,
                    self._left_limits,
                )
                left_valid = True
                left_reason = HandHoldReason.NONE
            except Exception:
                left_reason = HandHoldReason.SOLVER_FAILURE
        if source.right_active:
            try:
                self._right_target = self._mapped_target(
                    self.right_retargeter,
                    input_frame.get_side_mediapipe_landmarks("right"),
                    self._right_perm,
                    self._right_limits,
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
