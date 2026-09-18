"""PICO_2 frame validation used by the ROS entry without the Zenoh bridge."""
from __future__ import annotations

from collections.abc import Callable
import math
import time
from typing import Any

import numpy as np

from .wire import TrackingFrame, encode_tracking

_IDENTITY_POSE = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)


def _identity_hand() -> tuple[tuple[float, ...], ...]:
    return (_IDENTITY_POSE,) * 26


def _pose_values(pose: Any, name: str) -> tuple[float, ...]:
    values = tuple(float(v) for v in getattr(pose, "position")) + tuple(
        float(v) for v in getattr(pose, "quaternion_xyzw")
    )
    if len(values) != 7 or not all(math.isfinite(v) for v in values):
        raise ValueError(f"{name} must contain seven finite values")
    return values


def _canonical_hand(hand: Any, side: str) -> tuple[bool, float, tuple[tuple[float, ...], ...]]:
    active = bool(getattr(hand, "valid"))
    joints = tuple(getattr(hand, "joints"))
    if len(joints) != 26:
        raise ValueError(f"{side} hand must contain 26 joints")
    if not active:
        return False, 1.0, _identity_hand()
    decoded: list[tuple[float, ...]] = []
    for index, joint in enumerate(joints):
        if not bool(getattr(joint, "valid")):
            return False, 1.0, _identity_hand()
        values = _pose_values(joint, f"{side}.joint[{index}]")
        quaternion = np.asarray(values[3:7], dtype=np.float64)
        norm = float(np.linalg.norm(quaternion))
        if not math.isfinite(norm) or norm <= 0.0:
            return False, 1.0, _identity_hand()
        decoded.append(values[:3] + tuple(float(v) for v in quaternion / norm))
    return True, 1.0, tuple(decoded)


class PicoRosSourceCore:
    """Normalize raw receiver frames and reject duplicate/rollback timestamps."""

    def __init__(self, *, clock_ns: Callable[[], int] = time.monotonic_ns) -> None:
        self._clock_ns = clock_ns
        self._epoch = 0
        self._sequence = 0
        self._last_timestamp_ms: int | None = None
        self._last_bridge_ns = 0

    def reset_stream(self) -> None:
        self._epoch = max(1, self._epoch + 1)
        self._last_timestamp_ms = None

    def accept_frame(self, frame: Any) -> bytes | None:
        try:
            timestamp_ms = int(getattr(frame, "timestamp_ms"))
            if timestamp_ms < 0:
                raise ValueError("timestamp_ms must be non-negative")
            if self._epoch == 0:
                self.reset_stream()
            if self._last_timestamp_ms is not None:
                if timestamp_ms == self._last_timestamp_ms:
                    return None
                if timestamp_ms < self._last_timestamp_ms:
                    self.reset_stream()
            head = _pose_values(getattr(frame, "head"), "head")
            head_valid = bool(getattr(frame.head, "valid"))
            left_active, left_scale, left_hand = _canonical_hand(getattr(frame, "left"), "left")
            right_active, right_scale, right_hand = _canonical_hand(getattr(frame, "right"), "right")
            bridge_ns = max(1, int(self._clock_ns()))
            bridge_ns = max(bridge_ns, self._last_bridge_ns + 1)
            tracking = TrackingFrame(
                sequence=self._sequence + 1,
                tracking_epoch=max(1, self._epoch),
                source_timestamp_ns=max(1, timestamp_ms * 1_000_000),
                bridge_monotonic_ns=bridge_ns,
                left_active=left_active,
                right_active=right_active,
                head_valid=head_valid,
                left_scale=left_scale,
                right_scale=right_scale,
                head_pose=head if head_valid else _IDENTITY_POSE,
                left_hand=left_hand,
                right_hand=right_hand,
            )
            packet = encode_tracking(tracking)
        except (AttributeError, TypeError, ValueError, OverflowError):
            return None
        self._sequence += 1
        self._last_timestamp_ms = timestamp_ms
        self._last_bridge_ns = bridge_ns
        return packet


__all__ = ["PicoRosSourceCore"]
