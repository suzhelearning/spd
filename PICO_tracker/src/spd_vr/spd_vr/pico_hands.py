"""Atomic PICO hand-frame input and MediaPipe-compatible finger views."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

import numpy as np

try:
    from wuji_retargeting.mediapipe import apply_mediapipe_transformations
except ImportError as exc:  # pragma: no cover - exercised by packaging smoke tests
    raise ImportError(
        "spd_vr requires the editable local wuji-retargeting dependency"
    ) from exc

PICO_TO_MEDIAPIPE = np.asarray(
    [1, 2, 3, 4, 5, 7, 8, 9, 10, 12, 13, 14, 15, 17, 18, 19, 20, 22, 23, 24, 25],
    dtype=np.int64,
)
PICO_HAND_JOINT_COUNT = 26
MEDIAPIPE_JOINT_COUNT = 21

# ``apply_mediapipe_transformations`` produces the shared MANO/MediaPipe wrist
# frame.  Wuji Hand 2's frozen wrist frames use different axes.  These proper
# rotations are the matrix form of the tuned Hand 2 settings kept in the
# upstream examples:
#   left:  extrinsic XYZ (180, 0, -90) degrees
#   right: extrinsic XYZ (0, 180, -90) degrees
_MEDIAPIPE_TO_WUJI2_ROTATION = {
    "left": np.asarray(
        ((0.0, -1.0, 0.0), (-1.0, 0.0, 0.0), (0.0, 0.0, -1.0)),
        dtype=np.float64,
    ),
    "right": np.asarray(
        ((0.0, 1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, -1.0)),
        dtype=np.float64,
    ),
}


def mediapipe_to_wuji2_wrist_frame(points: Any, side: str) -> np.ndarray:
    """Rotate wrist-relative MediaPipe points into the Wuji2 wrist frame."""
    if side not in _MEDIAPIPE_TO_WUJI2_ROTATION:
        raise ValueError("side must be left or right")
    array = np.asarray(points, dtype=np.float64)
    if array.shape != (MEDIAPIPE_JOINT_COUNT, 3):
        raise HandFrameError(
            f"MediaPipe keypoints must have shape (21,3), got {array.shape}"
        )
    if not np.all(np.isfinite(array)):
        raise HandFrameError("MediaPipe keypoints must contain only finite values")
    return array @ _MEDIAPIPE_TO_WUJI2_ROTATION[side].T


class HandFrameError(ValueError):
    """Raised when one PICO hand does not contain a usable 26-joint frame."""


@dataclass(frozen=True)
class PicoHandFrame:
    """ROS-independent representation of one atomic dual-hand frame."""

    left_hand: np.ndarray
    right_hand: np.ndarray
    left_active: bool = True
    right_active: bool = True
    tracking_epoch: int = 0
    sequence_id: int = 0
    timestamp_ns: int = 0
    left_scale: float = 1.0
    right_scale: float = 1.0


class PicoHandsInput:
    """Convert the latest atomic PICO frame into two 21×3 hand arrays.

    Raw MediaPipe-ordered landmarks remain in the PICO source frame so the
    ``Retargeter`` stays the single owner of palm/MANO preprocessing.  A
    separate wrist-frame accessor is retained for visualization consumers.
    """

    def __init__(self, frame: PicoHandFrame | Mapping[str, Any] | Any | None = None) -> None:
        self._frame: PicoHandFrame | None = None
        if frame is not None:
            self.update(frame)

    @staticmethod
    def _array(value: Any, name: str) -> np.ndarray:
        array = np.asarray(value, dtype=np.float64)
        if array.shape == (PICO_HAND_JOINT_COUNT, 3):
            quaternion = np.zeros((PICO_HAND_JOINT_COUNT, 4), dtype=np.float64)
            quaternion[:, 3] = 1.0
            array = np.concatenate((array, quaternion), axis=1)
        if array.shape != (PICO_HAND_JOINT_COUNT, 7):
            raise HandFrameError(f"{name} must have shape (26,3) or (26,7), got {array.shape}")
        if not np.all(np.isfinite(array)):
            raise HandFrameError(f"{name} must contain only finite positions and quaternions")
        return array

    @staticmethod
    def _scale(value: Any, name: str) -> float:
        scale = float(value)
        if not np.isfinite(scale) or scale <= 0.0:
            raise HandFrameError(f"{name} must be finite and positive")
        return scale

    @classmethod
    def _from_message(cls, message: Any) -> PicoHandFrame:
        def pose_array(values: Any, name: str) -> np.ndarray:
            rows = []
            for pose in values:
                position = getattr(pose, "position", pose)
                if all(hasattr(position, axis) for axis in ("x", "y", "z")):
                    row = [position.x, position.y, position.z]
                    orientation = getattr(pose, "orientation", None)
                    if orientation is not None and all(hasattr(orientation, axis) for axis in ("x", "y", "z", "w")):
                        row.extend([orientation.x, orientation.y, orientation.z, orientation.w])
                    else:
                        row.extend([0.0, 0.0, 0.0, 1.0])
                else:
                    row = list(position)
                    if len(row) == 3:
                        row.extend([0.0, 0.0, 0.0, 1.0])
                rows.append(row)
            return cls._array(rows, name)

        header = getattr(message, "header", None)
        stamp = getattr(header, "stamp", None)
        timestamp_ns = (
            int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
            if stamp is not None else int(getattr(message, "timestamp_ns", 0))
        )
        return PicoHandFrame(
            left_hand=pose_array(message.left_joints, "left_joints"),
            right_hand=pose_array(message.right_joints, "right_joints"),
            left_active=bool(message.left_active),
            right_active=bool(message.right_active),
            tracking_epoch=int(getattr(message, "tracking_epoch", 0)),
            sequence_id=int(getattr(message, "sequence_id", 0)),
            timestamp_ns=timestamp_ns,
            left_scale=cls._scale(getattr(message, "left_scale", 1.0), "left_scale"),
            right_scale=cls._scale(getattr(message, "right_scale", 1.0), "right_scale"),
        )

    def update(
        self,
        frame: PicoHandFrame | Mapping[str, Any] | Any,
        right_hand: Any | None = None,
        *,
        left_active: bool = True,
        right_active: bool = True,
        tracking_epoch: int = 0,
        sequence_id: int = 0,
        timestamp_ns: int = 0,
        left_scale: float = 1.0,
        right_scale: float = 1.0,
    ) -> None:
        """Replace the immutable input snapshot.

        ``update(left, right, ...)`` is supported for replay/tests; a mapping
        uses the same field names as the NPZ contract, and a ROS ``PicoHands``
        message is accepted directly.
        """
        if right_hand is not None:
            frame = PicoHandFrame(
                left_hand=self._array(frame, "left_hand"),
                right_hand=self._array(right_hand, "right_hand"),
                left_active=left_active,
                right_active=right_active,
                tracking_epoch=tracking_epoch,
                sequence_id=sequence_id,
                timestamp_ns=timestamp_ns,
                left_scale=self._scale(left_scale, "left_scale"),
                right_scale=self._scale(right_scale, "right_scale"),
            )
        elif isinstance(frame, PicoHandFrame):
            frame = PicoHandFrame(
                left_hand=self._array(frame.left_hand, "left_hand"),
                right_hand=self._array(frame.right_hand, "right_hand"),
                left_active=bool(frame.left_active),
                right_active=bool(frame.right_active),
                tracking_epoch=int(frame.tracking_epoch),
                sequence_id=int(frame.sequence_id),
                timestamp_ns=int(frame.timestamp_ns),
                left_scale=self._scale(frame.left_scale, "left_scale"),
                right_scale=self._scale(frame.right_scale, "right_scale"),
            )
        elif isinstance(frame, Mapping):
            frame = PicoHandFrame(
                left_hand=self._array(frame["left_hand"], "left_hand"),
                right_hand=self._array(frame["right_hand"], "right_hand"),
                left_active=bool(frame.get("left_active", True)),
                right_active=bool(frame.get("right_active", True)),
                tracking_epoch=int(frame.get("tracking_epoch", 0)),
                sequence_id=int(frame.get("sequence_id", 0)),
                timestamp_ns=int(frame.get("timestamp_ns", 0)),
                left_scale=self._scale(frame.get("left_scale", 1.0), "left_scale"),
                right_scale=self._scale(frame.get("right_scale", 1.0), "right_scale"),
            )
        else:
            frame = self._from_message(frame)
        self._frame = frame

    def get_side_mediapipe_landmarks(self, side: str) -> np.ndarray:
        """Return one side in MediaPipe index order and the PICO source frame."""
        if self._frame is None:
            raise HandFrameError("PICO hand frame is not initialized")
        if side not in {"left", "right"}:
            raise ValueError("side must be left or right")
        source = self._frame.left_hand if side == "left" else self._frame.right_hand
        scale = self._frame.left_scale if side == "left" else self._frame.right_scale
        active = self._frame.left_active if side == "left" else self._frame.right_active
        if not active:
            return np.zeros((MEDIAPIPE_JOINT_COUNT, 3), dtype=np.float64)
        return source[PICO_TO_MEDIAPIPE, :3].copy() * scale

    def get_side_fingers_data(self, side: str) -> np.ndarray:
        """Return wrist-local MANO points for overlays and diagnostics.

        Retargeting code should use :meth:`get_side_mediapipe_landmarks`, then
        let ``Retargeter.retarget`` perform this transformation exactly once.
        """
        mediapipe_points = self.get_side_mediapipe_landmarks(side)
        transformed = np.asarray(
            apply_mediapipe_transformations(mediapipe_points, side),
            dtype=np.float64,
        )
        if transformed.shape != (MEDIAPIPE_JOINT_COUNT, 3):
            raise HandFrameError(
                f"{side} transformation returned {transformed.shape}, expected (21,3)"
            )
        if not np.all(np.isfinite(transformed)):
            raise HandFrameError(f"{side} transformed fingers are non-finite")
        if not np.allclose(transformed[0], 0.0, atol=1e-9):
            raise HandFrameError(f"{side} transformed Wrist is not the origin")
        return transformed

    def get_fingers_data(self) -> dict[str, np.ndarray]:
        """Return left/right MediaPipe arrays with Wrist at row zero."""
        return {
            "left_fingers": self.get_side_fingers_data("left"),
            "right_fingers": self.get_side_fingers_data("right"),
        }

    @property
    def frame(self) -> PicoHandFrame:
        if self._frame is None:
            raise HandFrameError("PICO hand frame is not initialized")
        return self._frame


__all__ = [
    "HandFrameError",
    "MEDIAPIPE_JOINT_COUNT",
    "PICO_HAND_JOINT_COUNT",
    "PICO_TO_MEDIAPIPE",
    "PicoHandFrame",
    "PicoHandsInput",
    "mediapipe_to_wuji2_wrist_frame",
]
