"""Operator-only stereo view; never enters policy camera datasets."""

from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Any, Callable, Mapping

import numpy as np

OPERATOR_WIDTH = 1280
OPERATOR_HEIGHT = 720
OPERATOR_HZ = 60
MOTION_TO_PHOTON_P95_MS = 120.0


class OperatorViewError(RuntimeError):
    pass


@dataclass(frozen=True)
class StereoFrame:
    timestamp_ns: int
    left_rgb: np.ndarray
    right_rgb: np.ndarray
    latency_ms: float


class OperatorViewProvider:
    """Choose APK-local body transform streaming or remote XRoboToolkit vision."""

    def __init__(
        self,
        *,
        mode: str = "remote",
        remote_renderer: Callable[[Any, Mapping[str, Any]], tuple[np.ndarray, np.ndarray]] | None = None,
        latency_samples_ms: list[float] | None = None,
    ) -> None:
        if mode not in {"local_scene", "remote"}:
            raise ValueError("mode must be local_scene or remote")
        self.mode = mode
        self.remote_renderer = remote_renderer
        self.latency_samples_ms = list(latency_samples_ms or [])
        self._last_timestamp_ns = -1

    def body_transforms(self, model: Any, data: Any) -> dict[str, dict[str, list[float]]]:
        """Return transforms for APK-local scene rendering without mutating data."""
        try:
            import mujoco
        except ImportError as exc:  # pragma: no cover
            raise ImportError("mujoco is required for local operator scene transforms") from exc
        transforms: dict[str, dict[str, list[float]]] = {}
        for body_id in range(model.nbody):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)
            if not name:
                continue
            transforms[name] = {
                "position": [float(value) for value in data.xpos[body_id]],
                "quaternion_wxyz": [float(value) for value in data.xquat[body_id]],
            }
        return transforms

    def capture_stereo(self, timestamp_ns: int, head_pose: Any, body_transforms: Mapping[str, Any]) -> StereoFrame:
        if timestamp_ns < self._last_timestamp_ns:
            raise OperatorViewError("operator view timestamp moved backwards")
        self._last_timestamp_ns = int(timestamp_ns)
        start = time.perf_counter_ns()
        if self.mode == "local_scene":
            if self.remote_renderer is None:
                raise OperatorViewError("local_scene mode requires a renderer callback")
            left, right = self.remote_renderer(head_pose, body_transforms)
        else:
            if self.remote_renderer is None:
                raise OperatorViewError("remote mode requires XRoboToolkit renderer callback")
            left, right = self.remote_renderer(head_pose, {})
        left = np.asarray(left)
        right = np.asarray(right)
        expected = (OPERATOR_HEIGHT, OPERATOR_WIDTH, 3)
        if left.dtype != np.uint8 or left.shape != expected or right.dtype != np.uint8 or right.shape != expected:
            raise OperatorViewError("operator stereo pair must be uint8[720,1280,3]")
        latency_ms = (time.perf_counter_ns() - start) / 1e6
        self.latency_samples_ms.append(latency_ms)
        return StereoFrame(int(timestamp_ns), left.copy(), right.copy(), latency_ms)

    def readiness(self) -> dict[str, float | bool]:
        if not self.latency_samples_ms:
            return {"ready": False, "motion_to_photon_p95_ms": float("inf")}
        p95 = float(np.percentile(np.asarray(self.latency_samples_ms, dtype=np.float64), 95))
        return {"ready": p95 < MOTION_TO_PHOTON_P95_MS, "motion_to_photon_p95_ms": p95}


__all__ = [
    "MOTION_TO_PHOTON_P95_MS",
    "OPERATOR_HZ",
    "OPERATOR_HEIGHT",
    "OPERATOR_WIDTH",
    "OperatorViewError",
    "OperatorViewProvider",
    "StereoFrame",
]
