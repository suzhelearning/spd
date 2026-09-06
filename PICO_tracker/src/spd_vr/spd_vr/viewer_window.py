"""Small adapter around MuJoCo's passive viewer.

The adapter deliberately owns no model or data.  ``PlantController`` remains the
single owner of the complete simulation state, while this class only renders it.
"""

from __future__ import annotations

from typing import Any, Callable, Mapping

import numpy as np


# Standard MediaPipe 21-landmark topology.  The first twenty edges keep each
# wrist-to-fingertip chain intact; the last three make the palm outline legible.
MEDIAPIPE_CONNECTIONS = (
    (0, 1), (1, 2), (2, 3), (3, 4),
    (0, 5), (5, 6), (6, 7), (7, 8),
    (0, 9), (9, 10), (10, 11), (11, 12),
    (0, 13), (13, 14), (14, 15), (15, 16),
    (0, 17), (17, 18), (18, 19), (19, 20),
    (5, 9), (9, 13), (13, 17),
)


class ViewerWindow:
    def __init__(
        self,
        model: Any | None = None,
        data: Any | None = None,
        *,
        headless: bool = False,
        clock_ns: Callable[[], int] | None = None,
        shutdown: Callable[[], None] | None = None,
        control: Callable[[str], None] | None = None,
        state: Callable[[], str] | None = None,
        pose_markers: Callable[[], Mapping[str, Any]] | None = None,
        hand_keypoints: Callable[[], Mapping[str, Any]] | None = None,
        window: Any | None = None,
    ) -> None:
        self.model = model
        self.data = data
        self.headless = bool(headless)
        self._clock_ns = clock_ns
        self._shutdown = shutdown
        self._control = control
        self._state = state
        self._pose_markers = pose_markers
        self._hand_keypoints = hand_keypoints
        self._window = window
        self._closed = False
        self._shutdown_sent = False
        self.hud: dict[str, Any] = {}

    @property
    def window(self) -> Any | None:
        return self._window

    @property
    def closed(self) -> bool:
        return self._closed

    def is_running(self) -> bool:
        if self._closed:
            return False
        running = getattr(self._window, "is_running", None)
        return bool(running()) if callable(running) else True

    @property
    def shutdown_sent(self) -> bool:
        return self._shutdown_sent

    def open(self) -> "ViewerWindow":
        if self._window is not None or self.headless or self._closed:
            return self
        if self.model is None or self.data is None:
            raise ValueError("a model and data are required for a visible viewer")
        try:
            from mujoco import viewer as mujoco_viewer
        except ImportError as exc:  # pragma: no cover - environment dependent
            raise RuntimeError("visible viewer requires mujoco.viewer") from exc
        self._window = mujoco_viewer.launch_passive(
            self.model,
            self.data,
            key_callback=self.on_key,
            show_left_ui=False,
            show_right_ui=False,
        )
        return self
    @staticmethod
    def _key_name(key: Any) -> str:
        if isinstance(key, str):
            return key.strip().lower().removeprefix("key_")
        if isinstance(key, int):
            if key in (27, 256):
                return "escape"
            if key in (ord("q"), ord("Q")):
                return "q"
            if key in (ord("r"), ord("R")):
                return "r"
            if key in (ord("n"), ord("N")):
                return "n"
            if key == 32:
                return "space"
        return str(key).strip().lower().removeprefix("key_")

    def on_key(self, key: Any) -> None:
        """Handle one MuJoCo/GLFW key event without repeated shutdown calls."""
        name = self._key_name(key)
        if name in {"q", "escape", "esc"}:
            if self._shutdown_sent:
                return
            self._shutdown_sent = True
            if self._shutdown is not None:
                self._shutdown()
        elif self._control is not None:
            if name == "space":
                current = (self._state() if self._state is not None else "IDLE").upper()
                command = "PAUSE" if current == "RUNNING" else "RESUME" if current == "PAUSED" else "START"
                self._control(command)
            elif name == "r":
                self._control("REALIGN")
            elif name == "n":
                self._control("RESET")
    handle_key = on_key
    def update_hud(self, values: Mapping[str, Any]) -> None:
        self.hud = dict(values)
        if self._window is None:
            return
        set_texts = getattr(self._window, "set_texts", None)
        if set_texts is not None:
            try:
                import mujoco
            except ImportError:  # pragma: no cover - visible mode requires MuJoCo
                return
            text = "\n".join(f"{key}: {value}" for key, value in self.hud.items())
            set_texts(
                (
                    mujoco.mjtFontScale.mjFONTSCALE_150,
                    mujoco.mjtGridPos.mjGRID_TOPLEFT,
                    "SPD VR",
                    text,
                )
            )
            return
        update = getattr(self._window, "update_hud", None)
        if update is not None:
            update(self.hud)
    def sync(self, now_ns: int | None = None) -> None:
        del now_ns
        if self._closed or self._window is None:
            return
        lock = getattr(self._window, "lock", None)
        if callable(lock):
            with lock():
                self._update_pose_markers()
        else:
            self._update_pose_markers()
        sync = getattr(self._window, "sync", None)
        if sync is not None:
            sync()

    def _update_pose_markers(self) -> None:
        scene = getattr(self._window, "user_scn", None)
        if scene is None or (self._pose_markers is None and self._hand_keypoints is None):
            return
        import mujoco

        scene.ngeom = 0
        colors = (
            np.array((1.0, 0.1, 0.1, 1.0), dtype=np.float32),
            np.array((0.1, 1.0, 0.1, 1.0), dtype=np.float32),
            np.array((0.1, 0.3, 1.0, 1.0), dtype=np.float32),
        )
        if self._pose_markers is not None:
            poses = self._pose_markers()
            for side in ("left", "right"):
                pose = np.asarray(poses.get(side, ()), dtype=float)
                if pose.shape != (4, 4) or not np.all(np.isfinite(pose)):
                    continue
                origin = pose[:3, 3]
                for axis, color in enumerate(colors):
                    if scene.ngeom >= len(scene.geoms):
                        return
                    geom = scene.geoms[scene.ngeom]
                    mujoco.mjv_initGeom(
                        geom,
                        mujoco.mjtGeom.mjGEOM_ARROW,
                        np.zeros(3),
                        origin,
                        np.eye(3).ravel(),
                        color,
                    )
                    mujoco.mjv_connector(
                        geom,
                        mujoco.mjtGeom.mjGEOM_ARROW,
                        0.004,
                        origin,
                        origin + 0.08 * pose[:3, axis],
                    )
                    scene.ngeom += 1

        if self._hand_keypoints is None:
            return
        point_colors = {
            "left": np.array((1.0, 0.50, 0.05, 0.95), dtype=np.float32),
            "right": np.array((0.05, 0.75, 1.0, 0.95), dtype=np.float32),
        }
        line_colors = {
            "left": np.array((1.0, 0.65, 0.15, 0.75), dtype=np.float32),
            "right": np.array((0.15, 0.85, 1.0, 0.75), dtype=np.float32),
        }
        keypoints = self._hand_keypoints()
        for side in ("left", "right"):
            points = np.asarray(keypoints.get(side, ()), dtype=float)
            if points.shape != (21, 3) or not np.all(np.isfinite(points)):
                continue
            for start_index, end_index in MEDIAPIPE_CONNECTIONS:
                if scene.ngeom >= len(scene.geoms):
                    return
                geom = scene.geoms[scene.ngeom]
                mujoco.mjv_initGeom(
                    geom,
                    mujoco.mjtGeom.mjGEOM_CAPSULE,
                    np.zeros(3),
                    np.zeros(3),
                    np.eye(3).ravel(),
                    line_colors[side],
                )
                mujoco.mjv_connector(
                    geom,
                    mujoco.mjtGeom.mjGEOM_CAPSULE,
                    0.002,
                    points[start_index],
                    points[end_index],
                )
                scene.ngeom += 1
            for point in points:
                if scene.ngeom >= len(scene.geoms):
                    return
                mujoco.mjv_initGeom(
                    scene.geoms[scene.ngeom],
                    mujoco.mjtGeom.mjGEOM_SPHERE,
                    np.array((0.004, 0.0, 0.0)),
                    point,
                    np.eye(3).ravel(),
                    point_colors[side],
                )
                scene.ngeom += 1
    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        window, self._window = self._window, None
        if window is not None:
            close = getattr(window, "close", None)
            if close is not None:
                close()

    def __enter__(self) -> "ViewerWindow":
        return self.open()

    def __exit__(self, *_: Any) -> None:
        self.close()


# Name used by a few callers that describe the object by behavior.
PassiveViewerWindow = ViewerWindow


__all__ = ["MEDIAPIPE_CONNECTIONS", "PassiveViewerWindow", "ViewerWindow"]
