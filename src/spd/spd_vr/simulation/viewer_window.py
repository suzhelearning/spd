"""Small adapter around MuJoCo's passive viewer.

The adapter deliberately owns no model or data.  ``PlantController`` remains the
single owner of the complete simulation state, while this class only renders it.
"""

from __future__ import annotations
from collections import deque
import threading

from typing import Any, Callable, Mapping

import numpy as np


class ViewerWindow:
    def __init__(
        self,
        model: Any | None = None,
        data: Any | None = None,
        *,
        headless: bool = False,
        shutdown: Callable[[], None] | None = None,
        recording_control: Callable[[str], None] | None = None,
        joint_control: Callable[[str], None] | None = None,
        window: Any | None = None,
    ) -> None:
        self.model = model
        self.data = data
        self.headless = bool(headless)
        self._shutdown = shutdown
        self._recording_control = recording_control
        self._joint_control = joint_control
        self._window = window
        self._render_thread: threading.Thread | None = None
        self._closed = False
        self._shutdown_sent = False
        self.hud: dict[str, Any] = {}
        self._joint_figure: Any | None = None
        self._joint_history: deque[tuple[float, float, float]] = deque(maxlen=300)
        self._plot_joint = ""
        if model is not None:
            arm_bodies = {
                f"{link}_{side}"
                for side in ("L", "R")
                for link in ("Base", *(f"Link{i}" for i in range(1, 8)), "TCP_Link")
            }
            for geom_id in range(model.ngeom):
                body_name = model.body(int(model.geom_bodyid[geom_id])).name
                # Group 1 contains visual meshes; collision geometry is untouched.
                if (model.geom_group[geom_id] == 1
                        and (body_name in arm_bodies or body_name.startswith(("l_", "r_")))):
                    model.geom_rgba[geom_id, 3] = 0.45

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
        existing_threads = set(threading.enumerate())
        self._window = mujoco_viewer.launch_passive(
            self.model,
            self.data,
            key_callback=self.on_key,
            show_left_ui=False,
            show_right_ui=False,
        )
        # MuJoCo 3.12 starts a daemon render thread but Handle.close() only
        # requests exit. Retain that specific thread so GLFW/C++ teardown
        # finishes before Python exits; otherwise finite viewers can abort.
        self._render_thread = next((
            thread for thread in threading.enumerate()
            if thread not in existing_threads
            and getattr(thread, "_target", None) is mujoco_viewer._launch_internal
        ), None)
        return self
    @staticmethod
    def _key_name(key: Any) -> str:
        if isinstance(key, str):
            return key.strip().lower().removeprefix("key_")
        if isinstance(key, int):
            if key in (27, 256):
                return "escape"
            if key in (297, 298):
                return "f8" if key == 297 else "f9"
            if key in (ord("e"), ord("E"), ord("c"), ord("C"), ord("q"), ord("Q")):
                return chr(key).lower()
            if key in (ord("r"), ord("R")):
                return "r"
            if key in (ord("s"), ord("S")):
                return "s"
            if key in (ord("d"), ord("D")):
                return "d"
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
        elif self._joint_control is not None and name in {"e", "c", "f8", "f9"}:
            self._joint_control(name)
        elif name == "r":
            if self._recording_control is not None:
                self._recording_control("start")
        elif name == "s":
            if self._recording_control is not None:
                self._recording_control("success")
        elif name == "d":
            if self._recording_control is not None:
                self._recording_control("discard")
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
                    "SPD Simulation",
                    text,
                )
            )
            return
        update = getattr(self._window, "update_hud", None)
        if update is not None:
            update(self.hud)

    def update_joint_plot(self, name: str, seconds: float, target: float, actual: float) -> None:
        """Plot an applied target against measured simulation position, in radians."""
        if self._window is None or not callable(getattr(self._window, "set_figures", None)):
            return
        import mujoco

        if self._joint_figure is None:
            self._joint_figure = mujoco.MjvFigure()
            self._joint_figure.xlabel = "Host elapsed time (s)"
            self._joint_figure.flg_legend = 1
            self._joint_figure.linename[0] = b"Applied target (rad)"
            self._joint_figure.flg_extend = 0
            self._joint_figure.linename[1] = b"Actual qpos (rad)"
            self._joint_figure.linergb[0] = (1.0, 0.65, 0.15)
            self._joint_figure.linergb[1] = (0.15, 0.8, 1.0)
        if name != self._plot_joint:
            self._joint_history.clear()
            self._plot_joint = name
        self._joint_history.append((seconds, target, actual))
        figure = self._joint_figure
        figure.title = name
        history = np.asarray(self._joint_history)
        figure.range[0] = (history[0, 0], max(history[-1, 0], history[0, 0] + 0.1))
        low, high = float(np.min(history[:, 1:])), float(np.max(history[:, 1:]))
        margin = max(0.02, (high - low) * 0.1)
        figure.range[1] = (low - margin, high + margin)
        count = len(history)
        for index in range(2):
            figure.linepnt[index] = count
            figure.linedata[index, :2 * count:2] = history[:, 0]
            figure.linedata[index, 1:2 * count:2] = history[:, index + 1]
        self._window.set_figures((mujoco.MjrRect(0, 0, 560, 220), figure))
    def sync(self, now_ns: int | None = None) -> None:
        del now_ns
        if self._closed or self._window is None:
            return
        sync = getattr(self._window, "sync", None)
        if sync is not None:
            sync()

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        window, self._window = self._window, None
        if window is not None:
            close = getattr(window, "close", None)
            if close is not None:
                close()
        if self._render_thread is not None:
            self._render_thread.join()
            self._render_thread = None

    def __enter__(self) -> "ViewerWindow":
        return self.open()

    def __exit__(self, *_: Any) -> None:
        self.close()


__all__ = ["ViewerWindow"]
