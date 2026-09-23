"""Small adapter around MuJoCo's passive viewer.

``PlantController`` owns the live model and data. This adapter may allocate
private kinematics scratch state, but never advances the simulation.
"""

from __future__ import annotations
from collections import deque
import threading

from typing import Any, Callable, Mapping, Sequence

import numpy as np

from interfaces.keyboard_control import KEY_COMMANDS


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
        self._task_text: tuple[str, str] | None = None
        self._task_header_key: tuple[Any, ...] | None = None
        self._task_header_height = 0
        self._task_fonts: dict[int, Any] = {}
        self._hand_ghost_target: np.ndarray | None = None
        self._hand_ghost: Any | None = None
        self._hand_ghost_dirty = False
        self._hand_ghost_label = ""
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
        try:
            self._sync_task_header()
            self._sync_hand_ghost()
        except Exception:
            self.close()
            raise
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
            if 0 <= key < 128:
                return chr(key).lower()
        return str(key).strip().lower().removeprefix("key_")

    def on_key(self, key: Any) -> None:
        """Handle one MuJoCo/GLFW key event without repeated shutdown calls."""
        name = self._key_name(key)
        if name in {"q", "escape", "esc"}:
            if self._shutdown_sent:
                return
            self._shutdown_sent = True
            if self._joint_control is not None:
                self._joint_control("q")
            elif self._shutdown is not None:
                self._shutdown()
        elif self._joint_control is not None and name in {"f8", "f9"}:
            self._joint_control(name)
        elif self._recording_control is not None:
            operation = KEY_COMMANDS.get(name)
            if operation is not None:
                self._recording_control(operation)

    def set_task(self, title_zh: str, goal_zh: str) -> None:
        """Set the Chinese episode header, including before opening the viewer.

        Headless windows only retain the strings; they never import Pillow or
        load fonts. Visible windows require Debian/Ubuntu's fonts-noto-cjk.
        """
        self._task_text = (title_zh, goal_zh)
        self._sync_task_header()

    def _task_font(self, size: int) -> Any:
        if size not in self._task_fonts:
            from PIL import ImageFont

            path = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
            try:
                # Noto's TTC order is JP, KR, SC, TC, HK; choose Simplified Chinese.
                self._task_fonts[size] = ImageFont.truetype(path, size, index=2)
            except OSError as exc:
                raise RuntimeError(
                    "Chinese task headers require fonts-noto-cjk: install it with "
                    "'sudo apt install fonts-noto-cjk'; expected font at " + path
                ) from exc
        return self._task_fonts[size]

    @staticmethod
    def _wrap_task_text(text: str, font: Any, width: int) -> list[str]:
        """Wrap by glyph, including Chinese text without whitespace."""
        lines: list[str] = []
        for paragraph in text.split("\n"):
            line = ""
            for character in paragraph:
                candidate = line + character
                if line and font.getlength(candidate) > width:
                    lines.append(line)
                    line = character
                else:
                    line = candidate
            lines.append(line)
        return lines

    def _sync_task_header(self) -> None:
        if self.headless or self._closed or self._window is None or self._task_text is None:
            return
        if not callable(getattr(self._window, "set_images", None)):
            raise RuntimeError("Chinese task headers require MuJoCo's viewer Handle.set_images API")
        viewport = self._window.viewport
        if viewport is None or viewport.width <= 0 or viewport.height <= 0:
            return
        bounds = (viewport.left, viewport.bottom, viewport.width, viewport.height)
        key = (*self._task_text, self._hand_ghost_label, *bounds)
        if key == self._task_header_key:
            return
        from PIL import Image, ImageDraw
        import mujoco

        width, available_height = int(viewport.width), int(viewport.height)
        padding = min(12, max(0, (width - 1) // 4))
        content_width = max(1, width - 2 * padding)
        title, goal = self._task_text
        # Keep the normal header readable; shrink only for unusually small windows.
        size = min(24, content_width)
        while True:
            title_font = self._task_font(size)
            goal_font = self._task_font(max(1, size - 4))
            title_lines = self._wrap_task_text("任务：" + title, title_font, content_width)
            goal_lines = self._wrap_task_text("目标：" + goal, goal_font, content_width)
            if self._hand_ghost_label:
                goal_lines.extend(self._wrap_task_text("虚影：" + self._hand_ghost_label, goal_font, content_width))
            title_step = sum(title_font.getmetrics()) + 2
            goal_step = sum(goal_font.getmetrics()) + 2
            height = 2 * padding + len(title_lines) * title_step + len(goal_lines) * goal_step
            if height <= available_height or size == 1:
                break
            size -= 1
        image = Image.new("RGB", (width, height), (18, 24, 33))
        draw = ImageDraw.Draw(image)
        y = padding
        for lines, font, step, color in (
            (title_lines, title_font, title_step, (255, 221, 138)),
            (goal_lines, goal_font, goal_step, (240, 244, 250)),
        ):
            for line in lines:
                draw.text((padding, y), line, font=font, fill=color, anchor="lt")
                y += step
        # Handle.set_images flips top-down Pillow RGB rows for OpenGL itself.
        self._window.set_images((
            mujoco.MjrRect(viewport.left, viewport.bottom + available_height - height, width, height),
            np.asarray(image),
        ))
        self._task_header_key = key
        self._task_header_height = height
        if self.hud:
            self.update_hud(self.hud)

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
            # Reserve the task strip in both text columns. A conservative row
            # height also leaves room when the viewer uses smaller UI fonts.
            spacer = "\n" * ((self._task_header_height + 14) // 15)
            set_texts(
                (
                    mujoco.mjtFontScale.mjFONTSCALE_150,
                    mujoco.mjtGridPos.mjGRID_TOPLEFT,
                    spacer + "SPD Simulation",
                    spacer + text,
                )
            )
            return
        update = getattr(self._window, "update_hud", None)
        if update is not None:
            update(self.hud)

    def set_hand_ghost(self, position_rad: Sequence[float] | None, *, label: str = "") -> None:
        """Draw only the Wuji2 hands at a canonical 54-joint target.

        The target is retained independently of the caller's array. Headless
        windows retain only this small pose; kinematics and render geometry are
        allocated lazily when a passive viewer can actually display them.
        """
        if self._closed:
            return
        self._hand_ghost_label = label if position_rad is not None else ""
        self._sync_task_header()
        if position_rad is None:
            if self._hand_ghost_target is None:
                return
            self._hand_ghost_target = None
        else:
            target = np.asarray(position_rad, dtype=np.float64)
            if target.shape != (54,) or not np.all(np.isfinite(target)):
                raise ValueError("hand ghost target must contain 54 finite joint positions")
            if self._hand_ghost_target is not None:
                if np.array_equal(target, self._hand_ghost_target):
                    return
                self._hand_ghost_target[:] = target
            else:
                self._hand_ghost_target = target.copy()
        self._hand_ghost_dirty = True
        self._sync_hand_ghost()

    def _sync_hand_ghost(self) -> None:
        if self.headless or self._closed or self._window is None:
            return
        scene = getattr(self._window, "user_scn", None)
        if scene is None:
            return
        if self._hand_ghost is not None and (
            self._hand_ghost.model is not self.model
            or self._hand_ghost.data is not self.data
        ):
            # Never retain a checkpoint overlay from a previous scene.
            self._hand_ghost = None
            self._hand_ghost_target = None
            self._hand_ghost_dirty = True
        if not self._hand_ghost_dirty:
            return
        target = self._hand_ghost_target
        if target is not None and self._hand_ghost is None:
            if self.model is None or self.data is None:
                raise ValueError("hand ghosts require a model and data")
            from simulation.hand_ghost import HandGhost

            self._hand_ghost = HandGhost(self.model, self.data)
        with self._window.lock():
            if target is None:
                scene.ngeom = 0
            else:
                self._hand_ghost.draw(scene, target)
        self._hand_ghost_dirty = False

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
        self._sync_task_header()
        self._sync_hand_ghost()
        sync = getattr(self._window, "sync", None)
        if sync is not None:
            sync()

    def close(self) -> None:
        if self._closed:
            return
        self.set_hand_ghost(None)
        self._hand_ghost = None
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
