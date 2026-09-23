"""Presentation and controls for the native MuJoCo renderer.

The physics owner submits state through this adapter. Only the renderer thread
loads fonts or draws pixels; headless use retains presentation data only.
"""

from __future__ import annotations

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
        split_view: bool = False,
    ) -> None:
        self.model = model
        self.data = data
        self.headless = bool(headless)
        self.split_view = bool(split_view)
        self._shutdown = shutdown
        self._recording_control = recording_control
        self._joint_control = joint_control
        self._renderer: Any | None = None
        self._closed = False
        self._shutdown_sent = False
        self.hud: dict[str, Any] = {}
        self._task_text: tuple[str, str] | None = None
        self._task_header_key: tuple[Any, ...] | None = None
        self._task_header_pixels: np.ndarray | None = None
        self._task_fonts: dict[int, Any] = {}
        self._hand_ghost_target: np.ndarray | None = None
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
    def closed(self) -> bool:
        return self._closed

    def is_running(self) -> bool:
        if self._closed:
            return False
        return self._renderer.is_running() if self._renderer is not None else True

    @property
    def shutdown_sent(self) -> bool:
        return self._shutdown_sent

    def open(self) -> "ViewerWindow":
        if self._renderer is not None or self.headless or self._closed:
            return self
        if self.model is None or self.data is None:
            raise ValueError("a model and data are required for a visible viewer")
        from simulation.split_view import SplitViewRenderer

        self._renderer = SplitViewRenderer(
            self.model,
            split_view=self.split_view,
            render_header=self._render_header,
            on_key=self.on_key,
        )
        try:
            self._renderer.open(
                self.data, self._task_text, self._hud_snapshot(),
                self._hand_ghost_target, self._hand_ghost_label,
            )
        except Exception:
            self.close()
            raise
        return self

    def frame(self, table_near_edge_m: float | None) -> None:
        """Frame the free camera without exposing renderer-owned MuJoCo state."""
        if not self._closed and self._renderer is not None:
            self._renderer.frame(table_near_edge_m)

    @staticmethod
    def _key_name(key: Any) -> str:
        if isinstance(key, str):
            if key == " ":
                return " "
            name = key.strip().lower().removeprefix("key_")
            return " " if name == "space" else name
        if isinstance(key, int):
            if key in (27, 256):
                return "escape"
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
        elif self._recording_control is not None:
            operation = KEY_COMMANDS.get(name)
            if operation is not None:
                self._recording_control(operation)

    def set_task(self, title_zh: str, goal_zh: str) -> None:
        """Retain the Chinese episode header without performing rendering."""
        self._task_text = (str(title_zh), str(goal_zh))

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

    def _render_header(
        self,
        width: int,
        height: int,
        task: tuple[str, str] | None,
        hud: tuple[tuple[str, str], ...],
        label: str,
    ) -> np.ndarray:
        """Render a cached, top-down RGB header from an immutable snapshot.

        Called exclusively by the renderer thread. ``height`` is the full
        available window height, not the requested header height.
        """
        width, height = max(1, int(width)), max(1, int(height))
        key = (width, height, task, hud, label, self.split_view)
        if key == self._task_header_key and self._task_header_pixels is not None:
            return self._task_header_pixels
        if height == 1:
            self._task_header_pixels = np.empty((0, width, 3), dtype=np.uint8)
            self._task_header_key = key
            return self._task_header_pixels
        from PIL import Image, ImageDraw

        background = (18, 24, 33)
        foreground = (240, 244, 250)
        # Never let lengthy status/error text consume the 3D viewports.
        limit = max(1, height * 2 // 5)
        padding = min(12, max(0, (width - 1) // 4), max(0, (limit - 1) // 4))
        content_width = max(1, width - 2 * padding)
        gap = min(24, content_width // 12)
        task_width = max(1, int((content_width - gap) * .45))
        status_width = max(1, content_width - task_width - gap)
        status_x = padding + task_width + gap
        label_font = self._task_font(16) if self.split_view else None
        label_height = min(limit, sum(label_font.getmetrics()) + 10) if label_font else 0
        size = min(24, task_width, status_width)
        while True:
            title_font = self._task_font(size)
            goal_font = self._task_font(max(1, size - 4))
            title_step = sum(title_font.getmetrics()) + 2
            goal_step = sum(goal_font.getmetrics()) + 2
            title_lines = self._wrap_task_text("任务：" + task[0], title_font, task_width) if task else []
            goal_lines = self._wrap_task_text("目标：" + task[1], goal_font, task_width) if task else []
            if label:
                goal_lines.extend(self._wrap_task_text("虚影：" + label, goal_font, task_width))
            task_rows = [
                (line, title_font, title_step, (255, 221, 138)) for line in title_lines
            ] + [(line, goal_font, goal_step, foreground) for line in goal_lines]
            status_rows = [
                (line, goal_font, goal_step, (255, 160, 140) if name == "异常" else foreground)
                for name, value in hud
                for line in self._wrap_task_text(f"{name}：{value}", goal_font, status_width)
            ]
            content_height = max(
                sum(row[2] for row in task_rows), sum(row[2] for row in status_rows),
            )
            header_height = max(1, 2 * padding + content_height + label_height)
            if header_height <= limit or size <= 12:
                break
            size -= 1
        header_height = min(limit, header_height)
        content_height = max(0, header_height - 2 * padding - label_height)
        image = Image.new("RGB", (width, header_height), background)
        draw = ImageDraw.Draw(image)
        for x, column_width, rows in (
            (padding, task_width, task_rows), (status_x, status_width, status_rows),
        ):
            if content_height == 0:
                continue
            # Separate images guarantee even unusually wide glyphs cannot bleed
            # into the other column. Oversized messages end in a visible ellipsis.
            column = Image.new("RGB", (column_width, content_height), background)
            column_draw = ImageDraw.Draw(column)
            y = 0
            for index, (text, font, step, color) in enumerate(rows):
                if y + step > content_height:
                    break
                if index + 1 < len(rows) and y + step + rows[index + 1][2] > content_height:
                    while text and font.getlength(text + "…") > column_width:
                        text = text[:-1]
                    text += "…"
                column_draw.text((0, y), text, font=font, fill=color, anchor="lt")
                y += step
            image.paste(column, (x, padding))
        if content_height:
            divider_x = padding + task_width + gap // 2
            draw.line(
                (divider_x, padding, divider_x, padding + content_height), fill=(64, 76, 91),
            )
        if label_font is not None:
            label_y = header_height - label_height
            draw.line((0, label_y, width, label_y), fill=(64, 76, 91))
            half = width // 2
            for x, label_width, text in (
                (0, half, "左侧：自由视角（拖动旋转／平移，滚轮缩放）"),
                (half, width - half, "右侧：固定头部视角"),
            ):
                if label_width <= 0:
                    continue
                row = Image.new("RGB", (label_width, max(1, label_height - 1)), background)
                ImageDraw.Draw(row).text((padding, 4), text, font=label_font, fill=(172, 192, 214), anchor="lt")
                image.paste(row, (x, label_y + 1))
        self._task_header_pixels = np.asarray(image)
        self._task_header_key = key
        return self._task_header_pixels

    def update_hud(self, values: Mapping[str, Any]) -> None:
        self.hud = dict(values)

    def _hud_snapshot(self) -> tuple[tuple[str, str], ...]:
        return tuple((str(name), str(value)) for name, value in self.hud.items())

    def set_hand_ghost(self, position_rad: Sequence[float] | None, *, label: str = "") -> None:
        """Retain an owned canonical 54-joint pose; rendering is asynchronous."""
        if self._closed:
            return
        if position_rad is None:
            self._hand_ghost_target = None
            self._hand_ghost_label = ""
            return
        target = np.asarray(position_rad, dtype=np.float64)
        if target.shape != (54,) or not np.all(np.isfinite(target)):
            raise ValueError("hand ghost target must contain 54 finite joint positions")
        if self._hand_ghost_target is None:
            self._hand_ghost_target = target.copy()
        else:
            self._hand_ghost_target[:] = target
        self._hand_ghost_label = str(label)

    def sync(self, now_ns: int | None = None) -> None:
        del now_ns
        if self._closed or self._renderer is None:
            return
        self._renderer.submit(
            self.data, self._task_text, self._hud_snapshot(),
            self._hand_ghost_target, self._hand_ghost_label,
        )

    def close(self) -> None:
        if self._closed:
            return
        try:
            if self._renderer is not None:
                self._renderer.close()
        finally:
            self._closed = True
            self._renderer = None
            self._hand_ghost_target = None
            self._hand_ghost_label = ""
            self._task_header_key = None
            self._task_header_pixels = None
            self._task_fonts.clear()

    def __enter__(self) -> "ViewerWindow":
        return self.open()

    def __exit__(self, *_: Any) -> None:
        self.close()


__all__ = ["ViewerWindow"]
