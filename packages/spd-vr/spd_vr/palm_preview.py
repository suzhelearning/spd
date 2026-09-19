"""Read-only Tk projection of measured and requested robot palm frames.

``render_palm_preview(canvas, core.status())`` needs no ROS or running viewer.
All poses and errors come from the core; absent measurements are never guessed.
"""
from __future__ import annotations

import math
from typing import Any


_AXIS_COLORS = ("#c63838", "#16834b", "#245cd0")
_VIEWS = (
    ("REAR  (looking toward +X)", 1, -1, 2, "left +Y", "up +Z"),
    ("SIDE  (looking toward +Y)", 0, 1, 2, "forward +X", "up +Z"),
    ("TOP  (looking toward -Z)", 1, -1, 0, "left +Y", "forward +X"),
)


def _pose(value: Any) -> list[list[float]] | None:
    if not isinstance(value, (list, tuple)) or len(value) != 4:
        return None
    try:
        pose = [[float(item) for item in row] for row in value]
    except (TypeError, ValueError):
        return None
    if any(len(row) != 4 for row in pose) or not all(math.isfinite(item) for row in pose for item in row):
        return None
    return pose


def _error(value: Any, factor: float, unit: str) -> str:
    if isinstance(value, (int, float)) and math.isfinite(value):
        return f"{value * factor:.1f} {unit}"
    return "unavailable"


def render_palm_preview(canvas: Any, status: dict[str, Any]) -> None:
    """Draw three metric orthographic views without changing control state.

    Physical axes: robot +X forward, +Y left, +Z up. Palm frame axes:
    +X fingers, +Y palm-left, +Z dorsal. Solid=T target; dashed=A actual;
    dotted=S solver. Every view uses the same scale, with 0.20 m grid spacing.
    Call after Tk layout (``root.update_idletasks()``) and on each new status.
    """
    canvas.delete("all")
    width, height = max(canvas.winfo_width(), 600), max(canvas.winfo_height(), 260)
    font = ("sans", 10)
    canvas.create_text(12, 12, anchor="nw", font=("sans", 11, "bold"), fill="#172435",
                       text="PALM PREVIEW ONLY — K calibrate / C align / F authorizes follow")
    canvas.create_text(12, 34, anchor="nw", font=font, fill="#28384b",
                       text="T target: solid   A actual: dashed   S solver: dotted   |   L/R hands   |   "
                            "axes: X red (fingers), Y green (left), Z blue (dorsal)")
    preview = status.get("arm_preview") or {}
    frames = []
    for side in ("left", "right"):
        entry = preview.get(side) or {}
        for key, letter, dash in (("target_palm", "T", ()), ("actual_palm", "A", (7, 4)),
                                  ("solver_palm", "S", (2, 4))):
            pose = _pose(entry.get(key))
            if pose is not None:
                frames.append((side[0].upper(), letter, dash, pose))
    # Always show robot origin and metric grid, but never fabricate a palm frame.
    coordinates = [[0.0, 0.0, 0.0]]
    axis_length = 0.10
    for _, _, _, pose in frames:
        origin = [pose[j][3] for j in range(3)]
        coordinates.append(origin)
        coordinates.extend([[origin[j] + axis_length * pose[j][axis] for j in range(3)] for axis in range(3)])
    low = [min(point[j] for point in coordinates) - 0.16 for j in range(3)]
    high = [max(point[j] for point in coordinates) + 0.16 for j in range(3)]
    panel_width = width / 3
    top, bottom = 88, height - 40
    plot_height = max(bottom - top, 100)
    scale = min(min((panel_width - 58) / (high[h] - low[h]), plot_height / (high[v] - low[v]))
                for _, h, _, v, _, _ in _VIEWS)
    for panel, (title, horizontal, sign, vertical, h_label, v_label) in enumerate(_VIEWS):
        left = panel * panel_width
        center_x, center_y = left + panel_width / 2, (top + bottom) / 2
        mid_h = (low[horizontal] + high[horizontal]) / 2
        mid_v = (low[vertical] + high[vertical]) / 2

        def project(point: list[float]) -> tuple[float, float]:
            return (center_x + sign * (point[horizontal] - mid_h) * scale,
                    center_y - (point[vertical] - mid_v) * scale)

        canvas.create_rectangle(left + 4, 61, left + panel_width - 4, height - 5,
                                outline="#bbc7d4", fill="#f8fafc")
        canvas.create_text(center_x, 73, text=title, font=("sans", 10, "bold"), fill="#172435")
        for dimension in (horizontal, vertical):
            for step in range(math.ceil(low[dimension] / 0.2), math.floor(high[dimension] / 0.2) + 1):
                value = step * 0.2
                start, end = list(low), list(high)
                start[dimension] = end[dimension] = value
                x1, y1 = project(start)
                x2, y2 = project(end)
                canvas.create_line(x1, y1, x2, y2, fill="#8e9eb0" if step == 0 else "#e0e6ed")
                canvas.create_text(x1 + 3, y1 - 3, anchor="sw", text=f"{value:.1f}",
                                   font=("sans", 8), fill="#68778a")
        origin_x, origin_y = project([0, 0, 0])
        canvas.create_text(origin_x + 3, origin_y + 3, anchor="nw", text="O", font=font, fill="#68778a")
        for hand, letter, dash, pose in frames:
            point = [pose[j][3] for j in range(3)]
            x, y = project(point)
            for axis, color in enumerate(_AXIS_COLORS):
                endpoint = [point[j] + axis_length * pose[j][axis] for j in range(3)]
                ex, ey = project(endpoint)
                canvas.create_line(x, y, ex, ey, fill=color, width=2, dash=dash, arrow="last")
            if letter == "A":
                canvas.create_oval(x - 4, y - 4, x + 4, y + 4, outline="#172435", width=2)
            elif letter == "T":
                canvas.create_rectangle(x - 3, y - 3, x + 3, y + 3, outline="#172435", width=2)
            offset = {"T": -15, "A": 2, "S": 16}[letter]
            canvas.create_text(x + 7, y + offset, anchor="nw", text=f"{hand}-{letter}", font=font, fill="#172435")
        canvas.create_text(center_x, height - 23, text=f"{h_label} / {v_label}   [m; grid 0.20 m]",
                           font=font, fill="#28384b")
        if not frames:
            canvas.create_text(center_x, center_y, text="No valid palm poses\nWaiting for calibration / fresh feedback",
                               font=font, fill="#9c451d", justify="center")


def palm_preview_text(status: dict[str, Any]) -> str:
    """Readable freshness, actual tracking errors, and solver diagnostics."""
    lines = []
    preview = status.get("arm_preview") or {}
    diagnostics = status.get("arm_solver") or {}
    for side in ("left", "right"):
        entry = preview.get(side) or {}
        errors = (f"actual error: {_error(entry.get('position_error_m'), 1000, 'mm')} / "
                  f"{_error(entry.get('orientation_error_rad'), 180 / math.pi, 'deg')}")
        available = "/".join(label for key, label in (("target_palm", "target"), ("actual_palm", "actual"),
                                                      ("solver_palm", "solver")) if _pose(entry.get(key)) is not None)
        lines.append(f"{side.upper()}: {entry.get('state', 'unavailable')} | {errors} | "
                     f"poses: {available or 'none'} | {entry.get('detail') or 'No current diagnostic'}")
        solver = diagnostics.get(side) or {}
        lines.append(
            f"{side.upper()} IK (not measured): {solver.get('state', 'unavailable')} | residual: "
            f"{_error(solver.get('position_error_m'), 1000, 'mm')} / "
            f"{_error(solver.get('orientation_error_rad'), 180 / math.pi, 'deg')} | "
            f"near pair: {_error(solver.get('collision_distance_m'), 1000, 'mm')} | "
            f"{solver.get('detail') or 'No current solver diagnostic'}"
        )
    return "\n".join(lines)
