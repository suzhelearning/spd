"""One GLFW window, with a private render-only copy of the simulation."""
from __future__ import annotations

import copy
import math
import threading
import time
from collections.abc import Callable, Mapping, Sequence
from typing import Any

import numpy as np


_NO_FRAME = object()


def _viewport_widths(width: int, split_view: bool) -> tuple[int, int, int]:
    """Share free-view, separator and fixed-view widths with the header."""
    if not split_view:
        return width, 0, 0
    separator = min(2, width)
    left_width = (width - separator) // 3
    return left_width, separator, width - separator - left_width


class SplitViewRenderer:
    """The physics owner submits snapshots; only the worker touches GLFW/GL.

    The mailbox lock protects pointer exchanges, never state capture, camera
    calculations, callbacks, or rendering. A slow display drops intermediate
    snapshots instead of slowing down the physics owner.
    """

    def __init__(
        self,
        model: Any,
        *,
        split_view: bool,
        render_header: Callable[..., np.ndarray],
        on_key: Callable[[str], None],
    ) -> None:
        import mujoco

        self._mujoco = mujoco
        self._source_model = model
        self._split_view = split_view
        self._render_header = render_header
        self._on_key = on_key
        self._state_spec = mujoco.mjtState.mjSTATE_INTEGRATION
        self._state_size = mujoco.mj_stateSize(model, self._state_spec)
        self._lock = threading.Lock()
        self._pending: tuple[Any, ...] | None = None
        self._pending_frame: object = None
        self._stop = threading.Event()
        self._ready = threading.Event()
        self._running = threading.Event()
        self._thread: threading.Thread | None = None
        self._error: BaseException | None = None

    def _raise_if_failed(self) -> None:
        if self._error is not None:
            raise RuntimeError("native viewer rendering failed") from self._error

    def open(
        self, data: Any, task: tuple[str, str] | None,
        hud: Mapping[str, str] | Sequence[tuple[str, str]],
        ghost: Sequence[float] | None, label: str,
    ) -> None:
        """Clone on the owner thread, then wait at most 15 seconds for startup."""
        self._raise_if_failed()
        if self._thread is not None:
            return
        if self._stop.is_set():
            raise RuntimeError("a closed native viewer cannot be reopened")
        model = copy.copy(self._source_model)
        # Operator visibility is presentation-only, never baked into trajectory models.
        arm_bodies = {f"{link}_{side}" for side in ("L", "R")
                      for link in ("Base", *(f"Link{i}" for i in range(1, 8)), "TCP_Link")}
        for geom_id in range(model.ngeom):
            if model.geom_group[geom_id] == 1 and model.body(int(model.geom_bodyid[geom_id])).name in arm_bodies:
                model.geom_rgba[geom_id, 3] = .45
        self.submit(data, task, hud, ghost, label)
        self._thread = threading.Thread(
            target=self._run, args=(model,), name="spd-split-view", daemon=True,
        )
        self._thread.start()
        if not self._ready.wait(15.0):
            self._stop.set()
            raise TimeoutError("native viewer did not finish startup within 15 seconds")
        self._raise_if_failed()

    def submit(
        self, data: Any, task: tuple[str, str] | None,
        hud: Mapping[str, str] | Sequence[tuple[str, str]],
        ghost: Sequence[float] | None, label: str,
    ) -> None:
        """Capture owned values on the physics owner; replace the sole packet."""
        self._raise_if_failed()
        if self._stop.is_set():
            return
        state = np.empty(self._state_size, dtype=np.float64)
        self._mujoco.mj_getState(self._source_model, data, state, self._state_spec)
        target = None if ghost is None else np.array(ghost, dtype=np.float64, copy=True)
        packet = (
            state, None if task is None else tuple(str(value) for value in task),
            tuple((str(key), str(value)) for key, value in (
                hud.items() if isinstance(hud, Mapping) else hud
            )),
            target, str(label),
        )
        with self._lock:
            self._pending = packet

    def frame(self, table_near_edge_m: float | None) -> None:
        """Queue free-camera framing; None uses the robot visual bounds."""
        self._raise_if_failed()
        with self._lock:
            self._pending_frame = table_near_edge_m

    def is_running(self) -> bool:
        self._raise_if_failed()
        return self._running.is_set() and not self._stop.is_set()

    def close(self) -> None:
        """Join worker teardown before another scene can create its window."""
        self._stop.set()
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=15.0)
            if thread.is_alive():
                raise TimeoutError("native viewer did not finish closing within 15 seconds")
        with self._lock:
            self._pending = None
        self._raise_if_failed()

    def _run(self, model: Any) -> None:
        try:
            self._render_loop(model)
        except BaseException as exc:
            # Do not retain a traceback containing private model/GL resources.
            if self._error is None:
                self._error = exc.with_traceback(None)
        finally:
            self._running.clear()
            self._stop.set()
            self._ready.set()

    def _render_loop(self, model: Any) -> None:
        import glfw

        mj = self._mujoco
        window = None
        context = None
        initialized = False
        try:
            if not glfw.init():
                raise RuntimeError("GLFW could not initialize a display")
            initialized = True
            glfw.default_window_hints()
            glfw.window_hint(glfw.DOUBLEBUFFER, glfw.TRUE)
            window = glfw.create_window(1600, 900, "SPD Simulation", None, None)
            if not window:
                raise RuntimeError("GLFW could not create the simulation window")
            glfw.set_window_size_limits(window, 960, 600, glfw.DONT_CARE, glfw.DONT_CARE)
            glfw.make_context_current(window)
            glfw.swap_interval(1)
            context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150)
            mj.mjr_setBuffer(mj.mjtFramebuffer.mjFB_WINDOW, context)
            data = mj.MjData(model)
            options = mj.MjvOption()
            options.geomgroup[0] = 0
            options.geomgroup[3] = 0
            perturb = mj.MjvPerturb()
            capacity = max(2000, 2 * model.ngeom + model.nsite + model.ntendon)
            left_scene = mj.MjvScene(model, maxgeom=capacity)
            left_camera = mj.MjvCamera()
            mj.mjv_defaultFreeCamera(model, left_camera)
            left_camera.orthographic = 0
            left_fovy = float(model.vis.global_.fovy)
            head_fovy = 90.0
            right_scene = right_camera = None
            base_ids = None
            if self._split_view:
                base_ids = tuple(
                    mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, name)
                    for name in ("Base_L", "Base_R")
                )
                if any(body_id < 0 for body_id in base_ids):
                    raise ValueError("head observation requires both robot arm bases")
                right_scene = mj.MjvScene(model, maxgeom=capacity)
                right_camera = mj.MjvCamera()
                mj.mjv_defaultFreeCamera(model, right_camera)
                right_camera.orthographic = 0
            pitch = math.radians(35.0)
            head_forward = np.array((math.cos(pitch), 0.0, -math.sin(pitch)))
            head_up = np.array((math.sin(pitch), 0.0, math.cos(pitch)))
            ghost_renderer = None
            packet = None
            header_key = None
            header_pixels = None
            # Framebuffer-space layout is shared only by this thread's callbacks.
            layout = (0, 0, 0, 0)  # width, height, header height, left width
            drag_buttons: set[int] = set()
            last_cursor = glfw.get_cursor_pos(window)
            close_notified = False

            def request_close(key: str = "q") -> None:
                nonlocal close_notified
                if close_notified or self._stop.is_set():
                    return
                close_notified = True
                self._stop.set()
                self._on_key(key)
                glfw.set_window_should_close(window, True)

            def guarded(callback: Callable[..., None]) -> Callable[..., None]:
                # ctypes callbacks otherwise print and swallow Python exceptions.
                def invoke(*args: Any) -> None:
                    if self._stop.is_set():
                        return
                    try:
                        callback(*args)
                    except BaseException as exc:
                        if self._error is None:
                            self._error = exc.with_traceback(None)
                        self._stop.set()
                return invoke

            def left_point(x: float, y: float) -> tuple[float, float] | None:
                width, height, header_height, left_width = layout
                window_width, window_height = glfw.get_window_size(window)
                # Ignore callbacks for stale layout between resize and redraw.
                if (width, height) != glfw.get_framebuffer_size(window):
                    return None
                if window_width <= 0 or window_height <= 0:
                    return None
                px = x * width / window_width
                py = y * height / window_height
                if 0 <= px < left_width and header_height <= py < height:
                    return px, py
                return None

            editor_keys = {
                glfw.KEY_UP: "up", glfw.KEY_DOWN: "down",
                glfw.KEY_LEFT: "left", glfw.KEY_RIGHT: "right",
                glfw.KEY_ENTER: "enter", glfw.KEY_KP_ENTER: "enter",
            }

            def key_callback(_window: Any, key: int, _scan: int, action: int, _mods: int) -> None:
                nonlocal head_fovy
                if action != glfw.PRESS:
                    return
                if key in (glfw.KEY_ESCAPE, glfw.KEY_Q):
                    request_close("escape" if key == glfw.KEY_ESCAPE else "q")
                elif self._split_view and key in (glfw.KEY_LEFT_BRACKET, glfw.KEY_RIGHT_BRACKET):
                    delta = 5.0 if key == glfw.KEY_RIGHT_BRACKET else -5.0
                    head_fovy = min(175.0, max(5.0, head_fovy + delta))
                elif key in editor_keys:
                    self._on_key(editor_keys[key])
                elif 0 <= key < 128:
                    self._on_key(chr(key).lower())

            def focus_callback(_window: Any, focused: int) -> None:
                if not focused:
                    drag_buttons.clear()

            def button_callback(_window: Any, button: int, action: int, _mods: int) -> None:
                nonlocal last_cursor
                last_cursor = glfw.get_cursor_pos(window)
                if action == glfw.RELEASE:
                    drag_buttons.discard(button)
                elif action == glfw.PRESS and left_point(*last_cursor) is not None:
                    if button in (glfw.MOUSE_BUTTON_LEFT, glfw.MOUSE_BUTTON_RIGHT, glfw.MOUSE_BUTTON_MIDDLE):
                        drag_buttons.add(button)

            def cursor_callback(_window: Any, x: float, y: float) -> None:
                nonlocal last_cursor
                previous = left_point(*last_cursor)
                current = left_point(x, y)
                last_cursor = (x, y)
                if previous is None or current is None:
                    drag_buttons.clear()
                    return
                if not drag_buttons:
                    return
                shift = (
                    glfw.get_key(window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS
                    or glfw.get_key(window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS
                )
                if glfw.MOUSE_BUTTON_RIGHT in drag_buttons:
                    action = mj.mjtMouse.mjMOUSE_MOVE_H if shift else mj.mjtMouse.mjMOUSE_MOVE_V
                elif glfw.MOUSE_BUTTON_MIDDLE in drag_buttons:
                    action = mj.mjtMouse.mjMOUSE_ZOOM
                else:
                    action = mj.mjtMouse.mjMOUSE_ROTATE_H if shift else mj.mjtMouse.mjMOUSE_ROTATE_V
                viewport_height = layout[1] - layout[2]
                mj.mjv_moveCamera(
                    model, action, (current[0] - previous[0]) / viewport_height,
                    (current[1] - previous[1]) / viewport_height, left_camera,
                )

            def scroll_callback(_window: Any, _dx: float, dy: float) -> None:
                if left_point(*glfw.get_cursor_pos(window)) is not None:
                    mj.mjv_moveCamera(model, mj.mjtMouse.mjMOUSE_ZOOM, 0.0, -0.05 * dy, left_camera)

            glfw.set_key_callback(window, guarded(key_callback))
            glfw.set_window_focus_callback(window, guarded(focus_callback))
            glfw.set_mouse_button_callback(window, guarded(button_callback))
            glfw.set_cursor_pos_callback(window, guarded(cursor_callback))
            glfw.set_scroll_callback(window, guarded(scroll_callback))
            glfw.set_window_close_callback(window, guarded(lambda _window: request_close()))

            while not self._stop.is_set():
                frame_started = time.monotonic()
                glfw.poll_events()
                if glfw.window_should_close(window):
                    request_close()
                if self._stop.is_set():
                    break
                with self._lock:
                    incoming, self._pending = self._pending, None
                    framing, self._pending_frame = self._pending_frame, _NO_FRAME
                if incoming is not None:
                    packet = incoming
                    mj.mj_setState(model, data, packet[0], self._state_spec)
                    mj.mj_kinematics(model, data)
                    mj.mj_comPos(model, data)
                    mj.mj_camlight(model, data)
                if packet is None:
                    raise RuntimeError("native viewer requires an initial state snapshot")
                if framing is not _NO_FRAME:
                    if framing is None:
                        visual_positions = data.geom_xpos[model.geom_group == 1]
                        if visual_positions.size:
                            low, high = visual_positions.min(axis=0), visual_positions.max(axis=0)
                            left_camera.lookat[:] = (low + high) * 0.5
                            left_camera.distance = max(2.0, float(np.linalg.norm(high - low)) * 2.0)
                        else:
                            mj.mjv_defaultFreeCamera(model, left_camera)
                        left_camera.azimuth = 135.0
                        left_camera.elevation = -20.0
                    else:
                        from simulation.scene import frame_scene

                        frame_scene(left_camera, options, float(framing))
                    left_camera.orthographic = 0
                width, height = glfw.get_framebuffer_size(window)
                if width <= 0 or height <= 0 or glfw.get_window_attrib(window, glfw.ICONIFIED):
                    layout = (0, 0, 0, 0)
                    self._running.set()
                    self._ready.set()
                    self._stop.wait(0.05)
                    continue
                _, task, hud, ghost, label = packet
                new_header_key = (width, height, task, hud, label, head_fovy)
                if new_header_key != header_key:
                    if self._split_view:
                        hud = (*hud, ("固定视角 FOV", f"{head_fovy:g}°；[ 缩小 / ] 放大，每次 5°"))
                    header = self._render_header(width, height, task, hud, label)
                    if (
                        not isinstance(header, np.ndarray) or header.dtype != np.uint8
                        or header.ndim != 3 or header.shape[1:] != (width, 3)
                        or not 0 <= header.shape[0] < height
                    ):
                        raise ValueError("header must be RGB uint8 with framebuffer width and height below the window height")
                    header_pixels = np.ascontiguousarray(header[::-1])
                    header_key = new_header_key
                header_height = header_pixels.shape[0]
                view_height = height - header_height
                left_width, separator, right_width = _viewport_widths(width, self._split_view)
                layout = (width, height, header_height, left_width)
                if left_width <= 0:
                    self._stop.wait(0.05)
                    continue
                if ghost is not None and ghost_renderer is None:
                    from simulation.hand_ghost import HandGhost

                    ghost_renderer = HandGhost(model, data)
                model.vis.global_.fovy = left_fovy
                mj.mjv_updateScene(
                    model, data, options, perturb, left_camera,
                    mj.mjtCatBit.mjCAT_ALL, left_scene,
                )
                if ghost is not None:
                    ghost_renderer.draw(left_scene, ghost, start_index=left_scene.ngeom)
                mj.mjr_render(mj.MjrRect(0, 0, left_width, view_height), left_scene, context)
                if right_scene is not None:
                    eye = (data.xpos[base_ids[0]] + data.xpos[base_ids[1]]) * 0.5
                    eye[2] += 0.35
                    right_camera.type = mj.mjtCamera.mjCAMERA_FREE
                    right_camera.fixedcamid = -1
                    right_camera.lookat[:] = eye + head_forward
                    right_camera.distance = 1.0
                    right_camera.azimuth = 0.0
                    right_camera.elevation = -35.0
                    model.vis.global_.fovy = head_fovy
                    mj.mjv_updateScene(
                        model, data, options, perturb, right_camera,
                        mj.mjtCatBit.mjCAT_ALL, right_scene,
                    )
                    # Remove stereo eye offsets: the head pose is exactly the
                    # midpoint of the bases, in world coordinates, for both eyes.
                    for camera in right_scene.camera:
                        camera.pos[:] = eye
                        camera.forward[:] = head_forward
                        camera.up[:] = head_up
                        camera.frustum_center = 0.0
                        camera.orthographic = 0
                    if ghost is not None:
                        ghost_renderer.draw(right_scene, ghost, start_index=right_scene.ngeom)
                    mj.mjr_render(
                        mj.MjrRect(left_width + separator, 0, right_width, view_height),
                        right_scene, context,
                    )
                    model.vis.global_.fovy = left_fovy
                    mj.mjr_rectangle(
                        mj.MjrRect(left_width, 0, separator, view_height),
                        0.16, 0.20, 0.25, 1.0,
                    )
                if header_height:
                    header_rect = mj.MjrRect(0, view_height, width, header_height)
                    # This initializes 2D state; a raw pixel blit after 3D can
                    # otherwise inherit depth/lighting and silently disappear.
                    mj.mjr_rectangle(header_rect, 0.06, 0.08, 0.11, 1.0)
                    mj.mjr_drawPixels(header_pixels.reshape(-1), None, header_rect, context)
                glfw.swap_buffers(window)
                self._running.set()
                self._ready.set()
                # Bound redraws even when the driver ignores swap interval.
                self._stop.wait(max(0.0, 1.0 / 60.0 - (time.monotonic() - frame_started)))
        finally:
            # Release GPU resources while this thread still owns its context.
            try:
                if context is not None:
                    context.free()
            finally:
                try:
                    if window:
                        glfw.make_context_current(None)
                        glfw.destroy_window(window)
                finally:
                    if initialized:
                        glfw.terminate()
