"""Small adapter around MuJoCo's passive viewer.

The adapter deliberately owns no model or data.  ``PlantController`` remains the
single owner of the complete simulation state, while this class only renders it.
"""

from __future__ import annotations

from typing import Any, Callable, Mapping


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
        window: Any | None = None,
    ) -> None:
        self.model = model
        self.data = data
        self.headless = bool(headless)
        self._clock_ns = clock_ns
        self._shutdown = shutdown
        self._control = control
        self._state = state
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
        update = getattr(self._window, "update_hud", None)
        if update is not None:
            update(self.hud)
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

    def __enter__(self) -> "ViewerWindow":
        return self.open()

    def __exit__(self, *_: Any) -> None:
        self.close()


# Name used by a few callers that describe the object by behavior.
PassiveViewerWindow = ViewerWindow


__all__ = ["PassiveViewerWindow", "ViewerWindow"]
