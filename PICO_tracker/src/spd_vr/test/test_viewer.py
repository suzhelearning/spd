from types import SimpleNamespace

from spd_vr.viewer import ViewerRuntime
from spd_vr.wire import ControlCommand


class Clock:
    def __init__(self):
        self.now = 0

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += int(seconds * 1_000_000_000)


def test_runtime_runs_480hz_ticks_with_independent_60hz_render():
    clock = Clock()
    ticks = []
    renders = []

    class Plant:
        def physics_tick(self, now_ns):
            ticks.append(now_ns)
            return SimpleNamespace(finite=True)

    class Window:
        def sync(self):
            renders.append(clock.now)

    runtime = ViewerRuntime(Plant(), window=Window(), clock_ns=clock, sleep=clock.sleep)
    runtime.run(ticks=960, auto_start=True)
    assert len(ticks) == 960
    assert 110 <= len(renders) <= 130
    assert all(b >= a for a, b in zip(ticks, ticks[1:]))


def test_headless_runtime_does_not_open_window_and_shutdown_is_once():
    class Plant:
        def __init__(self):
            self.calls = 0

        def physics_tick(self, _now_ns):
            self.calls += 1

    plant = Plant()
    runtime = ViewerRuntime(plant, headless=True, clock_ns=lambda: 0, sleep=lambda _: None)
    runtime.run(ticks=2, auto_start=True)
    assert plant.calls == 2
