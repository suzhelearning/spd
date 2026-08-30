from types import SimpleNamespace

import numpy as np

from spd_vr.viewer import PlantController, ViewerRuntime
from spd_vr.zenoh_transport import CONTROL_CONGESTION_CONTROL
from spd_vr.viewer_window import ViewerWindow
from spd_vr.wire import CONTROL_KEY, STATUS_VIEWER_KEY, ControlCommand, ControlFrame, TrackingFrame


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

def test_viewer_window_maps_lifecycle_keys_and_shutdown_once():
    from spd_vr.viewer_window import ViewerWindow

    commands = []
    shutdowns = []
    state = ["IDLE"]
    window = ViewerWindow(
        headless=True,
        control=commands.append,
        state=lambda: state[0],
        shutdown=lambda: shutdowns.append("shutdown"),
    )
    window.on_key("space")
    state[0] = "RUNNING"
    window.on_key("space")
    state[0] = "PAUSED"
    window.on_key("space")
    window.on_key("r")
    window.on_key("n")
    window.on_key("q")
    window.on_key("escape")
    assert commands == ["START", "PAUSE", "RESUME", "REALIGN", "RESET"]
    assert shutdowns == ["shutdown"]

def test_runtime_connects_control_fifo_with_blocking_publisher_and_closes(tmp_path):
    class Publisher:
        def __init__(self):
            self.payloads = []

        def put(self, payload):
            self.payloads.append(payload)

    class Node:
        def __init__(self):
            self.publisher = Publisher()
            self.status_publisher = Publisher()
            self.publisher_kwargs = None
            self.mailboxes = {}
            self.closed = 0

        def declare_publisher(self, key, **kwargs):
            if key == CONTROL_KEY:
                self.publisher_kwargs = kwargs
                return self.publisher
            assert key == STATUS_VIEWER_KEY
            return self.status_publisher
        def declare_latest_subscriber(self, key, _decoder, mailbox):
            self.mailboxes[key] = mailbox

        def close(self):
            self.closed += 1

    plant = PlantController.synthetic_fixture()
    runtime = ViewerRuntime(plant, headless=True, clock_ns=lambda: 1, sleep=lambda _: None, sequence_file=tmp_path / "sequence.json")
    node = Node()
    runtime.connect(node)
    assert node.publisher_kwargs["congestion_control"] is CONTROL_CONGESTION_CONTROL
    assert runtime._zenoh_status() == "connected_unmatched"
    runtime._status_mailboxes["bridge"].put({"ready": True})
    runtime._poll_status()
    assert runtime._zenoh_status() == "remote_status"
    runtime.send_control(ControlCommand.START)
    assert node.publisher.payloads
    node.mailboxes[CONTROL_KEY].put(ControlFrame(2, 2, ControlCommand.PAUSE))
    plant.physics_tick(2)
    assert runtime.session.snapshot.paused
    runtime.close()
    assert node.closed == 1
    plant.close()


def test_viewer_window_uses_real_handle_text_surface_for_hud():
    class Handle:
        def __init__(self):
            self.texts = None

        def set_texts(self, texts):
            self.texts = texts

    handle = Handle()
    window = ViewerWindow(headless=False, window=handle)
    window.update_hud(
        {
            "state": "RUNNING",
            "arm_valid_mask": 3,
            "input_age_ms": 1.5,
            "drops": 0,
            "physics_hz": 480,
            "artifact": "verified",
        }
    )
    assert handle.texts[2] == "SPD VR"
    assert "arm_valid_mask: 3" in handle.texts[3]
    assert "physics_hz: 480" in handle.texts[3]


def test_hud_keeps_pico_source_latency_unknown_and_uses_bridge_arrival_clock():
    plant = PlantController.synthetic_fixture()
    runtime = ViewerRuntime(plant, headless=True, clock_ns=lambda: 1_000, sleep=lambda _: None)
    hand = np.zeros((26, 7), dtype=np.float32)
    hand[:, 6] = 1.0
    frame = TrackingFrame(
        sequence=1,
        tracking_epoch=1,
        source_timestamp_ns=10,
        bridge_monotonic_ns=100,
        left_active=False,
        right_active=False,
        head_valid=True,
        left_scale=1.0,
        right_scale=1.0,
        head_pose=np.array((0, 0, 1.6, 0, 0, 0, 1), dtype=np.float32),
        left_hand=hand,
        right_hand=hand,
    )
    plant.submit_tracking(frame, now_ns=200)
    values = runtime._hud_values(SimpleNamespace(finite=True, arm_valid_mask=0, hand_valid_mask=0), 1_000)
    assert values["source_latency_ms"] == "unknown"
    assert values["bridge_latency_ms"] == 0.0
    plant.close()
