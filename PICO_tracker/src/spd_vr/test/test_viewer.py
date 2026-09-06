from contextlib import nullcontext
from pathlib import Path
from types import SimpleNamespace

import numpy as np

import spd_vr.viewer as viewer_module
from spd_vr.pico_hands import PICO_TO_MEDIAPIPE
from spd_vr.viewer import INPUT_STALE_NS, PlantController, ViewerRuntime
from spd_vr.zenoh_transport import CONTROL_CONGESTION_CONTROL
from spd_vr.viewer_window import MEDIAPIPE_CONNECTIONS, ViewerWindow
from spd_vr.wire import CONTROL_KEY, STATUS_VIEWER_KEY, ControlCommand, ControlFrame, TrackingFrame

def test_main_production_parser_passes_manifest_and_urdf(monkeypatch):
    captured = {}

    class Plant:
        data = SimpleNamespace(qpos=np.zeros(1), qvel=np.zeros(1), ctrl=np.zeros(1))
        sim_time_ns = 0

        def close(self):
            pass

    class Runtime:
        def __init__(self, plant, **_kwargs):
            self.plant = plant

        def connect(self, _node):
            pass

        def run(self, **_kwargs):
            return 0

        def close(self):
            pass

    class Node:
        def close(self):
            pass

    def make_plant(model, manifest, **kwargs):
        captured.update(model=model, manifest=manifest, **kwargs)
        return Plant()

    monkeypatch.setattr(viewer_module, "PlantController", make_plant)
    monkeypatch.setattr(viewer_module, "ViewerRuntime", Runtime)
    monkeypatch.setattr(viewer_module, "ZenohNode", lambda _config: Node())
    assert viewer_module.main(["--headless", "--ticks", "0", "--model", "model.xml", "--manifest", "manifest.yaml", "--urdf", "robot.urdf"]) == 0
    assert captured["manifest"] == Path("manifest.yaml")
    assert captured["urdf_path"] == Path("robot.urdf")



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


def test_runtime_starts_with_mujoco_physics_paused_by_default(tmp_path):
    plant = PlantController.synthetic_fixture()
    runtime = ViewerRuntime(
        plant,
        headless=True,
        clock_ns=lambda: 1,
        sleep=lambda _: None,
        sequence_file=tmp_path / "sequence.json",
    )

    runtime.run(ticks=2)

    assert runtime.session.snapshot.paused
    assert plant.paused is True
    assert plant.tick == 0
    assert plant.sim_time_ns == 0

    runtime.send_control(ControlCommand.RESUME)
    runtime.run(ticks=1)
    assert runtime.session.snapshot.running
    assert plant.sim_time_ns > 0
    plant.close()


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


def test_visible_viewer_hides_mujoco_side_panels(monkeypatch):
    import mujoco.viewer as mujoco_viewer

    captured = {}
    handle = object()

    def launch_passive(model, data, **kwargs):
        captured.update(model=model, data=data, **kwargs)
        return handle

    monkeypatch.setattr(mujoco_viewer, "launch_passive", launch_passive)
    model = object()
    data = object()

    window = ViewerWindow(model=model, data=data).open()

    assert window.window is handle
    assert captured["show_left_ui"] is False
    assert captured["show_right_ui"] is False


def test_viewer_draws_commanded_wrist_pose_axes():
    import mujoco

    plant = PlantController.synthetic_fixture()
    plant._arm_valid = {"left": True, "right": True}
    plant._alignment_ready = {"left": True, "right": True}

    class Handle:
        def __init__(self):
            self.user_scn = mujoco.MjvScene(plant.model, maxgeom=8)

        def lock(self):
            return nullcontext()

        def sync(self):
            pass

    handle = Handle()
    window = ViewerWindow(window=handle, pose_markers=plant.desired_wrist_poses)
    window.sync()
    assert handle.user_scn.ngeom == 6
    plant.close()


def test_viewer_draws_mediapipe_keypoints_and_finger_connections():
    import mujoco

    plant = PlantController.synthetic_fixture()
    keypoints = np.column_stack(
        (
            np.linspace(-0.05, 0.05, 21),
            np.linspace(0.00, 0.12, 21),
            np.linspace(0.01, 0.03, 21),
        )
    )

    class Handle:
        def __init__(self):
            self.user_scn = mujoco.MjvScene(plant.model, maxgeom=64)

        def lock(self):
            return nullcontext()

        def sync(self):
            pass

    handle = Handle()
    window = ViewerWindow(
        window=handle,
        hand_keypoints=lambda: {"left": keypoints},
    )
    window.sync()

    assert len(MEDIAPIPE_CONNECTIONS) == 23
    assert handle.user_scn.ngeom == 21 + len(MEDIAPIPE_CONNECTIONS)
    geom_types = [int(handle.user_scn.geoms[index].type) for index in range(handle.user_scn.ngeom)]
    assert geom_types.count(int(mujoco.mjtGeom.mjGEOM_SPHERE)) == 21
    assert geom_types.count(int(mujoco.mjtGeom.mjGEOM_CAPSULE)) == 23
    plant.close()


def test_plant_exposes_live_mediapipe_overlay_at_robot_wrist_and_hides_stale_side():
    class Retarget:
        def retarget(self, _frame):
            return {
                "left_qpos": np.zeros(20),
                "right_qpos": np.zeros(20),
                "left_valid": True,
                "right_valid": False,
                "left_hold_reason": "none",
                "right_hold_reason": "inactive",
            }

    mediapipe = np.zeros((21, 3), dtype=np.float32)
    finger_x = (-0.035, -0.0175, 0.0, 0.0175, 0.035)
    for finger, x_value in enumerate(finger_x):
        start = 1 + 4 * finger
        for segment in range(4):
            mediapipe[start + segment] = (
                x_value,
                0.02 + 0.025 * segment,
                0.002 * finger,
            )
    hand = np.zeros((26, 7), dtype=np.float32)
    hand[:, 6] = 1.0
    hand[PICO_TO_MEDIAPIPE, :3] = mediapipe + np.array((0.3, 0.2, 1.1))
    frame = TrackingFrame(
        sequence=1,
        tracking_epoch=1,
        source_timestamp_ns=1,
        bridge_monotonic_ns=1,
        left_active=True,
        right_active=False,
        head_valid=True,
        left_scale=1.0,
        right_scale=1.0,
        head_pose=np.array((0, 0, 1.6, 0, 0, 0, 1), dtype=np.float32),
        left_hand=hand,
        right_hand=hand,
    )
    now_ns = 1_000_000
    plant = PlantController.synthetic_fixture(hand_retargeter=Retarget())
    plant.submit_tracking(frame, now_ns=now_ns)
    plant.physics_tick(now_ns)

    world = plant.mediapipe_keypoints_world()
    assert set(world) == {"left"}
    assert world["left"].shape == (21, 3)
    np.testing.assert_allclose(
        world["left"][0],
        plant.data.site_xpos[plant._wrist_site_ids["left"]],
        atol=1.0e-9,
    )

    plant.physics_tick(now_ns + INPUT_STALE_NS + 1)
    assert plant.mediapipe_keypoints_world() == {}
    plant.close()


def test_mediapipe_overlay_rotates_into_each_wuji2_wrist_frame():
    plant = PlantController.synthetic_fixture()
    canonical = np.array(
        [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
            *np.zeros((17, 3)),
        ],
        dtype=np.float64,
    )
    plant._mediapipe_points = {
        "left": canonical.copy(),
        "right": canonical.copy(),
    }
    plant._mediapipe_arrival = {"left": 1, "right": 1}

    expected_wuji_local = {
        # Wuji2's frozen wrist convention: fingers extend along local -Z.
        "left": canonical @ np.array(
            ((0.0, -1.0, 0.0), (-1.0, 0.0, 0.0), (0.0, 0.0, -1.0))
        ).T,
        "right": canonical @ np.array(
            ((0.0, 1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, -1.0))
        ).T,
    }

    world = plant.mediapipe_keypoints_world()
    for side in ("left", "right"):
        site_id = plant._wrist_site_ids[side]
        wrist_position = plant.data.site_xpos[site_id]
        wrist_rotation = plant.data.site_xmat[site_id].reshape(3, 3)
        actual_wuji_local = (world[side] - wrist_position) @ wrist_rotation
        np.testing.assert_allclose(actual_wuji_local, expected_wuji_local[side])
    plant.close()


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
