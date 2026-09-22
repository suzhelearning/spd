import json
from datetime import date
from pathlib import Path
import threading
import time
from types import SimpleNamespace

import h5py
import numpy as np
import pytest
import yaml

from spd_vr.cameras.camera import CAMERA_NAMES, CameraFrame
from spd_vr.data_collector.config import CollectionConfig, load_collection_config
from spd_vr.data_collector.recorder import validate_episode_path
from spd_vr.data_collector.session import CollectionSession
from spd_vr.simulation.viewer import PlantController


class Camera:
    def __init__(self):
        self._renderers = {}
        self.rgb = np.zeros((720, 1280, 3), dtype=np.uint8)

    def capture(self, timestamp):
        return {name: CameraFrame(name, timestamp, self.rgb, "test") for name in CAMERA_NAMES}


def make_session(tmp_path, *, max_frames=0):
    mailbox = SimpleNamespace(enabled=True, latest=SimpleNamespace(ready_mask=7, stamp_ns=time.time_ns()))
    executor = SimpleNamespace(mailbox=mailbox, hold_mask=0)
    plant = SimpleNamespace(physics_hz=480, joint_command_positions=lambda: np.arange(54, dtype=np.float32))
    config = CollectionConfig(1, tmp_path, 120, 30, 256, max_frames)
    return CollectionSession(config, plant, executor, {"task": "test"}, Camera)




def wait_state(session, expected):
    deadline = time.monotonic() + 10
    while session.state != expected:
        assert time.monotonic() < deadline, session.snapshot()
        session.executor.mailbox.latest.stamp_ns = time.time_ns()
        session.poll()
        time.sleep(0.001)


def step(tick):
    return SimpleNamespace(tick=tick, sim_time_ns=tick * 2_083_333)


def test_limit_uses_actual_samples_and_independent_rates(tmp_path):
    session = make_session(tmp_path, max_frames=3)
    try:
        accepted, response = session.request("start")
        assert accepted
        wait_state(session, "recording")
        session.tick(step(100))
        assert session.snapshot()["state_frames"] == 1
        assert all(value == 1 for value in session.snapshot()["camera_frames"].values())
        for tick in range(101, 109):
            session.tick(step(tick))
        wait_state(session, "idle")
        status = session.snapshot()
        assert status["operation_id"] == response["operation_id"]
        assert status["state_frames"] == 3
        assert status["camera_frames"] == dict.fromkeys(CAMERA_NAMES, 1)
        assert validate_episode_path(status["last_saved_path"])["success"] is False
        with h5py.File(status["last_saved_path"], "r") as handle:
            assert set(handle["observations"]) == {"arms", "hands"}
            assert handle["observations/arms/qpos"].shape == (3, 14)
            assert handle["observations/hands/qpos"].shape == (3, 40)
            manifest = json.loads(handle.attrs["task_manifest"])
            assert manifest["collection_config"]["max_frames"] == 3
        session.tick(step(110))
        assert session.snapshot()["state_frames"] == 3
    finally:
        session.close()


def test_one_sample_limit_has_real_rgb_and_unsuccessful_result(tmp_path):
    session = make_session(tmp_path, max_frames=1)
    try:
        assert session.request("start")[0]
        wait_state(session, "recording")
        session.tick(step(7))
        wait_state(session, "idle")
        assert validate_episode_path(session.last_saved_path)["success"] is False
        assert session.camera_frames == dict.fromkeys(CAMERA_NAMES, 1)
    finally:
        session.close()


def test_collection_records_physics_state_without_new_command(tmp_path):
    plant = PlantController()
    for _ in range(120):
        plant.physics_tick()
    targets = plant.joint_command_targets()
    mailbox = SimpleNamespace(enabled=True, latest=SimpleNamespace(ready_mask=7, stamp_ns=time.time_ns()))
    executor = SimpleNamespace(mailbox=mailbox, hold_mask=0)
    config = CollectionConfig(1, tmp_path, 120, 30, 256, 1)
    session = CollectionSession(config, plant, executor, {"task": "state-source"}, Camera)
    try:
        assert session.request("start")[0]
        wait_state(session, "recording")
        physics_step = plant.physics_tick()
        actual = plant.joint_command_positions().copy()
        assert np.max(np.abs(actual - targets)) > 1e-4
        session.tick(physics_step)
        wait_state(session, "idle")
        with h5py.File(session.last_saved_path, "r") as handle:
            recorded = np.concatenate((
                handle["observations/arms/qpos"][0],
                handle["observations/hands/qpos"][0],
            ))
            np.testing.assert_allclose(recorded, actual, atol=1e-7)
            assert not np.allclose(recorded, targets, atol=1e-4)
            assert set(handle["observations"]) == {"arms", "hands"}
            assert "actions" not in handle
    finally:
        session.close()
        plant.close()


def test_busy_operations_reject_without_overwriting_accepted_id(tmp_path, monkeypatch):
    session = make_session(tmp_path)
    gate = threading.Event()
    original_start = session.recorder.start_episode
    original_finish = session.recorder.finish_episode

    def delayed_start(*args, **kwargs):
        assert gate.wait(10)
        return original_start(*args, **kwargs)

    def delayed_finish(**kwargs):
        assert gate.wait(10)
        return original_finish(**kwargs)

    try:
        monkeypatch.setattr(session.recorder, "start_episode", delayed_start)
        accepted, start_response = session.request("start")
        assert accepted
        assert not session.request("save")[0]
        assert not session.request("discard")[0]
        assert session.operation_id == start_response["operation_id"]
        assert session.camera is None
        gate.set()
        wait_state(session, "recording")
        session.tick(step(1))
        gate.clear()
        monkeypatch.setattr(session.recorder, "finish_episode", delayed_finish)
        accepted, save_response = session.request("save")
        assert accepted
        assert not session.request("discard")[0]
        assert not session.request("start")[0]
        assert session.operation_id == save_response["operation_id"]
        gate.set()
        wait_state(session, "idle")
        assert validate_episode_path(session.last_saved_path)["success"] is True
    finally:
        gate.set()
        session.close()


def test_reject_disabled_fully_held_and_stale_start(tmp_path):
    session = make_session(tmp_path)
    try:
        session.executor.mailbox.enabled = False
        assert not session.request("start")[0]
        session.executor.mailbox.enabled = True
        session.executor.hold_mask = 7
        assert not session.request("start")[0]
        session.executor.hold_mask = 0
        session.executor.mailbox.latest.stamp_ns = time.time_ns() - 1_000_000_000
        assert not session.request("start")[0]
        assert session.state == "idle"
        assert not tuple(tmp_path.glob("*.h5"))
    finally:
        session.close()


def test_shutdown_preserves_partial_and_never_success(tmp_path):
    session = make_session(tmp_path)
    assert session.request("start")[0]
    wait_state(session, "recording")
    session.tick(step(1))
    operation_id = session.operation_id
    session.close()
    status = session.snapshot()
    assert status["state"] == "error"
    assert status["operation_id"] == operation_id
    assert status["error"]
    assert not status["last_saved_path"]
    partial = Path(status["episode_path"])
    assert partial.name.endswith(".partial.h5")
    with h5py.File(partial, "r") as handle:
        assert not bool(handle.attrs["success"])
        assert handle["observations/arms/qpos"].shape[0] == 1


def test_render_error_preserves_partial_and_allows_next_episode(tmp_path, monkeypatch):
    session = make_session(tmp_path)
    try:
        assert session.request("start")[0]
        wait_state(session, "recording")

        def failed_capture(_timestamp):
            raise RuntimeError("camera unavailable")

        monkeypatch.setattr(session.camera, "capture", failed_capture)
        session.tick(step(1))
        wait_state(session, "error")
        assert "camera unavailable" in session.error
        partial = Path(session.episode_path)
        assert partial.is_file()
        session.executor.mailbox.latest.stamp_ns = time.time_ns()
        assert session.request("start")[0]
        wait_state(session, "recording")
        assert session.request("discard")[0]
        wait_state(session, "idle")
        assert partial.is_file()
        assert not session.episode_path
    finally:
        session.close()


def test_collection_config_paths_and_explicit_overrides(tmp_path):
    path = tmp_path / "config" / "collect.yaml"
    path.parent.mkdir()
    document = dict(version=1, data_dir="../episodes", state_rate_hz=120,
                    camera_rate_hz=30, writer_queue_size=256, max_frames=9)
    path.write_text(yaml.safe_dump(document))
    config = load_collection_config(path)
    assert config.data_dir == tmp_path / "episodes"
    overridden = load_collection_config(path, output=tmp_path / "other", max_frames=0)
    assert overridden.data_dir == tmp_path / "other"
    assert overridden.max_frames == 0
    document["state_rate_hz"] = 100
    path.write_text(yaml.safe_dump(document))
    with pytest.raises(ValueError):
        load_collection_config(path)
    document["state_rate_hz"] = True
    path.write_text(yaml.safe_dump(document))
    with pytest.raises(ValueError):
        load_collection_config(path)


def test_daily_directory_is_fixed_at_start_and_rolls_for_next_episode(tmp_path, monkeypatch):
    today = [date(2026, 9, 22)]
    monkeypatch.setattr("spd_vr.data_collector.session.date", SimpleNamespace(today=lambda: today[0]))
    session = make_session(tmp_path, max_frames=1)
    gate = threading.Event()
    original_start = session.recorder.start_episode

    def delayed_start(*args, **kwargs):
        assert gate.wait(10)
        return original_start(*args, **kwargs)

    monkeypatch.setattr(session.recorder, "start_episode", delayed_start)
    try:
        assert session.request("start")[0]
        assert Path(session.episode_path).parent == tmp_path / "20260922"
        today[0] = date(2026, 9, 23)
        gate.set()
        wait_state(session, "recording")
        session.tick(step(1))
        wait_state(session, "idle")
        first = Path(session.last_saved_path)
        assert first.parent == tmp_path / "20260922"
        assert validate_episode_path(first)["valid"]
        assert not (tmp_path / "20260923").exists()

        session.executor.mailbox.latest.stamp_ns = time.time_ns()
        assert session.request("start")[0]
        wait_state(session, "recording")
        session.tick(step(2))
        wait_state(session, "idle")
        second = Path(session.last_saved_path)
        assert second.parent == tmp_path / "20260923"
        assert first.is_file() and second.is_file()
        assert validate_episode_path(second)["valid"]
        assert (first.parent / "dataset_config.json").is_file()
        assert (second.parent / "dataset_config.json").is_file()
        assert not (tmp_path / "dataset_config.json").exists()
    finally:
        gate.set()
        session.close()


def test_daily_config_conflict_does_not_overwrite_or_block_next_day(tmp_path, monkeypatch):
    today = [date(2026, 9, 22)]
    monkeypatch.setattr("spd_vr.data_collector.session.date", SimpleNamespace(today=lambda: today[0]))
    directory = tmp_path / "20260922"
    directory.mkdir()
    config_path = directory / "dataset_config.json"
    config_path.write_text('{"robot_config": "different-robot"}')
    session = make_session(tmp_path, max_frames=1)
    try:
        assert session.request("start")[0]
        wait_state(session, "error")
        assert config_path.read_text() == '{"robot_config": "different-robot"}'
        assert not tuple(directory.glob("*.h5"))
        today[0] = date(2026, 9, 23)
        session.executor.mailbox.latest.stamp_ns = time.time_ns()
        assert session.request("start")[0]
        wait_state(session, "recording")
        session.tick(step(1))
        wait_state(session, "idle")
        assert Path(session.last_saved_path).parent == tmp_path / "20260923"
        assert validate_episode_path(session.last_saved_path)["valid"]
    finally:
        session.close()
