import h5py
import numpy as np
import pytest

from spd_vr.cameras.camera import CameraFrame
from spd_vr.data_collector.recorder import EpisodeRecorder, validate_episode_path


def _frame(name: str, timestamp_ns: int) -> CameraFrame:
    rgb = np.zeros((720, 1280, 3), dtype=np.uint8)
    return CameraFrame(name, timestamp_ns, rgb, "test")


def test_schema_v1_contains_only_actual_state_and_jpeg_streams(tmp_path):
    recorder = EpisodeRecorder(tmp_path, camera_names=("top", "left_wrist"))
    recorder.start_episode(1, {"task": "pick"})
    recorder.append_arm_qpos(10, np.zeros(14))
    recorder.append_arm_qpos(10, np.ones(14))
    recorder.append_hand_qpos(11, np.zeros(40), both_fresh=False)
    recorder.append_hand_qpos(12, np.ones(40), both_fresh=True)
    recorder.append_cameras({"top": _frame("top", 13), "left_wrist": _frame("left_wrist", 14)})

    episode = recorder.finish_episode()
    assert validate_episode_path(episode)["valid"] is True
    with h5py.File(episode, "r") as handle:
        assert set(handle["observations"]) == {"arms", "hands"}
        assert handle["observations/arms/qpos"].shape == (1, 14)
        assert handle["observations/arms/qpos"].dtype == np.float32
        np.testing.assert_allclose(handle["observations/arms/qpos"][0], 1.0)
        assert handle["observations/hands/qpos"].shape == (1, 40)
        assert set(handle["images"]) == {"top", "left_wrist"}
        assert bytes(handle["images/top/jpeg"][0][:2]) == b"\xff\xd8"
        assert bool(handle.attrs["success"])
    with h5py.File(episode, "a") as handle:
        handle["observations"].create_group("commands")
    with pytest.raises(ValueError):
        validate_episode_path(episode)
