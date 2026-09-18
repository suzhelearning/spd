import h5py
import numpy as np

from spd_vr.camera import CameraFrame
from spd_vr.recorder import EpisodeRecorder, validate_episode_path


def _frame(name: str, timestamp_ns: int) -> CameraFrame:
    rgb = np.zeros((720, 1280, 3), dtype=np.uint8)
    segmentation = np.zeros((720, 1280, 2), dtype=np.int32)
    return CameraFrame(name, timestamp_ns, rgb, segmentation, "test")


def test_schema_v1_contains_state_command_and_jpeg_streams(tmp_path):
    recorder = EpisodeRecorder(tmp_path, camera_names=("top", "left_wrist"))
    recorder.start_episode(1, {"task": "pick"})
    recorder.append_arm_qpos(10, np.zeros(14))
    recorder.append_arm_qpos(10, np.ones(14))
    recorder.append_hand_qpos(11, np.zeros(40), both_fresh=False)
    recorder.append_hand_qpos(12, np.ones(40), both_fresh=True)
    recorder.append_cameras({"top": _frame("top", 13), "left_wrist": _frame("left_wrist", 14)})
    recorder.append_command(
        13,
        np.zeros(54),
        sequence=1,
        stamp_utc_ns=1_700_000_000_000_000_000,
        ready_mask=7,
        session_id="test-session",
        applied_sim_time_ns=12,
    )

    episode = recorder.finish_episode()
    assert validate_episode_path(episode)["valid"] is True
    with h5py.File(episode, "r") as handle:
        assert set(handle["observations"]) == {"arms", "hands", "commands"}
        assert handle["observations/arms/qpos"].shape == (1, 14)
        assert handle["observations/arms/qpos"].dtype == np.float32
        np.testing.assert_allclose(handle["observations/arms/qpos"][0], 1.0)
        assert handle["observations/commands/position_rad"].shape == (1, 54)
        assert handle["observations/hands/qpos"].shape == (1, 40)
        assert set(handle["images"]) == {"top", "left_wrist"}
        assert bytes(handle["images/top/jpeg"][0][:2]) == b"\xff\xd8"
        assert bool(handle.attrs["success"])
