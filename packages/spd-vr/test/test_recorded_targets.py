import io

import h5py
import numpy as np
from PIL import Image
import pytest

from spd_vr.recorded_targets import RecordedTargets


def _recording(path, arm_times, arm_values, hand_times, hand_values):
    with h5py.File(path, "w") as handle:
        handle.attrs.update(schema_version=1, robot_config="tianji_wuji2_v1", task="pick_hammer")
        for name, times, values, width in (
            ("arms", arm_times, arm_values, 14),
            ("hands", hand_times, hand_values, 40),
        ):
            group = handle.create_group(f"observations/{name}")
            group.create_dataset("timestamp_ns", data=np.asarray(times, dtype=np.int64) * 1_000_000_000)
            group.create_dataset("qpos", data=np.repeat(np.asarray(values, dtype=np.float32)[:, None], width, axis=1))
    return path


def test_independent_asof_uses_common_interval_not_unpaired_tail(tmp_path):
    source = RecordedTargets(_recording(tmp_path / "episode.h5", [0, 2, 5], [0, 2, 5], [1, 3, 6], [10, 30, 60]))
    assert source.duration_s == 4.0
    for elapsed, arm, hand in ((0, 0, 10), (1, 2, 10), (2, 2, 30), (100, 5, 30)):
        sample = source.sample(elapsed)
        assert sample.dtype == np.float64
        np.testing.assert_array_equal(sample, [arm] * 14 + [hand] * 40)


def test_duplicate_timestamps_choose_last_joint_and_original_jpeg(tmp_path):
    path = _recording(tmp_path / "episode.h5", [0, 2, 2, 4], [0, 1, 2, 4], [0, 1, 1, 5], [0, 10, 20, 50])
    payloads = []
    for color in ("red", "blue"):
        buffer = io.BytesIO()
        Image.new("RGB", (1, 1), color).save(buffer, format="JPEG")
        payloads.append(buffer.getvalue())
    with h5py.File(path, "a") as handle:
        group = handle.create_group("images/top")
        group.create_dataset("timestamp_ns", data=np.asarray([1, 1], dtype=np.int64) * 1_000_000_000)
        jpeg = group.create_dataset("jpeg", shape=(2,), dtype=h5py.vlen_dtype(np.dtype("uint8")))
        for index, payload in enumerate(payloads):
            jpeg[index] = np.frombuffer(payload, dtype=np.uint8)
    source = RecordedTargets(path)
    np.testing.assert_array_equal(source.sample(2), [2] * 14 + [20] * 40)
    assert source.image_jpeg("top", 0) is None
    assert source.image_jpeg("top", 1) == payloads[-1]
    assert source.image_jpeg("top", 100) == payloads[-1]


def test_disjoint_streams_rejected_but_single_common_instant_is_usable(tmp_path):
    disjoint = _recording(tmp_path / "disjoint.h5", [0, 1], [0, 1], [2, 3], [2, 3])
    with pytest.raises(ValueError, match="no common time interval"):
        RecordedTargets(disjoint)
    touching = _recording(tmp_path / "touching.h5", [0, 2], [0, 2], [2, 3], [20, 30])
    source = RecordedTargets(touching)
    assert source.duration_s == 0
    np.testing.assert_array_equal(source.sample(10), [2] * 14 + [20] * 40)
