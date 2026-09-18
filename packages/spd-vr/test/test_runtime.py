from pathlib import Path

from spd_vr.recorder import validate_episode_path
from spd_vr.runtime import _LiveTickPacer, run_runtime


def test_hardware_free_runtime_writes_schema_v1_episode(tmp_path):
    episode = run_runtime(output=tmp_path, duration_s=0.1, mock=True)
    assert episode == tmp_path / "episode_000001.h5"
    assert (tmp_path / "dataset_config.json").is_file()
    assert validate_episode_path(episode)["valid"] is True


def test_runtime_rejects_non_mock_live_mode(tmp_path):
    try:
        run_runtime(output=tmp_path, duration_s=0.1, mock=False)
    except RuntimeError as exc:
        assert "hardware-free" in str(exc)
    else:
        raise AssertionError("live runtime must not be available")


def test_absolute_tick_pacer_does_not_catch_up():
    class Clock:
        now = 0

        def __call__(self):
            return self.now

        def sleep(self, seconds):
            self.now += int(seconds * 1_000_000_000)

    clock = Clock()
    pacer = _LiveTickPacer(1_000_000, clock, clock.sleep)
    assert pacer.wait() == 0
    clock.now = 5_000_000
    assert pacer.wait() == 5_000_000
    assert pacer.deadline_ns == 6_000_000
