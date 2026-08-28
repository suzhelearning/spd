from pathlib import Path
from types import SimpleNamespace
import time

import numpy as np

import pytest
from spd_vr import simulator as simulator_module
from spd_vr.arm_target_protocol import ArmTargetFrame, ArmTargetHoldReason, RIGHT_VALID
from spd_vr.simulator import CameraRequest, UnifiedSimulator


ROOT = Path(__file__).parents[1]
MODEL = ROOT / "generated" / "tianji_wuji2_spd.xml"
MANIFEST = ROOT / "generated" / "joint_manifest.yaml"


def arm_frame(simulator: UnifiedSimulator, sequence: int, valid_mask: int) -> ArmTargetFrame:
    left_q = tuple(float(entry.range[0] + entry.range[1]) / 2.0 for entry in simulator.joints[:7])
    right_q = tuple(float(entry.range[0] + entry.range[1]) / 2.0 for entry in simulator.joints[27:34])
    return ArmTargetFrame(
        sequence=sequence,
        tracking_epoch=1,
        source_timestamp_ns=1,
        control_timestamp_ns=1,
        valid_mask=valid_mask,
        left_hold_reason=(ArmTargetHoldReason.NONE if valid_mask & 1 else ArmTargetHoldReason.SOLVER_FAILURE),
        right_hold_reason=(ArmTargetHoldReason.NONE if valid_mask & 2 else ArmTargetHoldReason.SOLVER_FAILURE),
        left_q=left_q,
        right_q=right_q,
        left_qdot=(0.0,) * 7,
        right_qdot=(0.0,) * 7,
    )


def test_invalid_left_does_not_block_valid_right_and_stales_locally(monkeypatch):
    simulator = UnifiedSimulator(MODEL, MANIFEST)
    try:
        base = 10_000_000_000
        monkeypatch.setattr(simulator_module.time, "monotonic_ns", lambda: base)
        frame = arm_frame(simulator, 1, RIGHT_VALID)
        before_left = simulator._arm_snapshot.left_q
        snapshot = simulator.on_arm_target(frame, now_ns=base)
        assert snapshot.valid_mask == RIGHT_VALID
        assert snapshot.left_q == before_left
        assert snapshot.right_hold_reason is ArmTargetHoldReason.NONE
        monkeypatch.setattr(
            simulator_module.time,
            "monotonic_ns",
            lambda: base + 50_000_001,
        )

        result = simulator.step()
        assert result.arm_valid_mask == 0
        assert simulator._arm_snapshot.valid_mask == 0
        assert simulator._arm_snapshot.right_hold_reason is ArmTargetHoldReason.INPUT_STALE
        assert simulator._arm_snapshot.right_q == snapshot.right_q
    finally:
        simulator.close()


def test_pause_freezes_tick_time_state_and_rejects_callbacks(monkeypatch):
    simulator = UnifiedSimulator(MODEL, MANIFEST)
    try:
        simulator.step()
        tick = simulator.tick
        sim_time = simulator.data.time
        qpos = simulator.data.qpos.copy()
        callbacks = simulator.arm_callback_count
        simulator.set_paused(True)
        assert simulator._arm_snapshot.left_hold_reason is ArmTargetHoldReason.PAUSED
        simulator.on_arm_target(arm_frame(simulator, 2, RIGHT_VALID), now_ns=1)
        assert simulator.arm_callback_count == callbacks + 1
        for _ in range(3):
            result = simulator.step()
            assert result.tick == tick
            assert result.sim_time_ns == int(round(sim_time * 1e9))
            assert simulator.tick == tick
            assert simulator.data.time == sim_time
            np.testing.assert_array_equal(simulator.data.qpos, qpos)
        simulator.set_paused(False)
        assert simulator._arm_snapshot.valid_mask == 0
        assert simulator._arm_snapshot.left_hold_reason is ArmTargetHoldReason.INPUT_STALE
    finally:
        simulator.close()


def test_pause_blocks_existing_camera_and_recorder_queue_work():
    camera_calls: list[int] = []
    recorder_calls: list[int] = []

    class Camera:
        def capture(self, sim_time_ns):
            camera_calls.append(sim_time_ns)
            return {}

    class Recorder:
        def submit(self, **item):
            recorder_calls.append(item["sim_time_ns"])

    simulator = UnifiedSimulator(
        MODEL,
        MANIFEST,
        camera_provider=Camera(),
        recorder=Recorder(),
    )
    assert simulator.camera_drop_count == 0
    try:
        simulator.set_paused(True)
        simulator._camera_queue.put(
            CameraRequest(1, simulator.data.qpos.copy(), simulator.data.qvel.copy())
        )
        simulator._recorder_queue.put({"sim_time_ns": 1})
        for _ in range(3):
            simulator.step()
        time.sleep(0.05)
        assert camera_calls == []
        assert recorder_calls == []
        assert simulator._camera_queue.qsize() == 1
        assert simulator._recorder_queue.qsize() == 1
        simulator.set_paused(False)
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline and (not camera_calls or not recorder_calls):
            time.sleep(0.01)
        assert camera_calls == [1]
        assert recorder_calls == [1]
    finally:
        simulator.close()

def test_pause_barrier_timeout_does_not_enter_paused_state():
    class Camera:
        def capture(self, sim_time_ns):
            return {}

    class Recorder:
        def submit(self, **item):
            return None

    class AckThatNeverCompletes:
        def clear(self):
            return None

        def wait(self, timeout):
            return False

    simulator = UnifiedSimulator(
        MODEL,
        MANIFEST,
        camera_provider=Camera(),
        recorder=Recorder(),
    )
    simulator._camera_pause_ack = AckThatNeverCompletes()
    simulator._recorder_pause_ack = AckThatNeverCompletes()
    try:
        with pytest.raises(RuntimeError, match="pause barrier timeout"):
            simulator.set_paused(True)
        assert simulator.paused is False
        assert simulator._worker_pause.is_set() is False
    finally:
        simulator.close()
