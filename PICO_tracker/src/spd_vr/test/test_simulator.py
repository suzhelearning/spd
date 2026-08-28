from pathlib import Path
from types import SimpleNamespace

import numpy as np

from spd_vr import simulator as simulator_module
from spd_vr.arm_target_protocol import ArmTargetFrame, ArmTargetHoldReason, RIGHT_VALID
from spd_vr.simulator import UnifiedSimulator


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

        monkeypatch.setattr(simulator_module.time, "monotonic_ns", lambda: base + 50_000_001)
        simulator.step()
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
