import numpy as np

from spd_vr.arm_ik import DualArmController, build_synthetic_fixture
from spd_vr.wire import ArmTargetHoldReason, ControlCommand, ControlFrame, TrackingFrame


def frame(sequence=1, timestamp=1_000_000_000):
    hand = np.zeros((26, 7), dtype=np.float32)
    hand[:, 6] = 1.0
    return TrackingFrame(
        sequence=sequence,
        tracking_epoch=1,
        source_timestamp_ns=timestamp,
        bridge_monotonic_ns=timestamp,
        left_active=True,
        right_active=True,
        head_valid=True,
        left_scale=1.0,
        right_scale=1.0,
        head_pose=np.array([0, 0, 1.6, 0, 0, 0, 1], dtype=np.float32),
        left_hand=hand.copy(), right_hand=hand.copy(),
    )


def test_dual_controller_keeps_side_hold_isolated():
    controller, pose_left, pose_right = build_synthetic_fixture()
    controller.accept_tracking(frame())
    for i in range(10):
        controller.accept_tracking(frame(i + 1, 1_000_000_000 + i + 1))
        result = controller.tick(1_000_000_000 + i + 1)
    assert result.left_hold_reason.name == "NONE"
    assert result.right_hold_reason.name == "NONE"

    controller.left_alignment.reset()
    controller.accept_tracking(frame(20, 1_000_000_100))
    held = controller.tick(1_000_000_100)
    assert held.valid_mask & 1 == 0
    assert held.valid_mask & 2 == 2
    np.testing.assert_allclose(held.right_q, controller.right_q)


def test_controller_uses_absolute_deadlines_without_catchup():
    controller, _, _ = build_synthetic_fixture()
    controller.accept_tracking(frame())
    times = iter([0, 5_000_000, 20_000_000, 25_000_000])
    ticks = controller.run(2, clock=lambda: next(times), sleep=lambda _: None)
    assert len(ticks) == 2
    assert controller.tick_count == 2
def test_control_gate_processes_ordered_commands_once():
    controller, _, _ = build_synthetic_fixture()
    pause = ControlFrame(2, 2_000_000_000, ControlCommand.PAUSE)
    assert controller.accept_control(pause)
    assert not controller.accept_control(pause)
    assert not controller.accept_control(ControlFrame(1, 2_000_000_001, ControlCommand.START))
    held = controller.tick(2_000_000_000)
    assert held.left_hold_reason is ArmTargetHoldReason.PAUSED
