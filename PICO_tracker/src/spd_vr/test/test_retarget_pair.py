from types import SimpleNamespace

import numpy as np

from spd_vr.pico_hands import PICO_TO_MEDIAPIPE, PicoHandFrame
from spd_vr.retarget_pair import HandHoldReason, WujiRetargetPair


class FakeRetargeter:
    def __init__(self, *, fail=False):
        self.fail = fail
        names = [f"joint_{index}" for index in range(20)]
        self.optimizer = SimpleNamespace(robot=SimpleNamespace(dof_joint_names=names, joint_limits=np.zeros((20, 2))))

    def reset_filter(self, *_args):
        return None

    def retarget(self, points):
        assert np.asarray(points).shape == (21, 3)
        if self.fail:
            raise RuntimeError("left solver failure")
        return np.arange(20, dtype=np.float64)


def _frame(*, left_active=True, right_active=True, sequence_id=1):
    left = np.zeros((26, 7), dtype=np.float64)
    right = np.zeros((26, 7), dtype=np.float64)
    left[:, 6] = right[:, 6] = 1.0
    for joint in range(26):
        left[joint, :3] = (0.01 * joint, 0.001 * joint, 0.002 * joint)
        right[joint, :3] = (-0.01 * joint, 0.001 * joint, 0.002 * joint)
    return PicoHandFrame(
        left,
        right,
        left_active=left_active,
        right_active=right_active,
        tracking_epoch=1,
        sequence_id=sequence_id,
        timestamp_ns=100 + sequence_id,
    )


def test_fixed_pico_to_mediapipe_mapping_and_exact_20_dof_outputs():
    assert PICO_TO_MEDIAPIPE.tolist() == [1, 2, 3, 4, 5, 7, 8, 9, 10, 12, 13, 14, 15, 17, 18, 19, 20, 22, 23, 24, 25]
    names = [f"joint_{index}" for index in range(20)]
    pair = WujiRetargetPair(
        FakeRetargeter(),
        FakeRetargeter(),
        left_actuator_joint_names=names,
        right_actuator_joint_names=names,
    )

    result = pair.retarget(_frame())

    assert result.left_valid is True
    assert result.right_valid is True
    assert result.left_qpos.shape == (20,)
    assert result.right_qpos.shape == (20,)
    assert result.left_hold_reason is HandHoldReason.NONE
    assert result.right_hold_reason is HandHoldReason.NONE


def test_failed_or_inactive_left_side_does_not_block_right_side():
    names = [f"joint_{index}" for index in range(20)]
    pair = WujiRetargetPair(
        FakeRetargeter(fail=True),
        FakeRetargeter(),
        left_actuator_joint_names=names,
        right_actuator_joint_names=names,
    )

    result = pair.retarget(_frame())

    assert result.left_valid is False
    assert result.left_hold_reason is HandHoldReason.SOLVER_FAILURE
    assert result.right_valid is True
    assert result.right_hold_reason is HandHoldReason.NONE

    inactive = pair.retarget(_frame(left_active=False, right_active=True, sequence_id=2))
    assert inactive.left_hold_reason is HandHoldReason.INACTIVE
    assert inactive.right_valid is True


class BoundedFakeRetargeter(FakeRetargeter):
    def __init__(self):
        super().__init__()
        self.reset_count = 0
        self.optimizer.robot.joint_limits = np.column_stack(
            (-np.ones(20), np.ones(20))
        )

    def reset_filter(self, *_args):
        self.reset_count += 1

    def retarget(self, points):
        return np.linspace(-2.0, 2.0, 20)


def test_manifest_order_clamps_finite_outputs_and_resets_each_epoch():
    left = BoundedFakeRetargeter()
    right = BoundedFakeRetargeter()
    source_names = [f"joint_{index}" for index in range(20)]
    actuator_names = list(reversed(source_names))
    pair = WujiRetargetPair(
        left,
        right,
        left_actuator_joint_names=actuator_names,
        right_actuator_joint_names=actuator_names,
    )

    first = pair.retarget(_frame(sequence_id=1))
    assert np.isfinite(first.left_qpos).all()
    assert np.all(np.abs(first.left_qpos) <= 1.0)
    assert first.left_qpos[0] == 1.0
    assert first.left_qpos[-1] == -1.0
    assert left.reset_count == right.reset_count == 1

    second = pair.retarget(_frame(sequence_id=2))
    assert second.left_valid and second.right_valid
    assert left.reset_count == right.reset_count == 1
    epoch_reset = pair.retarget(
        PicoHandFrame(
            _frame(sequence_id=3).left_hand,
            _frame(sequence_id=3).right_hand,
            tracking_epoch=2,
            sequence_id=3,
        )
    )
    assert epoch_reset.left_valid and epoch_reset.right_valid
    assert left.reset_count == right.reset_count == 2
