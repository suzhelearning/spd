import numpy as np

from spd_vr.arm_target_protocol import ArmTargetFrame, ArmTargetHoldReason, RIGHT_VALID
from spd_vr.session_state import SessionController
from spd_vr.viewer import PlantController
from spd_vr.wire import ControlCommand, ControlFrame, TrackingFrame


def arm_frame(plant, sequence, valid_mask):
    left = tuple(float(value) for value in plant._home[:7])
    right = tuple(0.25 for _ in range(7))
    return ArmTargetFrame(
        sequence=sequence,
        tracking_epoch=1,
        source_timestamp_ns=1,
        control_timestamp_ns=1,
        valid_mask=valid_mask,
        left_hold_reason=ArmTargetHoldReason.SOLVER_FAILURE if not valid_mask & 1 else ArmTargetHoldReason.NONE,
        right_hold_reason=ArmTargetHoldReason.NONE if valid_mask & 2 else ArmTargetHoldReason.SOLVER_FAILURE,
        left_q=left,
        right_q=right,
        left_qdot=(0.0,) * 7,
        right_qdot=(0.0,) * 7,
    )


def test_plant_keeps_valid_side_and_holds_stale_side_locally():
    plant = PlantController.synthetic_fixture()
    plant.submit_arm_target(arm_frame(plant, 1, RIGHT_VALID), now_ns=10_000_000_000)
    step = plant.physics_tick(10_000_000_000)
    assert step.arm_valid_mask == RIGHT_VALID
    right_ids = [
        plant._actuator_ids[joint.actuator]
        for joint in plant.joints
        if joint.side == "right" and joint.group == "arm"
    ]
    left_ids = [
        plant._actuator_ids[joint.actuator]
        for joint in plant.joints
        if joint.side == "left" and joint.group == "arm"
    ]
    right_ctrl = plant.data.ctrl[right_ids].copy()
    np.testing.assert_allclose(right_ctrl, 0.25)
    np.testing.assert_array_equal(plant.data.ctrl[left_ids], 0.0)
    stale = plant.physics_tick(10_000_000_001 + 50_000_001)
    assert stale.arm_valid_mask == 0
    np.testing.assert_array_equal(plant.data.ctrl[right_ids], right_ctrl)
    plant.close()

def test_fresh_alignment_requires_valid_arm_and_hand_from_new_generation():
    class Retarget:
        def retarget(self, _frame):
            return type(
                "Target",
                (),
                {
                    "left_qpos": np.full(20, 0.1),
                    "right_qpos": np.full(20, 0.1),
                    "left_valid": True,
                    "right_valid": True,
                    "left_hold_reason": "none",
                    "right_hold_reason": "none",
                },
            )()

    hand = np.zeros((26, 7), dtype=np.float32)
    hand[:, 6] = 1.0
    tracking = TrackingFrame(
        sequence=1,
        tracking_epoch=7,
        source_timestamp_ns=1,
        bridge_monotonic_ns=1,
        left_active=True,
        right_active=True,
        head_valid=True,
        left_scale=1.0,
        right_scale=1.0,
        head_pose=np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]),
        left_hand=hand,
        right_hand=hand,
    )
    plant = PlantController.synthetic_fixture(hand_retargeter=Retarget())
    session = SessionController(plant)
    session.apply(ControlFrame(1, 1, ControlCommand.START))
    plant.submit_tracking(tracking, now_ns=2)
    plant.physics_tick(2)
    assert plant.requires_fresh_alignment
    plant.submit_arm_target(arm_frame(plant, 1, 3), now_ns=3)
    step = plant.physics_tick(3)
    assert not plant.requires_fresh_alignment
    assert step.arm_valid_mask == 3
    assert step.hand_valid_mask == 3
    plant.close()


def test_pause_freezes_full_mjdata_and_reset_restores_home():
    plant = PlantController.synthetic_fixture()
    plant.submit_arm_target(arm_frame(plant, 1, RIGHT_VALID), now_ns=1)
    plant.physics_tick(1)
    plant.set_paused(True)
    time = plant.data.time
    qpos, qvel, ctrl = plant.data.qpos.copy(), plant.data.qvel.copy(), plant.data.ctrl.copy()
    plant.physics_tick(2)
    assert plant.data.time == time
    np.testing.assert_array_equal(plant.data.qpos, qpos)
    np.testing.assert_array_equal(plant.data.qvel, qvel)
    np.testing.assert_array_equal(plant.data.ctrl, ctrl)
    plant.reset_home()
    assert plant.data.time == 0.0
    np.testing.assert_array_equal(plant.data.qvel, 0.0)
    np.testing.assert_array_equal(plant.data.ctrl, 0.0)
    assert np.all(np.isfinite(plant.data.qpos))
    plant.close()
