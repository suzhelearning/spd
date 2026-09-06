from pathlib import Path

import numpy as np
import pinocchio as pin
import pytest

from wuji_retargeting.robot import RobotWrapper


ROOT = Path(__file__).resolve().parents[2]
URDF = ROOT / "assets" / "tianji_wuji2" / "tianji_wuji2.urdf"
LEFT = [
    "l_thumb_cmc_flex", "l_thumb_cmc_abd", "l_thumb_mcp", "l_thumb_ip",
    "l_index_finger_mcp_flex", "l_index_finger_mcp_abd", "l_index_finger_pip", "l_index_finger_dip",
    "l_middle_finger_mcp_flex", "l_middle_finger_mcp_abd", "l_middle_finger_pip", "l_middle_finger_dip",
    "l_ring_finger_mcp_flex", "l_ring_finger_mcp_abd", "l_ring_finger_pip", "l_ring_finger_dip",
    "l_pinky_mcp_flex", "l_pinky_mcp_abd", "l_pinky_pip", "l_pinky_dip",
]
RIGHT = [name.replace("l_", "r_", 1) for name in LEFT]


def test_authoritative_urdf_reduces_to_one_hand_and_rejects_cross_side():
    robot = RobotWrapper(str(URDF), "left", LEFT)
    assert robot.model.nq == robot.model.nv == 20
    assert set(robot.dof_joint_names) == set(LEFT)
    for name in ("l_wrist", "l_index_finger_middle", "l_index_finger_distal", "l_index_finger_tip"):
        assert robot.get_link_index(name) < robot.model.nframes
    assert "Joint1_L" not in robot.dof_joint_names
    assert not set(robot.dof_joint_names) & set(RIGHT)
    with pytest.raises(ValueError):
        RobotWrapper(str(URDF), "left", LEFT[:-1] + ["r_pinky_dip"])


def test_reduced_model_lock_pose_is_pinocchio_neutral():
    robot = RobotWrapper(str(URDF), "right", RIGHT)
    assert robot.data is not None
    assert pin.neutral(robot.model).shape == (20,)


@pytest.mark.parametrize(
    ("side", "active_joints"),
    [("left", LEFT), ("right", RIGHT)],
)
def test_link_positions_and_jacobians_can_be_expressed_in_wrist_frame(
    side, active_joints
):
    robot = RobotWrapper(str(URDF), side, active_joints)
    qpos = robot.joint_limits.mean(axis=1)
    prefix = side[0]
    wrist_id = robot.get_link_index(f"{prefix}_wrist")
    wrist_parent_joint = robot.model.frames[wrist_id].parentJoint
    assert robot.model.nvs[wrist_parent_joint] == 0
    link_ids = [
        wrist_id,
        robot.get_link_index(f"{prefix}_thumb_tip"),
        robot.get_link_index(f"{prefix}_index_finger_tip"),
    ]

    robot.compute_forward_kinematics(qpos)
    wrist_pose = robot.get_link_pose(wrist_id)
    world_positions = np.array(
        [robot.get_link_pose(link_id)[:3, 3] for link_id in link_ids]
    )
    expected = (world_positions - wrist_pose[:3, 3]) @ wrist_pose[:3, :3]

    actual = robot.get_link_positions_in_frame(link_ids, wrist_id)
    jacobians = robot.compute_all_jacobians_batch(
        qpos, link_ids, reference_link_id=wrist_id
    )

    np.testing.assert_allclose(actual, expected, atol=1e-12)
    np.testing.assert_allclose(actual[0], 0.0, atol=1e-12)
    np.testing.assert_allclose(jacobians[0], 0.0, atol=1e-12)

    epsilon = 1e-7
    for joint_index in (0, 5, 10, 15):
        offset = np.zeros_like(qpos)
        offset[joint_index] = epsilon
        robot.compute_forward_kinematics(qpos + offset)
        plus = robot.get_link_positions_in_frame(link_ids, wrist_id)
        robot.compute_forward_kinematics(qpos - offset)
        minus = robot.get_link_positions_in_frame(link_ids, wrist_id)
        finite_difference = (plus - minus) / (2.0 * epsilon)
        np.testing.assert_allclose(
            jacobians[:, :, joint_index],
            finite_difference,
            atol=1e-7,
            rtol=1e-6,
        )
