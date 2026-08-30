from pathlib import Path

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
