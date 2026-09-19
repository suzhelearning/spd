from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from spd_vr.alignment import SideAlignment
from spd_vr.palm_mapping import PalmMapping


URDF = Path(__file__).resolve().parents[3] / "assets/tianji_wuji2/tianji_wuji2.urdf"


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    skew = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    return np.eye(3) + np.sin(angle) * skew + (1.0 - np.cos(angle)) * (skew @ skew)


def pose(position=(0.0, 0.0, 0.0), orientation=None):
    result = np.eye(4)
    result[:3, 3] = position
    if orientation is not None:
        result[:3, :3] = orientation
    return result


def standard_inputs(heading):
    wrists = {
        "left": pose((0.3, 0.2, 1.1), heading @ rotation((1, 0, 0), 0.7)),
        "right": pose((0.3, -0.2, 1.1), heading @ rotation((0, 1, 0), -0.6)),
    }
    palms = {side: wrist[:3, 3] + heading @ np.array([0.09, 0.0, 0.0]) for side, wrist in wrists.items()}
    return wrists, palms


def test_urdf_palm_axes_are_paired_physical_right_handed_frames():
    mapping = PalmMapping.from_urdf(URDF)
    root = ET.parse(URDF).getroot()
    for side, prefix in (("left", "l"), ("right", "r")):
        def point(finger):
            origin = root.find(f"joint[@name='{prefix}_{finger}_mcp_flex']/origin")
            return np.array([float(value) for value in origin.attrib["xyz"].split()])

        transform = mapping.robot_wrist_to_palm[side]
        palm_from_wrist = np.linalg.inv(transform)
        middle = point("middle_finger")
        local_middle = palm_from_wrist @ np.r_[middle, 1.0]
        # Wrist and middle MCP straddle palm center along forward, not dorsal.
        np.testing.assert_allclose(local_middle[:3], (0.5 * np.linalg.norm(middle), 0, 0), atol=1e-12)
        transverse = transform[:3, :3].T @ (point("index_finger") - point("pinky"))
        assert transverse[1] > 0 if side == "right" else transverse[1] < 0
        np.testing.assert_allclose(transform[:3, :3].T @ transform[:3, :3], np.eye(3), atol=1e-12)
        assert np.linalg.det(transform[:3, :3]) == pytest.approx(1.0)
        # Canonical CAD has mirrored palmar normals along wrist-local Y.
        assert transform[1, 2] < -0.9 if side == "right" else transform[1, 2] > 0.9


@pytest.mark.parametrize("yaw", [-np.pi / 2, np.pi / 2])
@pytest.mark.parametrize("side", ["left", "right"])
def test_frozen_head_yaw_maps_forward_left_up_and_standard_palms_down(yaw, side):
    mapping = PalmMapping.from_urdf(URDF)
    heading = rotation((0, 0, 1), yaw)
    wrists, palms = standard_inputs(heading)
    # Head pitch/roll must not tilt the operator axes.
    head = pose(orientation=heading @ rotation((0, 1, 0), 0.3) @ rotation((1, 0, 0), -0.2))
    mapping.calibrate(head, wrists, palms, epoch=7)
    robot_wrist = pose((0.4, 0.2, 0.8), rotation((1, 1, 0), 0.8))
    alignment = SideAlignment(neutral_robot=robot_wrist, stable_frames=1)
    mapping.configure_alignment(side, alignment)
    neutral = alignment.accept(wrists[side], True, 7, 10_000_000)
    robot_offset = mapping.robot_wrist_to_palm[side]
    neutral_palm = neutral.target_pose @ robot_offset
    # Initial calibrated orientation is anatomical, NOT initial robot wrist.
    np.testing.assert_allclose(neutral_palm[:3, :3], np.eye(3), atol=1e-12)
    np.testing.assert_allclose(neutral_palm[:3, 3], (robot_wrist @ robot_offset)[:3, 3], atol=1e-12)
    displacement = np.array([0.02, -0.03, 0.01])
    moved = wrists[side].copy()
    moved[:3, 3] += heading @ displacement
    # No subsequent head data: heading is fixed by explicit calibration.
    result = alignment.accept(moved, True, 7, 110_000_000)
    assert result.valid
    target_palm = result.target_pose @ robot_offset
    np.testing.assert_allclose(target_palm[:3, 3] - neutral_palm[:3, 3], displacement, atol=1e-12)
    np.testing.assert_allclose(target_palm[:3, :3], np.eye(3), atol=1e-12)


@pytest.mark.parametrize("side", ["left", "right"])
def test_local_palm_offsets_rotate_with_both_human_and_robot_wrists(side):
    mapping = PalmMapping.from_urdf(URDF)
    wrists, palms = standard_inputs(np.eye(3))
    mapping.calibrate(np.eye(4), wrists, palms, epoch=1)
    alignment = SideAlignment(neutral_robot=pose((0.4, 0.2, 0.8)), stable_frames=1)
    mapping.configure_alignment(side, alignment)
    first = alignment.accept(wrists[side], True, 1, 10_000_000)
    robot_offset = mapping.robot_wrist_to_palm[side]
    first_palm = first.target_pose @ robot_offset
    turn = rotation((0, 0, 1), np.pi / 2)
    moved = wrists[side].copy()
    moved[:3, :3] = turn @ moved[:3, :3]
    # Rotate wrist around a fixed anatomical palm point, not its own origin.
    moved[:3, 3] = palms[side] - turn @ (palms[side] - wrists[side][:3, 3])
    result = alignment.accept(moved, True, 1, 110_000_000)
    assert result.valid
    target_palm = result.target_pose @ robot_offset
    np.testing.assert_allclose(target_palm[:3, 3], first_palm[:3, 3], atol=1e-12)
    np.testing.assert_allclose(target_palm[:3, :3], turn, atol=1e-12)
    assert np.linalg.norm(result.target_pose[:3, 3] - first.target_pose[:3, 3]) > 0.02
    expected_wrist = first_palm[:3, 3] - result.target_pose[:3, :3] @ robot_offset[:3, 3]
    np.testing.assert_allclose(result.target_pose[:3, 3], expected_wrist, atol=1e-12)


@pytest.mark.parametrize("reset_method", ["realign", "reset"])
def test_position_realign_preserves_anatomical_orientation_calibration(reset_method):
    mapping = PalmMapping.from_urdf(URDF)
    heading = rotation((0, 0, 1), 0.8)
    wrists, palms = standard_inputs(heading)
    mapping.calibrate(pose(orientation=heading), wrists, palms, epoch=4)
    alignment = SideAlignment(stable_frames=1)
    mapping.configure_alignment("left", alignment)
    alignment.accept(wrists["left"], True, 4, 10_000_000)
    turned = wrists["left"].copy()
    turned[:3, :3] = heading @ rotation((1, 0, 0), 0.4) @ heading.T @ turned[:3, :3]
    turned[:3, 3] += (0.3, -0.2, 0.1)
    alignment.neutral_robot = pose((0.5, 0.2, 0.6), rotation((0, 1, 0), -0.9))
    getattr(alignment, reset_method)()
    result = alignment.accept(turned, True, 4, 110_000_000)
    assert result.valid
    robot_offset = mapping.robot_wrist_to_palm["left"]
    palm = result.target_pose @ robot_offset
    np.testing.assert_allclose(palm[:3, :3], rotation((1, 0, 0), 0.4), atol=1e-12)
    np.testing.assert_allclose(palm[:3, 3], (alignment.neutral_robot @ robot_offset)[:3, 3], atol=1e-12)
    assert mapping.calibrated and mapping.epoch == 4


def test_degenerate_heading_and_partial_calibration_cannot_replace_valid_mapping():
    mapping = PalmMapping.from_urdf(URDF)
    wrists, palms = standard_inputs(np.eye(3))
    mapping.calibrate(np.eye(4), wrists, palms, epoch=1)
    with pytest.raises(ValueError, match="heading"):
        mapping.calibrate(pose(orientation=rotation((0, 1, 0), np.pi / 2)), wrists, palms, epoch=2)
    with pytest.raises(ValueError, match="right palm"):
        mapping.calibrate(np.eye(4), wrists, {**palms, "right": np.full(3, np.nan)}, epoch=2)
    assert mapping.epoch == 1
    alignment = SideAlignment(stable_frames=1)
    mapping.configure_alignment("left", alignment)
    result = alignment.accept(wrists["left"], True, 1, 10_000_000)
    np.testing.assert_allclose((result.target_pose @ mapping.robot_wrist_to_palm["left"])[:3, :3], np.eye(3), atol=1e-12)
    mapping.reset()
    assert not mapping.calibrated and mapping.epoch is None
    with pytest.raises(ValueError, match="calibration"):
        mapping.configure_alignment("left", alignment)


def test_mcp_geometry_rejects_moving_link_parent(tmp_path):
    root = ET.parse(URDF)
    parent = root.getroot().find("joint[@name='r_middle_finger_mcp_flex']/parent")
    parent.set("link", "r_index_finger_proximal")
    invalid_urdf = tmp_path / "invalid.urdf"
    root.write(invalid_urdf)
    with pytest.raises(ValueError, match="rooted directly"):
        PalmMapping.from_urdf(invalid_urdf)
