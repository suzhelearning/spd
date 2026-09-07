import numpy as np

from spd_vr.alignment import PICO_TO_ROBOT_ROTATION, SideAlignment


def pose(x=0.0, y=0.0, z=0.0, angle=0.0):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s, 0.0, x], [s, c, 0.0, y], [0.0, 0.0, 1.0, z], [0.0, 0.0, 0.0, 1.0]])


def test_alignment_requires_ten_frames_and_uses_explicit_transform():
    robot_neutral = pose(1.0, -2.0, 0.5, 0.2)
    pico_neutral = pose(0.2, 0.3, 0.4, -0.3)
    alignment = SideAlignment(neutral_robot=robot_neutral)

    for timestamp in range(1, 10):
        result = alignment.accept(pico_neutral, True, 1, timestamp)
        assert not result.aligned
    result = alignment.accept(pico_neutral, True, 1, 10)

    assert result.aligned
    np.testing.assert_allclose(
        alignment.transform,
        robot_neutral @ np.linalg.inv(pico_neutral),
    )
    np.testing.assert_allclose(result.target_pose, robot_neutral)


def test_pico_world_axes_map_to_robot_world_axes_after_alignment():
    alignment = SideAlignment(pico_to_robot_rotation=PICO_TO_ROBOT_ROTATION)
    for timestamp in range(1, 11):
        alignment.accept(np.eye(4), True, 1, timestamp)

    current = np.eye(4)
    current[:3, 3] = (0.005, 0.004, 0.003)
    angle = 0.05
    current[:3, :3] = (
        (1.0, 0.0, 0.0),
        (0.0, np.cos(angle), -np.sin(angle)),
        (0.0, np.sin(angle), np.cos(angle)),
    )
    target = alignment.accept(current, True, 1, 11).target_pose

    np.testing.assert_allclose(
        target[:3, 3],
        PICO_TO_ROBOT_ROTATION @ current[:3, 3],
    )
    np.testing.assert_allclose(
        target[:3, :3],
        PICO_TO_ROBOT_ROTATION @ current[:3, :3] @ PICO_TO_ROBOT_ROTATION.T,
    )

def test_jump_resets_window_but_holds_last_target_and_sides_are_independent():
    left = SideAlignment(neutral_robot=pose(1.0))
    right = SideAlignment(neutral_robot=pose(-1.0))
    for timestamp in range(1, 11):
        assert left.accept(pose(), True, 1, timestamp).stable_count == min(timestamp, 10)
        right.accept(pose(), True, 1, timestamp)
    left.accept(pose(x=0.01), True, 1, 11)
    steady = left.accept(pose(x=0.0201), True, 1, 12)
    jumped = left.accept(pose(x=0.041), True, 1, 13)
    inactive = right.accept(pose(), False, 1, 12)

    assert steady.aligned
    assert not jumped.aligned
    assert jumped.hold_reason == "aligning"
    np.testing.assert_allclose(jumped.target_pose, steady.target_pose)
    assert inactive.hold_reason == "inactive"
    assert not right.aligned


def test_duplicate_timestamp_does_not_advance_stability():
    alignment = SideAlignment()
    assert alignment.accept(pose(), True, 1, 1).stable_count == 1
    assert alignment.accept(pose(), True, 1, 1).hold_reason == "aligning"
    assert alignment.stable_count == 1
def test_duplicate_stale_timestamp_resets_and_fresh_frame_realigns():
    alignment = SideAlignment(stale_after_ns=50)
    for timestamp in range(1, 11):
        alignment.accept(pose(), True, 1, timestamp)
    duplicate = alignment.accept(pose(), True, 1, 10, now_ns=20)
    assert duplicate.aligned
    stale = alignment.accept(pose(), True, 1, 10, now_ns=61)
    assert stale.hold_reason == "stale"
    assert not alignment.aligned
    assert alignment.accept(pose(), True, 1, 11, now_ns=11).hold_reason == "aligning"



def test_epoch_timestamp_stale_realign_and_reset_hold_last_target():
    alignment = SideAlignment(neutral_robot=pose(2.0), stale_after_ns=50)
    for timestamp in range(1, 11):
        result = alignment.accept(pose(), True, 1, timestamp)
    target = result.target_pose.copy()

    stale = alignment.accept(pose(), True, 1, 20, now_ns=71)
    assert stale.hold_reason == "stale"
    np.testing.assert_allclose(stale.target_pose, target)

    epoch = alignment.accept(pose(), True, 2, 21)
    assert epoch.hold_reason == "epoch_change"
    assert not epoch.aligned
    rollback = alignment.accept(pose(), True, 2, 20)
    assert rollback.hold_reason == "timestamp_rollback"

    alignment.realign()
    assert not alignment.aligned
    assert alignment.accept(pose(), True, 2, 22).hold_reason == "aligning"
    alignment.reset()
    assert alignment.last_target is None


def test_stationary_hold_rejects_wrist_jitter_without_delaying_real_motion():
    alignment = SideAlignment()
    frame_ns = 16_666_667
    for index in range(10):
        result = alignment.accept(
            pose(), True, 1, (index + 1) * frame_ns,
        )
    assert result.valid

    outputs = []
    for index in range(20):
        sign = -1.0 if index % 2 else 1.0
        jitter = pose(x=sign * 0.0008, angle=sign * 0.002)
        result = alignment.accept(
            jitter, True, 1, (index + 11) * frame_ns,
        )
        outputs.append(result.target_pose)

    for output in outputs[-5:]:
        np.testing.assert_allclose(output, outputs[-1], atol=1.0e-12)

    moved = alignment.accept(
        pose(x=0.015, angle=0.05), True, 1, 31 * frame_ns,
    )
    assert moved.valid
    assert moved.target_pose[0, 3] > 0.01
