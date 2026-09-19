import numpy as np
import pytest

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


def test_flu_wrist_motion_preserves_robot_forward_left_up_axes():
    alignment = SideAlignment(pico_to_robot_rotation=PICO_TO_ROBOT_ROTATION)
    frame_ns = 16_666_667
    for timestamp in range(1, 11):
        alignment.accept(np.eye(4), True, 1, timestamp * frame_ns)

    # Input protocol is already FLU, as is the robot world: no Unity remapping.
    current = pose(0.005, 0.004, 0.003, 0.05)
    target = alignment.accept(current, True, 1, 11 * frame_ns)
    assert target.valid
    np.testing.assert_allclose(target.target_pose[:3, 3], (0.005, 0.004, 0.003))
    # A positive yaw about input up remains a positive yaw about robot up.
    np.testing.assert_allclose(target.target_pose[:3, :3], current[:3, :3])

def test_jump_resets_window_but_holds_last_target_and_sides_are_independent():
    left = SideAlignment(neutral_robot=pose(1.0))
    right = SideAlignment(neutral_robot=pose(-1.0))
    frame_ns = 16_666_667
    for timestamp in range(1, 11):
        assert left.accept(pose(), True, 1, timestamp * frame_ns).stable_count == min(timestamp, 10)
        right.accept(pose(), True, 1, timestamp * frame_ns)
    left.accept(pose(x=0.01), True, 1, 11 * frame_ns)
    steady = left.accept(pose(x=0.0201), True, 1, 12 * frame_ns)
    jumped = left.accept(pose(x=0.4), True, 1, 13 * frame_ns)
    inactive = right.accept(pose(), False, 1, 12 * frame_ns)

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


@pytest.mark.parametrize("frame_ns", [11_111_111, 16_666_667, 33_333_333])
def test_running_rapid_motion_preserves_neutral_at_different_sample_rates(frame_ns):
    alignment = SideAlignment()
    neutral = pose(x=0.2, y=-0.1, angle=0.3)
    for index in range(1, 11):
        alignment.accept(neutral, True, 1, index * frame_ns)

    # 3 m/s and 14 rad/s exceed calibration steps even at 90 Hz, but are
    # plausible input wrist motion. The target follows without a new neutral.
    for index in range(1, 4):
        elapsed_s = index * frame_ns * 1.0e-9
        current = pose(x=0.2 + 3.0 * elapsed_s, y=-0.1, angle=0.3 + 14.0 * elapsed_s)
        result = alignment.accept(current, True, 1, (10 + index) * frame_ns)
        assert result.valid
        np.testing.assert_allclose(
            result.target_pose,
            pose(x=3.0 * elapsed_s, angle=14.0 * elapsed_s),
            atol=1.0e-12,
        )


@pytest.mark.parametrize("current", [pose(x=0.06), pose(angle=0.3)])
def test_running_rejects_discontinuity_using_elapsed_sample_time(current):
    # The same displacement is plausible in 20 ms, not in 10 ms:
    # respectively 3 vs 6 m/s, or 15 vs 30 rad/s.
    for frame_ns, accepted in [(20_000_000, True), (10_000_000, False)]:
        alignment = SideAlignment()
        for index in range(1, 11):
            previous = alignment.accept(pose(), True, 1, index * frame_ns)
        result = alignment.accept(current, True, 1, 11 * frame_ns)
        assert result.valid is accepted
        assert alignment.aligned is accepted
        if accepted:
            np.testing.assert_allclose(result.target_pose, current)
        else:
            assert result.hold_reason == "aligning"
            np.testing.assert_allclose(result.target_pose, previous.target_pose)
            # A rejected discontinuity starts a fresh ten-frame stable window.
            for index in range(12, 20):
                assert not alignment.accept(current, True, 1, index * frame_ns).aligned
            recovered = alignment.accept(current, True, 1, 20 * frame_ns)
            assert recovered.valid
            np.testing.assert_allclose(recovered.target_pose, np.eye(4), atol=1e-12)


@pytest.mark.parametrize("unstable", [pose(x=0.021), pose(angle=0.16)])
def test_calibration_still_restarts_stable_window_on_small_fast_steps(unstable):
    alignment = SideAlignment()
    frame_ns = 16_666_667
    for index in range(1, 10):
        assert not alignment.accept(pose(), True, 1, index * frame_ns).aligned
    # This step would be allowed in RUNNING, but is not a stable neutral.
    result = alignment.accept(unstable, True, 1, 10 * frame_ns)
    assert not result.aligned
    assert result.stable_count == 1
    for index in range(11, 19):
        assert not alignment.accept(unstable, True, 1, index * frame_ns).aligned
    result = alignment.accept(unstable, True, 1, 19 * frame_ns)
    assert result.valid
    np.testing.assert_allclose(result.target_pose, np.eye(4), atol=1e-12)


def test_anatomical_mapping_retains_timing_guards_and_absolute_orientation_after_jump():
    alignment = SideAlignment(neutral_robot=pose(0.5, angle=-0.4))
    robot_offset = pose(0.04, angle=0.7)
    alignment.configure_palm_mapping(np.eye(3), pose(0.08), robot_offset)
    frame_ns = 16_666_667
    for index in range(1, 11):
        previous = alignment.accept(pose(), True, 3, index * frame_ns)
    assert previous.valid
    np.testing.assert_allclose((previous.target_pose @ robot_offset)[:3, :3], np.eye(3), atol=1e-12)
    duplicate = alignment.accept(pose(x=0.4), True, 3, 10 * frame_ns)
    np.testing.assert_allclose(duplicate.target_pose, previous.target_pose)
    jumped = pose(x=0.4, angle=0.8)
    rejected = alignment.accept(jumped, True, 3, 11 * frame_ns)
    assert not rejected.valid
    np.testing.assert_allclose(rejected.target_pose, previous.target_pose)
    for index in range(12, 20):
        assert not alignment.accept(jumped, True, 3, index * frame_ns).valid
    recovered = alignment.accept(jumped, True, 3, 20 * frame_ns)
    assert recovered.valid
    # A fresh position baseline must not turn a new hand orientation into
    # the robot's old neutral orientation, even after a tracking discontinuity.
    palm = recovered.target_pose @ robot_offset
    np.testing.assert_allclose(palm[:3, :3], jumped[:3, :3], atol=1e-12)
    np.testing.assert_allclose(palm[:3, 3], (alignment.neutral_robot @ robot_offset)[:3, 3], atol=1e-12)
    stale = alignment.accept(jumped, True, 3, 20 * frame_ns, now_ns=24 * frame_ns)
    assert stale.hold_reason == "stale"
    np.testing.assert_allclose(stale.target_pose, recovered.target_pose)


def test_anatomical_stationary_hold_freezes_jitter_then_releases_real_rotation():
    alignment = SideAlignment()
    robot_offset = pose(0.04, angle=0.7)
    alignment.configure_palm_mapping(np.eye(3), pose(0.08), robot_offset)
    frame_ns = 16_666_667
    for index in range(1, 11):
        alignment.accept(pose(), True, 1, index * frame_ns)
    outputs = []
    for index in range(11, 31):
        sign = -1 if index % 2 else 1
        result = alignment.accept(pose(x=sign * 0.0008, angle=sign * 0.002), True, 1, index * frame_ns)
        outputs.append(result.target_pose)
    for output in outputs[-5:]:
        np.testing.assert_allclose(output, outputs[-1], atol=1e-12)
    moved = alignment.accept(pose(x=0.015, angle=0.05), True, 1, 31 * frame_ns)
    assert moved.valid
    np.testing.assert_allclose((moved.target_pose @ robot_offset)[:3, :3], pose(angle=0.05)[:3, :3], atol=1e-12)
