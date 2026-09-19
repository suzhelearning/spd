"""Source interlock regressions; production solvers are exercised by live smoke."""
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

from pico_hand_tracking.protocol import Hand, HandFrame, Joint, Pose
from spd_vr.arm_ik import build_synthetic_fixture
from spd_vr.pico_ros_source import PicoRosSourceCore, PicoTeleopCore
from spd_vr.ros_joint_command import ARMS_READY, LEFT_HAND_READY, RIGHT_HAND_READY, JOINT_NAMES
from spd_vr.wire import decode_tracking


def raw_frame(timestamp=1, *, left=True, right=True):
    pose = Pose((0.1, 0.2, 0.3), (0.0, 0.0, 0.0, 1.0))
    joints = tuple(Joint(pose.position, pose.quaternion_xyzw, index=i) for i in range(26))
    return HandFrame(timestamp, 7, pose, Hand(left, pose, joints), Hand(right, pose, joints))


class HandTargets:
    """Deterministic hand solver output to isolate source interlocks/rate limits."""

    def reset_filter(self):
        pass

    def retarget(self, frame):
        return SimpleNamespace(left_qpos=np.ones(20), right_qpos=-np.ones(20),
                               left_valid=frame.left_active, right_valid=frame.right_active)


@pytest.fixture
def source():
    clock = [1_000_000_000]
    arm, _, _ = build_synthetic_fixture()
    manifest = {"joints": [dict(joint=name, side="left" if i < 7 or 14 <= i < 34 else "right",
                               group="arm" if i < 14 else "hand", range=[-3.2, 3.2], velocity_limit=2.0)
                           for i, name in enumerate(JOINT_NAMES)]}
    core = PicoTeleopCore(arm, HandTargets(), manifest, clock_ns=lambda: clock[0])
    core.connected()
    return core, clock


def sample(core, clock, timestamp, **kwargs):
    clock[0] += 5_000_000
    return core.accept_frame(raw_frame(timestamp, **kwargs), received_ns=clock[0])


def test_ready_requires_align_and_hand_targets_cannot_jump(source):
    core, clock = source
    sample(core, clock, 1)
    held = core.snapshot(stamp_ns=1).position_rad
    assert not core.command("start")
    assert core.command("align")
    for timestamp in range(2, 12):
        sample(core, clock, timestamp)
        core.tick(clock[0])
    assert core.snapshot(stamp_ns=2).ready_mask == 7
    np.testing.assert_array_equal(core.snapshot(stamp_ns=3).position_rad, held)
    assert core.command("start")
    sample(core, clock, 12)
    core.tick(clock[0])
    position = np.asarray(core.snapshot(stamp_ns=4).position_rad)
    assert position[14] > held[14]
    assert np.max(np.abs(position[14:] - np.asarray(held)[14:])) <= 0.5 * 0.005 + 1e-12


def test_group_loss_latches_and_realign_revokes_old_session(source):
    core, clock = source
    sample(core, clock, 1)
    core.command("align")
    for timestamp in range(2, 12):
        sample(core, clock, timestamp)
    core.command("start")
    session = core.snapshot(stamp_ns=1).session_id
    held = core.position.copy()
    sample(core, clock, 12, left=False)
    assert core.ready_mask == RIGHT_HAND_READY
    assert core.running_mask == RIGHT_HAND_READY
    for timestamp in range(13, 25):
        sample(core, clock, timestamp)
    assert not core.ready_mask & (ARMS_READY | LEFT_HAND_READY)
    core.command("align")
    assert core.snapshot(stamp_ns=2).session_id != session
    assert core.running_mask == 0
    np.testing.assert_array_equal(core.position, held)


def test_stale_mailbox_and_clock_rollback_cannot_refresh_readiness(source):
    core, clock = source
    sample(core, clock, 20)
    core.command("align")
    clock[0] += core.FRESH_NS + 1
    assert core.snapshot(stamp_ns=999).ready_mask == 0
    assert not core.accept_frame(raw_frame(20), received_ns=clock[0])
    assert core.snapshot(stamp_ns=1000).ready_mask == 0
    sample(core, clock, 21)
    assert core.ready_mask == 0
    core.command("align")
    sample(core, clock, 10)
    assert core.ready_mask == 0  # device timestamp rollback is a new epoch
    assert not core.command("start")
    core.command("align")
    core.tick(clock[0])
    clock[0] -= 1
    core.tick(clock[0])
    assert core.ready_mask == 0


def test_reconnect_preserves_hold_and_requires_explicit_rearm(source):
    core, clock = source
    sample(core, clock, 1)
    core.command("align")
    core.command("start")
    core.tick(clock[0])
    sample(core, clock, 2)
    core.tick(clock[0])
    held = core.position.copy()
    core.disconnected()
    core.connected()
    sample(core, clock, 1)
    assert not core.command("start")
    assert core.snapshot(stamp_ns=1).ready_mask == 0
    np.testing.assert_array_equal(core.position, held)


def test_malformed_side_invalidates_only_that_side_and_receipt_is_not_relabelled():
    normalizer = PicoRosSourceCore()
    frame = raw_frame()
    broken = replace(frame.left.joints[3], quaternion_xyzw=(0.0, 0.0, 0.0, 0.0))
    frame = replace(frame, left=replace(frame.left, joints=frame.left.joints[:3] + (broken,) + frame.left.joints[4:]))
    tracking = decode_tracking(normalizer.accept_frame(frame, received_ns=100))
    assert not tracking.left_active and tracking.right_active
    assert tracking.bridge_monotonic_ns == 100
    assert normalizer.accept_frame(raw_frame(2), received_ns=99) is None
    recovered = decode_tracking(normalizer.accept_frame(raw_frame(3), received_ns=101))
    assert recovered.tracking_epoch > tracking.tracking_epoch
    assert recovered.bridge_monotonic_ns == 101
