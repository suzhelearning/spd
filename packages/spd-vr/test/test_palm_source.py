"""Production mapping lifecycle: actual-state baseline, held preview and rearm."""
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from pico_hand_tracking.protocol import Hand, HandFrame, Joint, Pose
from spd_vr.arm_ik import build_synthetic_fixture
from spd_vr.palm_mapping import PalmMapping
from spd_vr.pico_ros_source import PicoTeleopCore
from spd_vr.ros_joint_command import ARMS_READY, JOINT_NAMES


class HandTargets:
    def reset_filter(self):
        pass

    def retarget(self, frame):
        return SimpleNamespace(left_qpos=np.ones(20), right_qpos=-np.ones(20),
                               left_valid=frame.left_active, right_valid=frame.right_active)


def standard_frame(timestamp, *, yaw=0.0, turn=0.0):
    heading = Rotation.from_euler("z", yaw).as_matrix()
    rotation = heading @ Rotation.from_euler("z", turn).as_matrix()
    quaternion = tuple(Rotation.from_matrix(rotation).as_quat())
    hands = []
    for side, lateral in (("left", 0.2), ("right", -0.2)):
        wrist = heading @ np.array((0.25, lateral, 0.8))
        points = np.tile((0.05, 0.0, 0.0), (26, 1))
        points[0], points[1] = (0.03, 0.0, 0.0), (0.0, 0.0, 0.0)
        sign = -1 if side == "left" else 1
        points[7], points[12], points[22] = (0.07, sign * 0.02, 0), (0.08, 0, 0), (0.05, -sign * 0.035, 0)
        points = points @ rotation.T + wrist
        pose = Pose(tuple(wrist), quaternion)
        hands.append(Hand(True, pose, tuple(Joint(tuple(p), quaternion, index=i) for i, p in enumerate(points))))
    head = Pose((0, 0, 1.6), tuple(Rotation.from_matrix(heading).as_quat()))
    return HandFrame(timestamp, 7, head, *hands)


@pytest.fixture
def mapped_source():
    clock = [1_000_000_000]
    arm, _, _ = build_synthetic_fixture()
    entries = [dict(joint=name, side="left" if i < 7 or 14 <= i < 34 else "right",
                    group="arm" if i < 14 else "hand", range=[-3.2, 3.2], velocity_limit=2.0)
               for i, name in enumerate(JOINT_NAMES)]
    urdf = Path(__file__).resolve().parents[3] / "assets/tianji_wuji2/tianji_wuji2.urdf"
    mapping = PalmMapping.from_urdf(urdf)
    core = PicoTeleopCore(arm, HandTargets(), {"joints": entries}, clock_ns=lambda: clock[0], palm_mapping=mapping)
    core.connected()
    actual = np.concatenate((arm.left_q, arm.right_q))
    actual[0] += 0.025

    def sample(*, yaw=0.0, turn=0.0, feedback=True):
        clock[0] += 15_000_000
        if feedback:
            assert core.update_feedback(dict(monotonic_ns=clock[0], joint_names=list(JOINT_NAMES[:14]),
                                             position_rad=actual.tolist(), velocity_rad_s=[0.0] * 14,
                                             retained_position_rad=(actual - 0.02).tolist(), scene_xml=None))
        assert core.accept_frame(standard_frame(clock[0] // 1_000_000, yaw=yaw, turn=turn), received_ns=clock[0])
        core.tick(clock[0])

    return core, clock, actual, sample


def calibrate_and_align(core, sample, *, yaw=0.0):
    sample(yaw=yaw)
    assert not core.command("align")
    assert core.command("calibrate")
    for _ in range(12):
        sample(yaw=yaw)
    assert core.palm_mapping.calibrated, core.status()
    assert core.command("align")
    for _ in range(10):
        sample(yaw=yaw)
    assert core.ready_mask == 7


def test_position_baseline_uses_actual_and_preview_never_moves(mapped_source):
    core, _, actual, sample = mapped_source
    calibrate_and_align(core, sample, yaw=np.pi / 2)
    np.testing.assert_allclose(core.position[:14], actual)
    held = core.position.copy()
    sample(yaw=np.pi / 2, turn=0.15)
    preview = core.status()["arm_preview"]
    expected = Rotation.from_euler("z", 0.15).as_matrix()
    for side in ("left", "right"):
        np.testing.assert_allclose(np.asarray(preview[side]["target_palm"])[:3, :3], expected, atol=1e-6)
        assert preview[side]["actual_palm"] is not None
    np.testing.assert_array_equal(core.position, held)
    assert core.running_mask == 0


def test_realign_keeps_anatomical_orientation_but_reconnect_requires_calibration(mapped_source):
    core, _, _, sample = mapped_source
    calibrate_and_align(core, sample)
    sample(turn=0.15)
    assert core.command("align")
    for _ in range(10):
        sample(turn=0.15)
    expected = Rotation.from_euler("z", 0.15).as_matrix()
    for side in ("left", "right"):
        target = np.asarray(core.status()["arm_preview"][side]["target_palm"])
        np.testing.assert_allclose(target[:3, :3], expected, atol=1e-6)
    core.disconnected()
    core.connected()
    sample(turn=0.15)
    assert not core.palm_mapping.calibrated
    assert not core.command("align")
    assert not core.command("start")


def test_stale_actual_feedback_revokes_arm_readiness_not_palm_calibration(mapped_source):
    core, clock, _, sample = mapped_source
    calibrate_and_align(core, sample)
    for _ in range(18):
        sample(feedback=False)
    assert not core.ready_mask & ARMS_READY
    assert core.palm_mapping.calibrated
    assert not core.command("align")
    assert all(info["actual_palm"] is None for info in core.status()["arm_preview"].values())
    stale = dict(monotonic_ns=clock[0] - core.FEEDBACK_FRESH_NS - 1, joint_names=list(JOINT_NAMES[:14]),
                 position_rad=[0.0] * 14, velocity_rad_s=[0.0] * 14, retained_position_rad=[0.0] * 14, scene_xml=None)
    assert not core.update_feedback(stale)


def test_invalid_feedback_cannot_rearm_using_previous_fresh_sample(mapped_source):
    core, clock, actual, sample = mapped_source
    calibrate_and_align(core, sample)
    clock[0] += 1_000_000
    invalid = dict(monotonic_ns=clock[0], joint_names=list(JOINT_NAMES[:14]),
                   position_rad=[float("nan")] * 14, velocity_rad_s=[0.0] * 14,
                   retained_position_rad=actual.tolist(), scene_xml=None)
    assert not core.update_feedback(invalid)
    assert not core.command("align")
    assert core.palm_mapping.calibrated
    assert all(info["actual_palm"] is None for info in core.status()["arm_preview"].values())
    sample()
    assert core.command("align")
