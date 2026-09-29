"""Relative mapping and independent finger authorization boundaries."""
from dataclasses import replace
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from pico2_hands.collection_session import _FingerGate, _RelativeAnchor
from pico2_hands.tests.helpers import frame


class RelativeAnchorTests(unittest.TestCase):
    def test_waist_bind_preserves_full_tcp_pose_and_ignores_global_translation(self):
        raw = frame(yaw=.6, move=[-.17, 0., -.25])
        hands = dict(raw.hands)
        for i, side in enumerate(("left", "right")):
            pose = hands[side].wrist_pose.copy()
            pose[3:] = Rotation.from_euler("xyz", [.4, -.3, .7 + i]).as_quat()
            hands[side] = replace(hands[side], wrist_pose=pose)
        raw = replace(raw, hands=hands)
        reference = {
            "left": {"achieved_pose": np.r_[[.3, .2, .8], Rotation.from_euler("xyz", [.2, .5, -.4]).as_quat()]},
            "right": {"achieved_pose": np.r_[[.4, -.2, .9], Rotation.from_euler("xyz", [-.2, -.5, .4]).as_quat()]},
        }
        anchor = _RelativeAnchor(1.62)
        geometry = anchor.extract(raw)
        anchor.bind(geometry, reference)
        targets = anchor.targets(geometry)
        for i, side in enumerate(("left", "right")):
            np.testing.assert_allclose(targets[i, :3], reference[side]["achieved_pose"][:3], atol=1e-14)
            delta = Rotation.from_quat(reference[side]["achieved_pose"][3:]).inv() * Rotation.from_quat(targets[i, 3:])
            self.assertLess(delta.magnitude(), 1e-12)
        translation = np.array([.5, -.3, .1])
        head, head_rotation, wrists, rotations = geometry
        translated = (head + translation, head_rotation, wrists + translation, rotations)
        np.testing.assert_allclose(anchor.targets(translated), targets, atol=1e-14)
        # A head turn after binding does not turn the calibrated level axes.
        turned = (head, Rotation.from_euler("z", 1.2), wrists, rotations)
        np.testing.assert_allclose(anchor.targets(turned), targets, atol=1e-14)

    def test_finger_occlusion_does_not_revoke_valid_wrist_tracking(self):
        raw = frame()
        left = raw.hands["left"]
        joints = tuple(replace(joint, valid=False) for joint in left.joints)
        occluded = replace(raw, hands={**raw.hands, "left": replace(left, joints=joints)})
        geometry = _RelativeAnchor.extract(occluded)
        np.testing.assert_array_equal(geometry[2][0], left.wrist_pose[:3])
        missing_wrist = replace(occluded, hands={**occluded.hands, "left": replace(left, wrist_valid=False)})
        with self.assertRaises(ValueError):
            _RelativeAnchor.extract(missing_wrist)


class FingerReentryTests(unittest.TestCase):
    def test_distant_targets_follow_independently_without_matching(self):
        retained = {"left": np.full(20, .5), "right": np.full(20, .5)}
        targets = {"left": np.full(20, -1.), "right": np.full(20, 1.5)}
        gates = {side: _FingerGate() for side in retained}
        for side in gates:
            gates[side].reset(retained[side])
        for tick in range(100):
            stamp = 1_000_000_000 + tick * 20_000_000
            for side in gates:
                before = retained[side].copy()
                gates[side].observe(targets[side], stamp, retained[side])
                gates[side].advance(retained[side], stamp, .02)
                self.assertLessEqual(float(np.max(np.abs(retained[side] - before))), .040000000001)
                if tick == 0:
                    np.testing.assert_array_equal(retained[side], before)
        for side in gates:
            np.testing.assert_allclose(retained[side], targets[side], atol=1e-12)

    def test_loss_holds_and_first_fresh_target_reenters_without_jump(self):
        retained = np.full(20, .5)
        gate = _FingerGate()
        gate.reset(retained)
        start = 1_000_000_000
        gate.observe(np.full(20, 1.5), start, retained)
        gate.advance(retained, start, .02)
        gate.advance(retained, start + 20_000_000, .02)
        self.assertTrue(np.all(retained > .5))
        held = retained.copy()
        gate.advance(retained, start + 46_000_000, .02)
        np.testing.assert_array_equal(retained, held)
        # A duplicate sample must not authorize stale motion.
        gate.observe(np.full(20, 1.5), start, retained)
        gate.advance(retained, start + 60_000_000, .02)
        np.testing.assert_array_equal(retained, held)
        # Recovery need not resemble the old grip or wait for three samples.
        fresh = start + 100_000_000
        gate.observe(np.full(20, -1.), fresh, retained)
        gate.advance(retained, fresh, .02)
        np.testing.assert_array_equal(retained, held)
        gate.advance(retained, fresh + 20_000_000, .02)
        self.assertTrue(np.all(retained < held))
        self.assertLessEqual(float(np.max(np.abs(retained - held))), .040000000001)
