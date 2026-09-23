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
    def test_unmatched_grip_is_held_while_other_side_blends_and_loss_rearms_gate(self):
        retained = {"left": np.full(20, .5), "right": np.full(20, .5)}
        gates = {side: _FingerGate() for side in retained}
        for side in gates:
            gates[side].reset(retained[side])
        start = 1_000_000_000
        for tick in range(20):
            stamp = start + tick * 20_000_000
            gates["left"].observe(np.full(20, .1), stamp, retained["left"])
            gates["right"].observe(np.full(20, .6), stamp, retained["right"])
            for side in gates:
                gates[side].advance(retained[side], stamp, .02)
        np.testing.assert_array_equal(retained["left"], np.full(20, .5))
        np.testing.assert_allclose(retained["right"], np.full(20, .6), atol=1e-12)
        self.assertEqual(gates["left"].mode, "waiting")
        self.assertEqual(gates["right"].mode, "live")
        gates["right"].advance(retained["right"], stamp + 46_000_000, .02)
        held = retained["right"].copy()
        for tick in range(4):
            stamp += 60_000_000
            gates["right"].observe(np.full(20, .9), stamp, retained["right"])
            gates["right"].advance(retained["right"], stamp, .02)
        np.testing.assert_array_equal(retained["right"], held)
        self.assertEqual(gates["right"].mode, "waiting")

    def test_match_requires_distinct_stable_samples_and_starts_without_jump(self):
        retained = np.full(20, .5)
        gate = _FingerGate()
        gate.reset(retained)
        target = np.full(20, .6)
        start = 1_000_000_000
        gate.observe(target, start, retained)
        for _ in range(10):
            gate.observe(target, start, retained)
            gate.advance(retained, start + 10_000_000, .005)
        np.testing.assert_array_equal(retained, np.full(20, .5))
        gate.observe(target, start + 20_000_000, retained)
        gate.observe(target, start + 40_000_000, retained)
        gate.advance(retained, start + 40_000_000, .005)
        np.testing.assert_array_equal(retained, np.full(20, .5))
        gate.advance(retained, start + 60_000_000, .005)
        self.assertTrue(np.all(retained > .5))
        self.assertTrue(np.all(retained < .51))
