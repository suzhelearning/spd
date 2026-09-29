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
        gate.advance(retained, start + 120_000_000, .02)
        self.assertEqual(gate.mode, "waiting")
        # A persistent loss restarts the blend without changing the held grip.
        fresh = start + 150_000_000
        gate.observe(np.full(20, -1.), fresh, retained)
        gate.advance(retained, fresh, .02)
        np.testing.assert_array_equal(retained, held)
        gate.advance(retained, fresh + 20_000_000, .02)
        self.assertTrue(np.all(retained < held))
        self.assertLessEqual(float(np.max(np.abs(retained - held))), .040000000001)

    def test_short_invalid_and_stale_gaps_hold_then_resume_live_without_reentry(self):
        retained = np.zeros(20)
        gate = _FingerGate()
        start = 1_000_000_000
        for tick in range(20):
            stamp = start + tick * 20_000_000
            gate.observe(np.full(20, .2), stamp, retained)
            gate.advance(retained, stamp, .02)
        np.testing.assert_allclose(retained, .2)
        gate.invalidate(retained, stamp + 10_000_000)
        gate.advance(retained, stamp + 10_000_000, .02)
        np.testing.assert_allclose(retained, .2)
        self.assertFalse(gate.fresh(stamp + 10_000_000))
        self.assertEqual(gate.mode, "live")
        # One fresh frame resumes the live rate-limited path, not a new blend.
        stamp += 40_000_000
        gate.observe(np.full(20, .8), stamp, retained)
        gate.advance(retained, stamp, .02)
        np.testing.assert_allclose(retained, .24)
        gate.advance(retained, stamp + 46_000_000, .02)
        np.testing.assert_allclose(retained, .24)
        self.assertEqual(gate.mode, "live")
        gate.observe(np.full(20, .8), stamp + 80_000_000, retained)
        gate.advance(retained, stamp + 80_000_000, .02)
        np.testing.assert_allclose(retained, .28)

    def test_short_gap_pauses_blend_progress_without_resetting_or_skipping_it(self):
        retained = np.zeros(20)
        uninterrupted = retained.copy()
        gate, reference = _FingerGate(), _FingerGate()
        start = 1_000_000_000
        target = np.full(20, .1)
        for tick in (0, 20):
            stamp = start + tick * 1_000_000
            for current, output in ((gate, retained), (reference, uninterrupted)):
                current.observe(target, stamp, output)
                current.advance(output, stamp, .02)
        held = retained.copy()
        gate.invalidate(retained, start + 30_000_000)
        gate.advance(retained, start + 70_000_000, .02)
        np.testing.assert_array_equal(retained, held)
        gate.observe(target, start + 80_000_000, retained)
        gate.advance(retained, start + 80_000_000, .02)
        # 50 ms of invalid input is excluded from the 200 ms blend clock.
        reference.observe(target, start + 30_000_000, uninterrupted)
        reference.advance(uninterrupted, start + 30_000_000, .02)
        np.testing.assert_allclose(retained, uninterrupted, atol=1e-12)
        self.assertTrue(np.all(retained > held))

    def test_repeated_invalid_frames_do_not_extend_persistent_loss_budget(self):
        retained = np.full(20, .2)
        gate = _FingerGate()
        gate.reset(retained)
        start = 1_000_000_000
        for tick in range(12):
            stamp = start + tick * 20_000_000
            gate.observe(np.full(20, .2), stamp, retained)
            gate.advance(retained, stamp, .02)
        for offset in (10, 40, 80, 119):
            gate.invalidate(retained, stamp + offset * 1_000_000)
            gate.advance(retained, stamp + offset * 1_000_000, .02)
            self.assertEqual(gate.mode, "live")
        gate.invalidate(retained, stamp + 120_000_000)
        self.assertEqual(gate.mode, "waiting")
        np.testing.assert_array_equal(retained, np.full(20, .2))
        gate.observe(np.full(20, .8), stamp + 130_000_000, retained)
        gate.advance(retained, stamp + 130_000_000, .02)
        np.testing.assert_array_equal(retained, np.full(20, .2))
        gate.advance(retained, stamp + 150_000_000, .02)
        self.assertTrue(np.all(retained > .2))
        self.assertTrue(np.all(retained < .24))
