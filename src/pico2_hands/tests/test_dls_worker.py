import unittest
from unittest.mock import patch

import numpy as np
import yaml

from pico2_hands.dls_worker import DlsWorker
from pico2_hands.tests.helpers import dls_available
from pico2_hands.resources import controller_profile


@unittest.skipUnless(dls_available(), "build DLS worker")
class DlsWorkerTest(unittest.TestCase):
    def seeds(self):
        config = yaml.safe_load(controller_profile("qp_ik_pico_shared_root_dls.yaml").read_text())["controller"]
        return np.array([config["initial_left_q_rad"], config["initial_right_q_rad"]])

    def settled_tracking(self, worker):
        joints = self.seeds()
        fk = worker.reset(joints)
        target = np.array([fk[s]["achieved_pose"] for s in ("left", "right")])
        stamp = 1.
        self.assertTrue(worker.command(4, joints, stamp)["left"]["accepted"])
        # Finish the original approach at its target, including the limit ramp.
        for _ in range(180):
            stamp += 1. / 60. if worker.collection_session else .005
            result = worker.solve(joints, target, source_time=stamp, received_time=stamp, now=stamp)
            self.assertTrue(all(result[s]["accepted"] for s in ("left", "right")))
            joints = np.array([result[s]["joints"] for s in ("left", "right")])
        return joints, target, stamp

    def stop_at_hold(self, worker, joints, stamp):
        worker.command(6, joints, stamp)
        for _ in range(200):
            stamp += 1. / 60. if worker.collection_session else .005
            result = worker.command(7, joints, stamp)
            joints = np.array([result[s]["joints"] for s in ("left", "right")])
            if result["left"]["status"] == "HOLD":
                return joints, stamp
        self.fail("native worker did not finish bounded braking")

    def test_short_resume_preserves_tracking_response_within_ruckig_limits(self):
        displacement = {}
        config = yaml.safe_load(controller_profile("qp_ik_pico_shared_root_dls.yaml").read_text())
        limits = config["pico_ee_franka_dls"]["post_smoothing"]
        for operation in (4, 8):
            with self.subTest(operation=operation), DlsWorker(continuous_follow=True) as worker:
                joints, target, stamp = self.settled_tracking(worker)
                joints, stamp = self.stop_at_hold(worker, joints, stamp)
                initial = joints.copy()
                self.assertTrue(worker.command(operation, joints, stamp)["left"]["accepted"])
                # Starting/resuming must not jump the reference pose.
                target[:, 0] += .03
                positions = [joints.copy()] * 3
                for _ in range(20):
                    stamp += .005
                    result = worker.solve(joints, target, source_time=stamp, received_time=stamp, now=stamp)
                    self.assertTrue(all(result[s]["accepted"] for s in ("left", "right")))
                    joints = np.array([result[s]["joints"] for s in ("left", "right")])
                    positions.append(joints.copy())
                trajectory = np.asarray(positions)
                # Finite differences of exact position samples average native
                # derivatives; they must remain within the nominal envelope.
                velocity = np.diff(trajectory, axis=0) / .005
                acceleration = np.diff(velocity, axis=0) / .005
                jerk = np.diff(acceleration, axis=0) / .005
                self.assertTrue(np.isfinite(trajectory).all())
                self.assertTrue(np.all(np.abs(velocity) <= np.array(limits["max_velocity_rad_s"]) + 1e-4))
                self.assertTrue(np.all(np.abs(acceleration) <= np.array(limits["max_acceleration_rad_s2"]) + 1e-3))
                self.assertTrue(np.all(np.abs(jerk) <= np.array(limits["max_jerk_rad_s3"]) + 1e-2))
                displacement[operation] = np.max(np.abs(joints-initial))
        self.assertGreater(displacement[8], 2 * displacement[4])

    def test_resume_requires_recent_live_session_and_does_not_refresh_its_age(self):
        with DlsWorker(continuous_follow=True) as worker:
            joints = self.seeds()
            worker.reset(joints)
            self.assertFalse(worker.command(8, joints, 1.)["left"]["accepted"])
            joints, target, stamp = self.settled_tracking(worker)
            last_live = stamp
            joints, stamp = self.stop_at_hold(worker, joints, stamp)
            stale = worker._exchange(8, joints, np.zeros((2, 7)), 0., stamp-.05, stamp)
            self.assertFalse(stale["left"]["accepted"])
            self.assertTrue(worker.command(8, joints, stamp)["left"]["accepted"])
            joints, stamp = self.stop_at_hold(worker, joints, stamp)
            self.assertFalse(worker.command(8, joints, last_live+1.01)["left"]["accepted"])
            self.assertTrue(worker.command(4, joints, last_live+1.01)["left"]["accepted"])

    def test_resume_cannot_bypass_reset_or_manual_mode(self):
        for continuous in (False, True):
            with self.subTest(continuous=continuous), DlsWorker(continuous_follow=continuous) as worker:
                joints, _, stamp = self.settled_tracking(worker)
                joints, stamp = self.stop_at_hold(worker, joints, stamp)
                if continuous:
                    worker.reset(joints)  # A new numerical epoch revokes resume.
                result = worker.command(8, joints, stamp)
                self.assertFalse(result["left"]["accepted"])
                self.assertEqual(result["left"]["status"], "HOLD")
                np.testing.assert_array_equal(result["left"]["joints"], joints[0])

    def test_collection_resume_interrupts_braking_without_derivative_reset(self):
        config = yaml.safe_load(controller_profile("qp_ik_pico_shared_root_dls.yaml").read_text())
        limits = config["pico_ee_franka_dls"]["post_smoothing"]
        with DlsWorker(collection_session=True) as worker:
            joints, target, stamp = self.settled_tracking(worker)
            target[:, 0] += .04
            positions = []
            for _ in range(5):  # Interrupt after ~80 ms, before reaching the new target.
                stamp += 1. / 60.
                result = worker.solve(joints, target, source_time=stamp, received_time=stamp, now=stamp)
                self.assertTrue(result["left"]["accepted"])
                joints = np.array([result[s]["joints"] for s in ("left", "right")])
                positions.append(joints.copy())
            self.assertGreater(np.max(np.abs(positions[-1] - positions[-2])), 1e-7)
            worker.command(6, joints, stamp)
            stamp += 1. / 60.
            result = worker.command(7, joints, stamp)
            self.assertEqual(result["left"]["status"], "BRAKING")
            joints = np.array([result[s]["joints"] for s in ("left", "right")])
            positions.append(joints.copy())
            resumed = worker.command(8, joints, stamp)
            self.assertTrue(resumed["left"]["accepted"])
            np.testing.assert_array_equal(resumed["left"]["joints"], joints[0])
            for _ in range(20):
                stamp += 1. / 60.
                result = worker.solve(joints, target, source_time=stamp, received_time=stamp, now=stamp)
                self.assertTrue(result["left"]["accepted"])
                joints = np.array([result[s]["joints"] for s in ("left", "right")])
                positions.append(joints.copy())
            velocity = np.diff(np.asarray(positions), axis=0) * 60.
            acceleration = np.diff(velocity, axis=0) * 60.
            jerk = np.diff(acceleration, axis=0) * 60.
            self.assertTrue(np.all(np.abs(velocity) <= np.asarray(limits["max_velocity_rad_s"]) + 1e-4))
            self.assertTrue(np.all(np.abs(acceleration) <= np.asarray(limits["max_acceleration_rad_s2"]) + 1e-3))
            self.assertTrue(np.all(np.abs(jerk) <= np.asarray(limits["max_jerk_rad_s3"]) + 1e-2))

    def test_frozen_authority_replaces_moving_reference_and_homes_without_tracking(self):
        with DlsWorker(collection_session=True) as worker:
            joints, target, stamp = self.settled_tracking(worker)
            target[:, 0] += .03
            for _ in range(25):
                stamp += 1. / 60.
                result = worker.solve(joints, target, source_time=stamp, received_time=stamp, now=stamp)
                joints = np.array([result[s]["joints"] for s in ("left", "right")])
            retained = self.seeds()
            retained[:, 0] += .02
            bound = worker.bind_frozen(retained, stamp)
            for i, side in enumerate(("left", "right")):
                np.testing.assert_array_equal(bound[side]["joints"], retained[i])
            self.assertFalse(worker.command(8, retained, stamp)["left"]["accepted"])
            destination = self.seeds()
            destination[:, 0] -= .01
            result = worker.home_frozen(retained, destination, stamp)
            joints = retained
            for _ in range(1200):
                stamp += 1. / 60.
                result = worker.command(7, joints, stamp)
                joints = np.array([result[s]["joints"] for s in ("left", "right")])
                if result["left"]["status"] == "HOME_REACHED":
                    break
            self.assertEqual(result["left"]["status"], "HOME_REACHED")
            np.testing.assert_allclose(joints, destination, atol=1e-6, rtol=0.)

    def test_ordinary_reset_still_rejects_running_collection_reference(self):
        with DlsWorker(collection_session=True) as worker:
            joints, _, stamp = self.settled_tracking(worker)
            with self.assertRaises(RuntimeError):
                worker.reset(joints)
            self.assertTrue(worker.failed)

    def test_unexpected_worker_exit_is_not_hidden_by_close(self):
        worker = DlsWorker()
        worker.process.terminate()
        worker.process.wait(timeout=2)
        with self.assertRaises(RuntimeError):
            worker.close()
        self.assertTrue(worker.failed)

    def test_explicit_reset_start_motion_and_home(self):
        seed = self.seeds()
        with DlsWorker() as worker:
            fk = worker.forward(seed)
            target = np.array([fk[s]["achieved_pose"] for s in ("left", "right")])
            with self.assertRaises(ValueError):
                worker.solve(seed, target, source_time=1., received_time=1., now=1.)
            worker.reset(seed)
            worker.reset(seed)  # Repeated at-rest epoch barriers remain valid.
            self.assertTrue(worker.command(4, seed, 1.)["left"]["accepted"])
            target[:, 0] += .01
            joints = seed.copy()
            for i in range(100):
                stamp = 1.005 + i*.005
                result = worker.solve(joints, target, source_time=stamp, received_time=stamp, now=stamp)
                self.assertTrue(all(result[s]["accepted"] for s in ("left", "right")))
                joints = np.array([result[s]["joints"] for s in ("left", "right")])
            self.assertGreater(np.max(np.abs(joints-seed)), 1e-4)
            worker.command(5, joints, stamp)
            for _ in range(5000):
                stamp += .005
                result = worker.command(7, joints, stamp)
                joints = np.array([result[s]["joints"] for s in ("left", "right")])
                if result["left"]["status"] == "HOME_REACHED":
                    break
            self.assertEqual(result["left"]["status"], "HOME_REACHED")
            np.testing.assert_allclose(joints, seed, atol=1e-6, rtol=0)
        self.assertEqual(worker.process.returncode, 0)

    def test_invalid_geometry_does_not_poison_next_request(self):
        with DlsWorker() as worker:
            seed = self.seeds()
            worker.reset(seed)
            sequence = worker.sequence
            with self.assertRaises(ValueError):
                worker.solve(seed, np.zeros((2, 7)), source_time=1., received_time=1., now=1.)
            self.assertEqual(worker.sequence, sequence)
            self.assertTrue(worker.command(4, seed, 1.)["left"]["accepted"])

    def test_native_clock_rollback_fails_closed(self):
        for source, received, now in ((1., 3., 3.), (2., 1., 1.)):
            with self.subTest(source=source, now=now), DlsWorker() as worker:
                seed = self.seeds()
                fk = worker.reset(seed)
                targets = [fk[s]["achieved_pose"] for s in ("left", "right")]
                worker.command(4, seed, 1.)
                result = worker.solve(seed, targets, source_time=2., received_time=2., now=2.)
                joints = np.array([result[s]["joints"] for s in ("left", "right")])
                with self.assertRaises(RuntimeError):
                    worker.solve(joints, targets, source_time=source, received_time=received, now=now)
                self.assertTrue(worker.failed)
                with self.assertRaises(RuntimeError):
                    worker.reset(joints)

    def test_native_repeated_sequence_fails_closed(self):
        with DlsWorker() as worker:
            seed = self.seeds()
            worker.reset(seed)
            worker.sequence -= 1
            with self.assertRaises(RuntimeError):
                worker.forward(seed)
            self.assertTrue(worker.failed)

    def test_response_association_mismatch_fails_closed(self):
        with DlsWorker() as worker:
            original_read = worker._read

            def wrong_sequence(count, deadline):
                response = original_read(count, deadline)
                response[8] ^= 1
                return response

            with patch.object(worker, "_read", side_effect=wrong_sequence):
                with self.assertRaises(RuntimeError):
                    worker.reset(self.seeds())
            self.assertTrue(worker.failed)

    def test_timeout_latches_client(self):
        with DlsWorker() as worker:
            with patch.object(worker, "_read", side_effect=TimeoutError("injected")):
                with self.assertRaises(TimeoutError):
                    worker.reset(self.seeds())
            self.assertTrue(worker.failed)
            with self.assertRaises(RuntimeError):
                worker.forward(self.seeds())
