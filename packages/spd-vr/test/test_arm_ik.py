from dataclasses import replace
import json
import pytest

import numpy as np

import spd_vr.arm_ik as arm_ik
from spd_vr.arm_ik import DualArmController, build_synthetic_fixture
from spd_vr.alignment import SideAlignment
from spd_vr.wire import (
    ARM_TARGETS_KEY,
    STATUS_IK_KEY,
    ArmTargetHoldReason,
    ControlCommand,
    ControlFrame,
    TrackingFrame,
)


def frame(sequence=1, timestamp=1_000_000_000):
    hand = np.zeros((26, 7), dtype=np.float32)
    hand[:, 6] = 1.0
    return TrackingFrame(
        sequence=sequence,
        tracking_epoch=1,
        source_timestamp_ns=timestamp,
        bridge_monotonic_ns=timestamp,
        left_active=True,
        right_active=True,
        head_valid=True,
        left_scale=1.0,
        right_scale=1.0,
        head_pose=np.array([0, 0, 1.6, 0, 0, 0, 1], dtype=np.float32),
        left_hand=hand.copy(), right_hand=hand.copy(),
    )


def test_dual_controller_keeps_side_hold_isolated():
    controller, pose_left, pose_right = build_synthetic_fixture()
    controller.accept_tracking(frame())
    for i in range(10):
        controller.accept_tracking(frame(i + 1, 1_000_000_000 + i + 1))
        result = controller.tick(1_000_000_000 + i + 1)
    assert result.left_hold_reason.name == "NONE"
    assert result.right_hold_reason.name == "NONE"

    controller.left_alignment.reset()
    controller.accept_tracking(frame(20, 1_000_000_100))
    held = controller.tick(1_000_000_100)
    assert held.valid_mask & 1 == 0
    assert held.valid_mask & 2 == 2
    np.testing.assert_allclose(held.right_q, controller.right_q)


def test_controller_uses_absolute_deadlines_without_catchup():
    controller, _, _ = build_synthetic_fixture()
    controller.accept_tracking(frame())
    times = iter([0, 5_000_000, 20_000_000, 25_000_000])
    ticks = controller.run(2, clock=lambda: next(times), sleep=lambda _: None)
    assert len(ticks) == 2
    assert controller.tick_count == 2


@pytest.mark.parametrize(
    "elapsed_ns, integrated_s",
    [
        (5_000_000, 0.005),
        (20_000_000, 0.020),
        (100_000_000, 0.050),
        (0, 0.0),
        (-5_000_000, 0.0),
    ],
)
def test_arm_motion_tracks_elapsed_time_without_unbounded_catchup(elapsed_ns, integrated_s):
    controller, left_pose, right_pose = build_synthetic_fixture()
    controller.left_alignment = SideAlignment(neutral_robot=left_pose, stable_frames=1)
    controller.right_alignment = SideAlignment(neutral_robot=right_pose, stable_frames=1)
    controller.left_solver.velocity_limits[:] = 0.01
    start = 1_000_000_000
    controller.accept_tracking(arm_ik._synthetic_tracking(left_pose, right_pose, 1, start))
    controller.tick(start)
    previous_q = controller.left_q.copy()
    left_pose[1, 3] += 0.02
    timestamp = start + max(elapsed_ns, 5_000_000)
    controller.accept_tracking(arm_ik._synthetic_tracking(left_pose, right_pose, 2, timestamp))

    result = controller.tick(start + elapsed_ns)

    # The reachable translational target saturates one hinge at 0.01 rad/s.
    assert np.linalg.norm(controller.left_q - previous_q) == pytest.approx(
        0.01 * integrated_s, abs=1.0e-8
    )
    if integrated_s == 0.0:
        assert result.left_hold_reason is ArmTargetHoldReason.SOLVER_FAILURE
    else:
        assert result.left_hold_reason is ArmTargetHoldReason.NONE


def test_bursty_receipt_uses_device_motion_time_but_local_freshness():
    controller, left_pose, right_pose = build_synthetic_fixture()
    controller.left_alignment = SideAlignment(neutral_robot=left_pose, stable_frames=1)
    controller.right_alignment = SideAlignment(neutral_robot=right_pose, stable_frames=1)
    source = 1_000_000_000
    receipt = 10_000_000_000
    initial = replace(
        arm_ik._synthetic_tracking(left_pose, right_pose, 1, source),
        bridge_monotonic_ns=receipt,
    )
    controller.accept_tracking(initial)
    controller.tick(receipt)
    left_pose[1, 3] += 0.04
    next_receipt = receipt + 1_000
    moved = replace(
        arm_ik._synthetic_tracking(left_pose, right_pose, 2, source + 20_000_000),
        bridge_monotonic_ns=next_receipt,
    )
    controller.accept_tracking(moved)

    active = controller.tick(next_receipt)
    assert active.left_hold_reason is ArmTargetHoldReason.NONE
    assert controller.left_alignment.aligned

    stale = controller.tick(next_receipt + 50_000_001)
    assert stale.left_hold_reason is ArmTargetHoldReason.INPUT_STALE
    assert not controller.left_alignment.aligned


def test_control_gate_processes_ordered_commands_once():
    controller, _, _ = build_synthetic_fixture()
    pause = ControlFrame(2, 2_000_000_000, ControlCommand.PAUSE)
    assert controller.accept_control(pause)
    assert not controller.accept_control(pause)
    assert not controller.accept_control(ControlFrame(1, 2_000_000_001, ControlCommand.START))
    held = controller.tick(2_000_000_000)
    assert held.control_timestamp_ns == pause.monotonic_timestamp_ns
    assert held.left_hold_reason is ArmTargetHoldReason.PAUSED
def test_reset_does_not_drop_queued_shutdown():
    controller, _, _ = build_synthetic_fixture()
    controller.control_mailbox.put(ControlFrame(1, 3_000_000_000, ControlCommand.RESET))
    controller.control_mailbox.put(ControlFrame(2, 3_000_000_001, ControlCommand.SHUTDOWN))
    controller.tick(3_000_000_000)
    assert not controller.running


def test_production_ik_uses_connect_only_peer_config(monkeypatch):
    controller, _, _ = build_synthetic_fixture()
    seen: dict[str, object] = {}
    class Publisher:
        def __init__(self):
            self.payloads = []

        def put(self, payload):
            self.payloads.append(payload)

    class FakeNode:
        def __init__(self, config):
            seen["config"] = config
            self.publishers = {}
            seen["publishers"] = self.publishers

        def declare_latest_subscriber(self, *args):
            return object()

        def declare_publisher(self, key, *args, **kwargs):
            publisher = Publisher()
            self.publishers[key] = publisher
            return publisher

        def close(self):
            seen["closed"] = True

    def fake_config(*, listen, endpoint):
        seen["listen"] = listen
        seen["endpoint"] = endpoint
        return object()

    monkeypatch.setattr(arm_ik, "_verified_model", lambda *args: (object(), object()))
    monkeypatch.setattr(arm_ik, "_production_controller", lambda *args: controller)
    monkeypatch.setattr(arm_ik, "peer_config", fake_config)
    monkeypatch.setattr(arm_ik, "ZenohNode", FakeNode)

    def fake_run():
        assert controller.accept_control(ControlFrame(7, 7, ControlCommand.PAUSE))
        assert controller.accept_control(ControlFrame(8, 8, ControlCommand.SHUTDOWN))
        raise KeyboardInterrupt
    monkeypatch.setattr(controller, "run", fake_run)

    result = arm_ik.main(["--model", "arm.xml", "--manifest", "manifest.yaml", "--urdf", "robot.urdf"])
    assert result == 0
    assert seen["listen"] is False
    assert set(seen["publishers"]) == {ARM_TARGETS_KEY, STATUS_IK_KEY}
    status_payloads = seen["publishers"][STATUS_IK_KEY].payloads
    assert status_payloads
    statuses = [json.loads(payload) for payload in status_payloads]
    assert any(item["status"] == "ready" and item["sequence"] is None for item in statuses)
    assert any(item["status"] == "paused" and item["sequence"] == 7 for item in statuses)
    assert sum(item["status"] == "shutdown" for item in statuses) == 1
    assert statuses[-1]["status"] == "shutdown"
    assert statuses[-1]["sequence"] == 8
    assert statuses[-1]["running"] is False
    assert seen["closed"] is True


def test_shutdown_status_publish_failure_is_idempotent():
    controller, _, _ = build_synthetic_fixture()

    class FailingPublisher:
        def __init__(self):
            self.calls = 0

        def put(self, _payload):
            self.calls += 1
            raise RuntimeError("status transport failed")

    publisher = FailingPublisher()
    controller._status_publisher = publisher
    with pytest.raises(RuntimeError, match="status transport failed"):
        controller.shutdown()
    controller.shutdown()
    assert publisher.calls == 1


def test_main_closes_node_when_final_shutdown_fails(monkeypatch):
    controller, _, _ = build_synthetic_fixture()
    seen: dict[str, bool] = {"closed": False}

    class Publisher:
        def put(self, _payload):
            return None

    class FakeNode:
        def __init__(self, _config):
            pass

        def declare_latest_subscriber(self, *args):
            return object()

        def declare_publisher(self, _key, *args, **kwargs):
            return Publisher()

        def close(self):
            seen["closed"] = True

    monkeypatch.setattr(arm_ik, "_verified_model", lambda *args: (object(), object()))
    monkeypatch.setattr(arm_ik, "_production_controller", lambda *args: controller)
    monkeypatch.setattr(arm_ik, "peer_config", lambda **kwargs: object())
    monkeypatch.setattr(arm_ik, "ZenohNode", FakeNode)
    monkeypatch.setattr(controller, "run", lambda: (_ for _ in ()).throw(KeyboardInterrupt))
    monkeypatch.setattr(controller, "shutdown", lambda: (_ for _ in ()).throw(RuntimeError("cleanup failed")))

    assert arm_ik.main(["--model", "arm.xml", "--manifest", "manifest.yaml", "--urdf", "robot.urdf"]) == 0
    assert seen["closed"] is True


def test_unreachable_wrist_keeps_alignment_and_valid_bounded_hold():
    controller, left_pose, right_pose = build_synthetic_fixture()
    unreachable = left_pose.copy()
    unreachable[0, 3] += 10.0
    controller.left_alignment = SideAlignment(neutral_robot=unreachable, stable_frames=1)
    controller.right_alignment = SideAlignment(neutral_robot=right_pose, stable_frames=1)
    before = controller.left_q.copy()
    for i in range(3):
        timestamp = 1_000_000_000 + i * 5_000_000
        controller.accept_tracking(arm_ik._synthetic_tracking(left_pose, right_pose, i + 1, timestamp))
        result = controller.tick(timestamp)
        assert controller.left_alignment.aligned
        assert result.valid_mask & 1
        assert result.left_hold_reason == ArmTargetHoldReason.NONE
        np.testing.assert_array_equal(result.left_q, before)
    diagnostic = controller.diagnostics()["left"]
    assert diagnostic["state"] == "blocked"
    assert diagnostic["position_error_m"] > 9
    assert diagnostic["detail"]


def test_actual_feedback_does_not_jump_command_trajectory_and_hold_preserves_alignment():
    controller, left_pose, right_pose = build_synthetic_fixture()
    controller.left_alignment = SideAlignment(neutral_robot=left_pose, stable_frames=1)
    controller.right_alignment = SideAlignment(neutral_robot=right_pose, stable_frames=1)
    controller.accept_tracking(arm_ik._synthetic_tracking(left_pose, right_pose, 1, 1_000_000_000))
    controller.tick(1_000_000_000)
    original = np.concatenate((controller.left_q, controller.right_q))
    controller.update_actual_state(original + 0.01, np.zeros(14))
    np.testing.assert_array_equal(np.concatenate((controller.left_q, controller.right_q)), original)
    controller.reset_motion()
    assert controller.left_alignment.aligned
    assert controller.right_alignment.aligned
    with pytest.raises(ValueError):
        controller.update_actual_state(np.full(14, np.nan), np.zeros(14))
