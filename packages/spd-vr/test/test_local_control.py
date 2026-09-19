from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import socket
import tempfile
import threading
import time
import uuid

import numpy as np
import pytest

from spd_vr.local_control import LocalControlClient, LocalControlServer
from spd_vr.ros_joint_command import ARM_NAMES
from test_ros_joint_command import _message, command_plant, receiver


@pytest.fixture
def control(receiver):
    executor, clock = receiver
    # Keep AF_UNIX paths short independently of pytest's test-directory names.
    with tempfile.TemporaryDirectory(prefix="spd-ctl-") as directory:
        path = Path(directory) / "control"
        server = LocalControlServer(path, executor)
        with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as peer:
            peer.bind(str(Path(directory) / "reply"))
            peer.settimeout(0.5)

            def request(op, session="session-a", *, request_id=None, deadline_ns=None):
                packet = {
                    "request_id": request_id or uuid.uuid4().hex,
                    "op": op,
                    "session_id": session,
                    "deadline_ns": clock.mono + 500_000_000 if deadline_ns is None else deadline_ns,
                }
                peer.sendto(json.dumps(packet).encode(), str(path))
                server.poll()
                return json.loads(peer.recv(16384))

            yield server, path, request
        server.close()


def _roundtrip(server, path, op="status", session="session-a"):
    with ThreadPoolExecutor(max_workers=1) as worker:
        result = worker.submit(LocalControlClient(path).request, op, session)
        deadline = time.perf_counter() + 2
        while not result.done() and time.perf_counter() < deadline:
            server.poll()
            threading.Event().wait(.001)
        return result.result(timeout=1)


def test_expired_enable_cannot_authorize_or_apply(control, receiver, command_plant):
    _, _, request = control
    executor, clock = receiver
    assert executor.mailbox.receive(_message(1, ready_mask=7, positions=np.full(54, .1)))
    result = request("enable", deadline_ns=clock.mono)
    assert not result["ok"]
    assert "expired" in result["error"]
    assert not executor.mailbox.enabled
    assert executor.apply_pending() is None
    np.testing.assert_array_equal(command_plant.joint_command_targets(), np.zeros(54))


def test_deadline_crossing_during_gate_revokes_before_targets_apply(control, receiver, monkeypatch):
    _, _, request = control
    executor, clock = receiver
    assert executor.mailbox.receive(_message(1, ready_mask=7))
    authorize = executor.authorize

    def delayed_authorize(enabled):
        result = authorize(enabled)
        if enabled:
            clock.mono += 500_000_000
        return result

    monkeypatch.setattr(executor, "authorize", delayed_authorize)
    result = request("enable")
    assert not result["ok"] and not result["enabled"]
    assert result["authorized_session"] is None
    assert executor.apply_pending() is None


def test_wrong_session_cannot_enable_or_hold_another_session(control, receiver):
    _, _, request = control
    executor, _ = receiver
    assert executor.mailbox.receive(_message(1, ready_mask=7))
    assert not request("enable", "old-session")["ok"]
    assert not executor.mailbox.enabled
    assert request("enable")["ok"]
    result = request("hold", "old-session")
    assert not result["ok"]
    assert result["enabled"]
    assert result["authorized_session"] == "session-a"
    assert executor.mailbox.receive(_message(1, session="session-b", ready_mask=7))
    assert not request("enable", "session-a")["ok"]
    assert not executor.mailbox.enabled


def test_local_enable_requires_all_groups_without_changing_executor_semantics(control, receiver):
    _, _, request = control
    executor, _ = receiver
    assert executor.mailbox.receive(_message(1, ready_mask=1))
    result = request("enable")
    assert not result["ok"]
    assert result["ready_mask"] == 1
    assert not result["enabled"]
    assert executor.authorize(True)  # Other existing operator paths still allow partial groups.


def test_enable_keeps_delta_gate_and_success_does_not_report_old_rejection(control, receiver):
    _, _, request = control
    executor, _ = receiver
    assert executor.mailbox.receive(_message(1, ready_mask=7, positions=np.full(54, .16)))
    result = request("enable")
    assert not result["ok"]
    assert "delta" in result["error"]
    assert not result["enabled"]
    assert executor.mailbox.receive(_message(2, ready_mask=7, positions=np.full(54, .1)))
    result = request("enable")
    assert result["ok"] and result["enabled"]
    assert result["error"] == ""
    assert result["authorized_session"] == result["candidate_session"] == "session-a"
    assert result["hold_mask"] == 0
    assert request("hold")["ok"]
    status = request("status")
    assert status["ok"] and not status["enabled"]
    assert status["error"] == ""
    assert "delta" not in status["reason"]


def test_enable_revalidates_candidate_age(control, receiver):
    _, _, request = control
    executor, clock = receiver
    assert executor.mailbox.receive(_message(1, ready_mask=7))
    clock.utc += 100_000_001
    result = request("enable")
    assert not result["ok"]
    assert "older" in result["error"]
    assert not result["enabled"]


def test_hold_retains_physics_and_targets_and_replay_cannot_resume(control, receiver, command_plant):
    _, _, request = control
    executor, _ = receiver
    assert executor.mailbox.receive(_message(1, ready_mask=7, positions=np.full(54, .1)))
    enable_id = uuid.uuid4().hex
    assert request("enable", request_id=enable_id)["ok"]
    executor.apply_pending()
    command_plant.physics_tick()
    positions = command_plant.data.qpos.copy()
    targets = command_plant.joint_command_targets()
    assert executor.mailbox.receive(_message(2, ready_mask=7, positions=np.full(54, .12)))
    result = request("hold")
    assert result["ok"] and not result["enabled"]
    assert result["authorized_session"] is None
    assert result["hold_mask"] == 7
    assert executor.apply_pending() is None
    np.testing.assert_array_equal(command_plant.data.qpos, positions)
    np.testing.assert_array_equal(command_plant.joint_command_targets(), targets)
    replay = request("enable", request_id=enable_id)
    assert not replay["ok"] and not replay["enabled"]
    assert executor.apply_pending() is None
    np.testing.assert_array_equal(command_plant.joint_command_targets(), targets)


def test_client_roundtrip_and_owned_socket_cleanup(control, receiver):
    server, path, _ = control
    executor, _ = receiver
    assert executor.mailbox.receive(_message(1, ready_mask=7))
    client = LocalControlClient(path)
    # Only the worker may block; the owning thread keeps servicing the endpoint.
    with ThreadPoolExecutor(max_workers=1) as worker:
        for op in ("enable", "status", "hold"):
            result = worker.submit(client.request, op, "session-a")
            deadline = time.perf_counter() + 2
            while not result.done() and time.perf_counter() < deadline:
                server.poll()
                threading.Event().wait(.001)
            response = result.result(timeout=1)
            assert response["ok"]
            assert response["enabled"] == (op != "hold")
    server.close()
    assert not path.exists()
    with pytest.raises(RuntimeError, match="unavailable"):
        client.request("status")


def test_client_timeout_does_not_leave_an_enable_for_later(control, receiver):
    server, path, _ = control
    executor, clock = receiver
    assert executor.mailbox.receive(_message(1, ready_mask=7))
    with pytest.raises(RuntimeError):
        LocalControlClient(path, timeout=.01).request("enable", "session-a")
    clock.mono += 10_000_001
    server.poll()
    assert not executor.mailbox.enabled
    assert executor.apply_pending() is None


def test_feedback_samples_displaced_plant_not_retained_command(control, receiver, command_plant):
    _, _, request = control
    executor, clock = receiver
    model, data = command_plant.model, command_plant.data
    joint_ids = np.asarray([model.joint(name).id for name in ARM_NAMES])
    qpos_indices = model.jnt_qposadr[joint_ids]
    dof_indices = model.jnt_dofadr[joint_ids]
    # The fixture reverses joints and puts a scene free joint ahead of the robot.
    # qpos and qvel therefore do not share addresses or canonical ordering.
    actual = np.linspace(-.4, .4, 14)
    velocity = np.linspace(.7, -.7, 14)
    data.qpos[qpos_indices] = actual
    data.qvel[dof_indices] = velocity
    retained = np.full(54, .1)
    assert executor.mailbox.receive(_message(1, ready_mask=7, positions=retained))
    enabled = request("enable")
    assert enabled["ok"]
    np.testing.assert_array_equal(enabled["feedback"]["position_rad"], actual)
    np.testing.assert_array_equal(enabled["feedback"]["retained_position_rad"], np.zeros(14))
    executor.apply_pending()
    clock.mono += 12_000_000
    actual += .02
    data.qpos[qpos_indices] = actual
    before_qpos, before_qvel, before_ctrl = data.qpos.copy(), data.qvel.copy(), data.ctrl.copy()
    pending = executor.mailbox._pending
    for operation in ("status", "hold"):
        sampled_ns = clock.mono
        response = request(operation, "observer" if operation == "status" else "session-a")
        assert response["ok"]
        feedback = response["feedback"]
        assert feedback["monotonic_ns"] == sampled_ns
        assert feedback["joint_names"] == list(ARM_NAMES)
        assert feedback["scene_xml"] is None
        np.testing.assert_array_equal(feedback["position_rad"], actual)
        np.testing.assert_array_equal(feedback["velocity_rad_s"], velocity)
        np.testing.assert_array_equal(feedback["retained_position_rad"], retained[:14])
        np.testing.assert_array_equal(data.qpos, before_qpos)
        np.testing.assert_array_equal(data.qvel, before_qvel)
        np.testing.assert_array_equal(data.ctrl, before_ctrl)
        np.testing.assert_array_equal(command_plant.joint_command_targets(), retained)
        if operation == "status":
            assert executor.mailbox.enabled
            assert executor.mailbox.authorized_session == "session-a"
            assert executor.mailbox._pending is pending
        else:
            assert not executor.mailbox.enabled
        clock.mono += 1_000_000
    assert request("status")["ok"]
    assert not executor.mailbox.enabled


def test_feedback_runs_on_poll_owner_and_reports_scene(control, receiver, command_plant, monkeypatch, tmp_path):
    server, path, _ = control
    executor, _ = receiver
    scene = tmp_path / "actual-scene.xml"
    command_plant._mujoco.mj_saveLastXML(str(scene), command_plant.model)
    command_plant.scene_model_path = scene
    command_plant.full_model_path = tmp_path / "not-the-current-scene.xml"
    sampled_threads = []
    sample = executor.arm_feedback

    def observe_sample():
        sampled_threads.append(threading.get_ident())
        return sample()

    monkeypatch.setattr(executor, "arm_feedback", observe_sample)
    response = _roundtrip(server, path)
    assert response["feedback"]["scene_xml"] == str(scene)
    assert sampled_threads == [threading.get_ident()]
    assert not executor.mailbox.enabled


@pytest.mark.parametrize("missing", [True, False])
def test_unavailable_feedback_is_explicit_and_cannot_enable(
    control, receiver, command_plant, monkeypatch, missing,
):
    _, _, request = control
    executor, _ = receiver
    assert executor.mailbox.receive(_message(1, ready_mask=7))
    if missing:
        def unavailable():
            raise ValueError("no actual velocity mapping")
        monkeypatch.setattr(command_plant, "joint_command_velocities", unavailable)
    else:
        command_plant.data.qvel[:] = np.nan
    response = request("enable")
    assert not response["ok"]
    assert response["feedback"] is None
    assert "feedback unavailable" in response["error"]
    assert not executor.mailbox.enabled
    assert executor.apply_pending() is None
    response = request("status")
    assert not response["ok"]
    assert response["feedback"] is None
    # Feedback failure alone must not revoke an existing manual authorization.
    assert executor.authorize(True)
    response = request("status", "observer")
    assert not response["ok"]
    assert executor.mailbox.enabled
    assert executor.mailbox.authorized_session == "session-a"


def test_sampling_deadline_revokes_enable_without_applying(control, receiver, monkeypatch):
    _, _, request = control
    executor, clock = receiver
    assert executor.mailbox.receive(_message(1, ready_mask=7, positions=np.full(54, .1)))
    sample = executor.arm_feedback

    def delayed_sample():
        value = sample()
        clock.mono += 500_000_000
        return value

    monkeypatch.setattr(executor, "arm_feedback", delayed_sample)
    response = request("enable")
    assert not response["ok"]
    assert response["feedback"] is None
    assert not response["enabled"]
    assert executor.apply_pending() is None
    np.testing.assert_array_equal(executor.plant.joint_command_targets(), np.zeros(54))


@pytest.mark.parametrize("corruption", ["joint_order", "stale_time", "future_time", "missing_velocity", "nonfinite"])
def test_client_rejects_misidentified_or_invalid_feedback(control, receiver, monkeypatch, corruption):
    server, path, _ = control
    executor, clock = receiver
    sample = executor.arm_feedback

    def corrupt_sample():
        feedback = sample()
        if corruption == "joint_order":
            feedback["joint_names"].reverse()
        elif corruption == "stale_time":
            feedback["monotonic_ns"] = clock.mono - 1
        elif corruption == "future_time":
            feedback["monotonic_ns"] = clock.mono + 1
        elif corruption == "missing_velocity":
            del feedback["velocity_rad_s"]
        else:
            feedback["position_rad"][0] = float("nan")
        return feedback

    monkeypatch.setattr(executor, "arm_feedback", corrupt_sample)
    with pytest.raises(RuntimeError, match="feedback"):
        _roundtrip(server, path)
    assert not executor.mailbox.enabled


@pytest.mark.parametrize("identity", ["request_id", "session_id"])
def test_client_rejects_response_identity_mismatch(control, monkeypatch, identity):
    server, path, _ = control
    status = server._status

    def wrong_identity(request, error=""):
        response = status(request, error)
        response[identity] = "not-the-request"
        return response

    monkeypatch.setattr(server, "_status", wrong_identity)
    with pytest.raises(RuntimeError, match="mismatch"):
        _roundtrip(server, path)


def test_feedback_packet_overflow_cannot_leave_enable_pending(control, receiver, monkeypatch):
    _, _, request = control
    executor, _ = receiver
    assert executor.mailbox.receive(_message(1, ready_mask=7, positions=np.full(54, .1)))
    # Force the reply with sampled state over the transport cap, while retaining
    # enough space for a bounded refusal and the original request.
    monkeypatch.setattr("spd_vr.local_control._MAX_PACKET", 512)
    response = request("enable")
    assert not response["ok"]
    assert "packet limit" in response["error"]
    assert response["feedback"] is None
    assert not executor.mailbox.enabled
    assert executor.apply_pending() is None
    np.testing.assert_array_equal(executor.plant.joint_command_targets(), np.zeros(54))
