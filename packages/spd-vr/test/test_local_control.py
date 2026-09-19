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
                return json.loads(peer.recv(4096))

            yield server, path, request
        server.close()


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
