"""Authorization/cancellation contracts without ROS or production solvers."""
import pytest

from spd_vr.ros_publisher import _FollowControl


class HeldSource:
    """Small source state machine: Hold latches, Align creates a new session."""

    def __init__(self):
        self.session_id = "session-0"
        self.fresh = True
        self.input_mask = self.ready_mask = 7
        self.running_mask = 0
        self.state = "hold"
        self.reason = "aligned"
        self.alignments = 0

    def status(self):
        return dict(fresh=self.fresh, input_mask=self.input_mask, ready_mask=self.ready_mask,
                    running_mask=self.running_mask, state=self.state, reason=self.reason)

    def command(self, command):
        if command == "start":
            self.running_mask = self.ready_mask
            return bool(self.running_mask)
        self.running_mask = self.ready_mask = 0
        self.state = "hold"
        if command == "align":
            self.alignments += 1
            self.session_id = f"session-{self.alignments}"
            self.state = "aligning"
        return True


class DeferredReceiver:
    def __init__(self):
        self.requests = []

    def context(self, generation, session):
        self.current = generation, session

    def submit(self, op, generation, session):
        self.requests.append((op, generation, session))


def reply(request, *, transport_error="", **changes):
    op, generation, session = request
    response = dict(ok=True, error="", state="active", enabled=True,
                    authorized_session=session, candidate_session=session, ready_mask=7,
                    hold_mask=0)
    response.update(changes)
    return op, generation, session, None if transport_error else response, transport_error


@pytest.fixture
def controls():
    source, receiver = HeldSource(), DeferredReceiver()
    return source, receiver, _FollowControl(source, receiver)


def test_only_matching_enable_ack_starts_source_not_status(controls):
    source, receiver, control = controls
    control.command("confirm")
    request = receiver.requests[-1]
    assert source.running_mask == 0
    control.result(reply(("status", *request[1:])))
    assert source.running_mask == 0
    control.result(reply(request))
    assert source.running_mask == 7


@pytest.mark.parametrize("cancel", ["hold", "align", "tracking_loss", "session_change"])
def test_cancellation_during_enable_rejects_late_success(controls, cancel):
    source, receiver, control = controls
    control.command("confirm")
    request = receiver.requests[-1]
    if cancel in {"hold", "align"}:
        control.command(cancel)
    else:
        if cancel == "tracking_loss":
            source.input_mask = source.ready_mask = 4
        else:
            source.session_id = "replacement-session"
        control.observe()
    control.result(reply(request))
    assert source.running_mask == 0
    assert not control.pending and not control.following
    assert any(op == "hold" and session == request[2] for op, _, session in receiver.requests)


@pytest.mark.parametrize("changes", [
    {"authorized_session": "other"},
    {"candidate_session": "other"},
    {"enabled": False},
    {"ready_mask": 3},
    {"hold_mask": 1},
    {"ok": False, "error": "candidate not yet fully ready"},
    {"transport_error": "control request timed out"},
])
def test_rejected_or_unconfirmed_enable_never_starts_and_revokes(controls, changes):
    source, receiver, control = controls
    control.command("confirm")
    request = receiver.requests[-1]
    control.result(reply(request, **changes))
    assert source.running_mask == 0
    assert source.ready_mask == 0
    assert receiver.requests[-1][0] == "hold"
    # A later status recovery must never substitute for operator rearming.
    control.result(reply(("status", control.generation, source.session_id)))
    assert source.running_mask == 0


def test_partial_runtime_hold_preserves_other_groups(controls):
    source, receiver, control = controls
    control.command("confirm")
    control.result(reply(receiver.requests[-1]))
    source.input_mask = source.ready_mask = source.running_mask = 4
    control.observe()
    control.result(reply(("status", control.generation, source.session_id),
                         ready_mask=4, hold_mask=3, state="holding"))
    assert source.running_mask == 4
    assert control.following
    # Fresh tracking by itself cannot restore the lost groups.
    source.input_mask = 7
    control.observe()
    assert source.running_mask == 4


@pytest.mark.parametrize("changes", [
    {"enabled": False}, {"candidate_session": "other"},
    {"authorized_session": "other"}, {"transport_error": "control link unavailable"},
])
def test_authorization_loss_stops_and_requires_rearm(controls, changes):
    source, receiver, control = controls
    control.command("confirm")
    control.result(reply(receiver.requests[-1]))
    control.result(reply(("status", control.generation, source.session_id), **changes))
    assert source.running_mask == 0
    assert not control.following
    control.result(reply(("status", control.generation, source.session_id)))
    assert source.running_mask == 0


def test_old_poll_does_not_cancel_new_acknowledgment(controls):
    source, receiver, control = controls
    old_poll = ("status", control.generation, source.session_id)
    control.command("confirm")
    control.result(reply(receiver.requests[-1]))
    control.result(reply(old_poll, enabled=False))
    assert source.running_mask == 7
    assert control.following


def test_all_groups_held_is_not_reported_as_following(controls):
    source, receiver, control = controls
    control.command("confirm")
    control.result(reply(receiver.requests[-1]))
    source.input_mask = source.ready_mask = source.running_mask = 0
    control.observe()
    assert control.status()["control_phase"] == "held"
    assert not control.following
    source.input_mask = 7
    control.observe()
    assert source.running_mask == 0


def test_confirm_refuses_partial_or_incomplete_calibration(controls):
    source, receiver, control = controls
    source.ready_mask = 6
    source.state = "aligning"
    control.command("confirm")
    assert not receiver.requests
    assert source.running_mask == 0
