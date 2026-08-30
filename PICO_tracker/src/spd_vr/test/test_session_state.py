from types import SimpleNamespace

import pytest

from spd_vr.session_state import SessionController, SessionState
from spd_vr.wire import ControlCommand, ControlFrame

def frame(sequence, command):
    return ControlFrame(sequence, sequence + 1, command)


def test_session_sequence_gate_and_lifecycle_are_idempotent():
    calls = []

    class Plant:
        def set_paused(self, value):
            calls.append(("pause", value))

        def reset_home(self):
            calls.append(("reset",))

        def require_fresh_alignment(self):
            calls.append(("align",))

        def shutdown(self):
            calls.append(("shutdown",))

    controller = SessionController(Plant())
    assert controller.apply(frame(1, ControlCommand.START)).state is SessionState.RUNNING
    assert controller.apply(frame(2, ControlCommand.PAUSE)).state is SessionState.PAUSED
    assert controller.apply(frame(2, ControlCommand.PAUSE)) == controller.snapshot
    assert controller.apply(frame(3, ControlCommand.RESUME)).requires_fresh_alignment
    assert controller.snapshot.state is SessionState.RUNNING
    assert controller.apply(frame(4, ControlCommand.REALIGN)).requires_fresh_alignment
    assert controller.apply(frame(5, ControlCommand.RESET)).state is SessionState.IDLE
    assert controller.apply(frame(6, ControlCommand.SHUTDOWN)).state is SessionState.SHUTDOWN
    assert calls == [
        ("align",),
        ("pause", True),
        ("pause", False),
        ("align",),
        ("align",),
        ("reset",),
        ("shutdown",),
    ]


def test_session_rejects_sequence_rollback_and_conflicting_duplicate():
    controller = SessionController()
    controller.apply(frame(2, ControlCommand.PAUSE))
    with pytest.raises(ValueError, match="sequence_rollback"):
        controller.apply(frame(1, ControlCommand.RESUME))
    with pytest.raises(ValueError, match="duplicate_conflict"):
        controller.apply(frame(2, ControlCommand.RESUME))
