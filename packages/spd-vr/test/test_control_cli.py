import json

from spd_vr.control_cli import publish_control
from spd_vr.wire import CONTROL_KEY, ControlCommand, decode_control


class _Matching:
    matching = True


class _Publisher:
    matching_status = _Matching()

    def __init__(self, node):
        self.node = node
        self.payloads = []

    def put(self, payload):
        self.payloads.append(payload)
        frame = decode_control(payload)
        for mailbox in self.node.mailboxes.values():
            mailbox.put({"status": "running", "ready": True, "sequence": frame.sequence})

    def declare_matching_listener(self, _callback):
        return type("Listener", (), {"undeclare": lambda self: None})()

def test_allocator_rejects_corrupt_state(tmp_path):
    from spd_vr.control_sequence import ControlSequenceAllocator

    state = tmp_path / "sequence.json"
    state.write_text("{broken", encoding="utf-8")
    try:
        ControlSequenceAllocator(state).allocate()
    except ValueError:
        pass
    else:
        raise AssertionError("corrupt sequence state must fail closed")


class _Node:
    def __init__(self, _config):
        self.mailboxes = {}
        self.publisher = _Publisher(self)
        self.closed = False

    def declare_latest_subscriber(self, key, _decoder, mailbox):
        self.mailboxes[key] = mailbox

    def declare_publisher(self, key, **_kwargs):
        assert key == CONTROL_KEY
        return self.publisher

    def close(self):
        self.closed = True


def test_publish_control_is_acked_by_all_status_consumers(tmp_path):
    holder = {}

    def factory(config):
        holder["node"] = _Node(config)
        return holder["node"]

    frame = publish_control(
        ControlCommand.START,
        sequence_file=tmp_path / "sequence.json",
        node_factory=factory,
        timeout_s=0.2,
    )
    assert frame.sequence == 1
    assert len(holder["node"].publisher.payloads) == 1
    assert holder["node"].closed
