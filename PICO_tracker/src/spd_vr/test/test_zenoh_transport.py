import json
import socket
import time

import pytest

import spd_vr.zenoh_transport as transport
from spd_vr.zenoh_transport import LatestSample, ZenohNode, peer_config


def free_tcp_endpoint() -> str:
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return f"tcp/127.0.0.1:{sock.getsockname()[1]}"


def wait_for_value(mailbox: LatestSample[str], expected: str) -> tuple[int, str]:
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        latest = mailbox.take_new(-1)
        if latest is not None and latest[1] == expected:
            return latest
        time.sleep(0.01)
    pytest.fail(f"timed out waiting for {expected!r}")


def test_latest_sample_overwrites_one_slot_and_invalidates():
    box = LatestSample[int]()
    for value in range(1000):
        box.put(value)

    generation, value = box.take_new(-1)
    assert value == 999
    assert box.storage_size == 1
    assert box.take_new(generation) is None

    box.invalidate()
    assert box.storage_size == 0
    assert box.take_new(-1) is None
    box.put(1000)
    next_generation, value = box.take_new(generation)
    assert next_generation > generation
    assert value == 1000


def test_peer_config_uses_only_the_explicit_endpoint_for_each_role():
    endpoint = "tcp/127.0.0.1:7447"
    listener = peer_config(listen=True, endpoint=endpoint)
    connector = peer_config(listen=False, endpoint=endpoint)

    assert listener.get_json("scouting/multicast/enabled") == "false"
    assert connector.get_json("scouting/multicast/enabled") == "false"
    assert json.loads(listener.get_json("listen/endpoints")) == [endpoint]
    assert json.loads(connector.get_json("connect/endpoints")) == [endpoint]
    assert str(listener).count("7447") == 1
    assert str(connector).count("7447") == 1


def test_two_real_peers_decode_into_latest_slot_and_stop_after_close():
    endpoint = free_tcp_endpoint()
    mailbox = LatestSample[str]()
    decoded_payloads: list[bytes] = []

    def decode(payload: bytes) -> str:
        assert type(payload) is bytes
        decoded_payloads.append(payload)
        return payload.decode()

    receiver = ZenohNode(peer_config(listen=True, endpoint=endpoint))
    sender = ZenohNode(peer_config(listen=False, endpoint=endpoint))
    try:
        receiver.declare_latest_subscriber("spd/test/latest", decode, mailbox)
        publisher = sender.declare_publisher("spd/test/latest")
        for value in range(1000):
            publisher.put(str(value).encode())

        generation, value = wait_for_value(mailbox, "999")
        assert value == "999"
        assert decoded_payloads
        assert mailbox.storage_size == 1

        receiver.close()
        publisher.put(b"after-close")
        time.sleep(0.1)
        assert mailbox.take_new(generation) is None
    finally:
        receiver.close()
        sender.close()


class OwnedResource:
    def __init__(self, name: str, events: list[str], *, fail: bool = False):
        self.name = name
        self.events = events
        self.fail = fail

    def undeclare(self) -> None:
        self.events.append(self.name)
        if self.fail:
            raise RuntimeError(self.name)


class FakeSession:
    def __init__(self, events: list[str], *, fail_subscriber: bool = False):
        self.events = events
        self.fail_subscriber = fail_subscriber

    def declare_subscriber(self, key: str, handler):
        return OwnedResource("subscriber", self.events, fail=self.fail_subscriber)

    def declare_publisher(self, key: str):
        return OwnedResource("publisher", self.events)

    def close(self) -> None:
        self.events.append("session")


def test_node_closes_owned_resources_once_in_reverse_order(monkeypatch):
    events: list[str] = []
    monkeypatch.setattr(transport.zenoh, "open", lambda config: FakeSession(events))

    with ZenohNode(object()) as node:
        node.declare_publisher("key")
        node.declare_latest_subscriber("key", bytes, LatestSample[bytes]())

    node.close()
    assert events == ["subscriber", "publisher", "session"]


def test_node_propagates_close_error_after_releasing_remaining_resources(monkeypatch):
    events: list[str] = []
    monkeypatch.setattr(
        transport.zenoh,
        "open",
        lambda config: FakeSession(events, fail_subscriber=True),
    )
    node = ZenohNode(object())
    node.declare_publisher("key")
    node.declare_latest_subscriber("key", bytes, LatestSample[bytes]())

    with pytest.raises(RuntimeError, match="subscriber"):
        node.close()
    node.close()
    assert events == ["subscriber", "publisher", "session"]
