import json
import socket
import threading
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


def wait_for_publisher_match(publisher, timeout: float = 5.0) -> None:
    if publisher.matching_status.matching:
        return
    matched = threading.Event()
    listener = publisher.declare_matching_listener(
        lambda status: matched.set() if status.matching else None
    )
    try:
        if not publisher.matching_status.matching and not matched.wait(timeout):
            raise TimeoutError("timed out waiting for subscriber match")
    finally:
        listener.undeclare()




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

def test_latest_sample_counts_only_unconsumed_overwrites():
    box = LatestSample[int]()
    box.put(1)
    box.put(2)
    assert box.dropped_count == 1
    generation, value = box.take_new(-1)
    assert (generation, value) == (2, 2)
    box.put(3)
    assert box.dropped_count == 1
    box.put(4)
    assert box.dropped_count == 2


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
        wait_for_publisher_match(publisher)
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


class FakeMatchingStatus:
    def __init__(self, matching: bool):
        self.matching = matching


class FakeMatchingListener:
    def __init__(self):
        self.undeclared = False

    def undeclare(self) -> None:
        self.undeclared = True


class FakePublisher:
    def __init__(self, *, match_on_listen: bool = False):
        self.matching_status = FakeMatchingStatus(False)
        self.match_on_listen = match_on_listen
        self.listener: FakeMatchingListener | None = None

    def declare_matching_listener(self, handler):
        self.listener = FakeMatchingListener()
        if self.match_on_listen:
            self.matching_status.matching = True
            handler(self.matching_status)
        return self.listener


def test_wait_for_publisher_match_times_out_and_releases_listener():
    publisher = FakePublisher()

    with pytest.raises(TimeoutError, match="subscriber match"):
        wait_for_publisher_match(publisher, timeout=0)
    assert publisher.listener is not None
    assert publisher.listener.undeclared is True


def test_wait_for_publisher_match_observes_listener_and_releases_it():
    publisher = FakePublisher(match_on_listen=True)

    wait_for_publisher_match(publisher, timeout=0)
    assert publisher.matching_status.matching is True
    assert publisher.listener is not None
    assert publisher.listener.undeclared is True


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
