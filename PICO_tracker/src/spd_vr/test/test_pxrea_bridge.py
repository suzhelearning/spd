import json
import struct
import time
from pathlib import Path
import pytest

from spd_vr.pico_frames import FRAME_TYPE_HAND_LEFT, FRAME_TYPE_HAND_RIGHT
from spd_vr.pxrea_bridge import BridgeCore, BridgeWorker, _run_fake_source
from spd_vr.pxrea_sdk import (
    BoundedCallbackQueue,
    CallbackEvent,
    PXREA_DEVICE_MISSING,
    PXREA_DEVICE_STATE_JSON,
)
from spd_vr.wire import decode_tracking


def hand_frame(frame_type: int, timestamp: int) -> bytes:
    payload = bytes([0, 0, 0x80, 0x3F]) + bytes(729)
    return struct.pack("<BBqI", 0xAB, frame_type, timestamp, len(payload)) + payload


def test_core_pairs_hands_and_publishes_1540_bytes():
    core = BridgeCore(selected_device_id="FAKE")
    published = core.accept_event(("FAKE", hand_frame(FRAME_TYPE_HAND_LEFT, 10)))
    assert published == []
    published = core.accept_event(("FAKE", hand_frame(FRAME_TYPE_HAND_RIGHT, 10)))
    assert len(published) == 1
    assert len(published[0]) == 1540
    assert decode_tracking(published[0]).left_active is False
    assert decode_tracking(published[0]).right_active is False


def test_core_publishes_xrobotoolkit_state_json():
    fixture = (
        Path(__file__).parents[2]
        / "pico_bridge/test/fixtures/xrobotoolkit_hand_state.json"
    )
    payload = json.loads(fixture.read_text(encoding="utf-8"))
    state = json.loads(payload["value"])
    for side in state["Hand"].values():
        side["isActive"] = int(side["isActive"])
        for joint in side["HandJointLocations"]:
            joint["p"] = joint["p"].replace(" ", ",")
    payload["value"] = json.dumps(state)
    core = BridgeCore()
    published = core.accept_event(
        CallbackEvent("FAKE", json.dumps(payload).encode(), PXREA_DEVICE_STATE_JSON)
    )
    assert len(published) == 1
    frame = decode_tracking(published[0])
    assert core.status()["device_id"] == "FAKE"
    assert frame.source_timestamp_ns == 1724846400000000000
    assert frame.left_active is True
    assert frame.right_active is False
    assert frame.left_hand[1, :3].tolist() == pytest.approx([0.101, 0.201, 0.301])


def test_device_selection_is_ambiguous_without_explicit_selection():
    core = BridgeCore()
    core.accept_event(("A", hand_frame(FRAME_TYPE_HAND_LEFT, 1)))
    assert core.accept_event(("B", hand_frame(FRAME_TYPE_HAND_RIGHT, 1))) == []
    assert core.status()["device_selection_ambiguous"] is True


def test_reset_increments_epoch_and_invalid_payload_is_counted():
    core = BridgeCore(selected_device_id="FAKE")
    core.accept_event(("FAKE", b"bad"))
    assert core.status()["invalid_payloads"] == 1
    old = core.epoch
    core.reset_device()
    assert core.epoch == old + 1



def test_lifecycle_event_clears_pending_pair_and_increments_epoch():
    core = BridgeCore(selected_device_id="FAKE")
    core.accept_event(("FAKE", hand_frame(FRAME_TYPE_HAND_LEFT, 10)))
    epoch = core.epoch
    core.accept_event(CallbackEvent("", b"", PXREA_DEVICE_MISSING))
    assert core.epoch == epoch + 1
    assert core.accept_event(("FAKE", hand_frame(FRAME_TYPE_HAND_RIGHT, 10))) == []

def test_worker_publishes_tracking_and_status():
    queue = BoundedCallbackQueue()
    core = BridgeCore(selected_device_id="FAKE")
    tracking = []
    statuses = []
    core.set_ready()
    worker = BridgeWorker(queue, core, tracking.append, statuses.append)
    worker.start()
    queue.put(CallbackEvent("FAKE", hand_frame(FRAME_TYPE_HAND_LEFT, 10)))
    queue.put(CallbackEvent("FAKE", hand_frame(FRAME_TYPE_HAND_RIGHT, 10)))
    deadline = time.monotonic() + 1
    while len(tracking) < 1 and time.monotonic() < deadline:
        time.sleep(0.01)
    worker.stop()
    assert len(tracking) == 1
    status = json.loads(statuses[-1])
    assert {"ready", "device_id", "tracking_epoch", "published", "invalid_payloads", "dropped"} <= status.keys()
    assert status["published"] == 1
def test_fake_source_jsonl_uses_worker_path_and_rejects_malformed_line(tmp_path, capsys):
    data = tmp_path / "source.jsonl"
    data.write_text(json.dumps({"device_id": "FAKE", "data_hex": "ab", "delay_ms": 0}) + "\n")
    assert _run_fake_source(data, [].append, [].append) == 0
    bad = tmp_path / "bad.jsonl"
    bad.write_text('{"device_id":"FAKE","data_hex":"not hex","delay_ms":0}\n')
    assert _run_fake_source(bad, [].append, [].append) != 0
