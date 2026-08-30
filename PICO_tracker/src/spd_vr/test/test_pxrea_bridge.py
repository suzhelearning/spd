import json
import struct

import pytest

from spd_vr.pico_frames import FRAME_TYPE_HAND_LEFT, FRAME_TYPE_HAND_RIGHT
from spd_vr.pxrea_bridge import BridgeCore, main
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


def test_fake_source_jsonl_uses_worker_path_and_rejects_malformed_line(tmp_path, capsys):
    data = tmp_path / "source.jsonl"
    data.write_text(json.dumps({"device_id": "FAKE", "data_hex": "ab", "delay_ms": 0}) + "\n")
    assert main(["--fake-source-jsonl", str(data)]) == 0
    bad = tmp_path / "bad.jsonl"
    bad.write_text('{"device_id":"FAKE","data_hex":"not hex","delay_ms":0}\n')
    assert main(["--fake-source-jsonl", str(bad)]) != 0
