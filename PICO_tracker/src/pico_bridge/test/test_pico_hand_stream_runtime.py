from pathlib import Path
import importlib.util
import struct


SCRIPT = Path(__file__).parents[1] / "scripts" / "mock_pico_server.py"
SPEC = importlib.util.spec_from_file_location("mock_pico_server", SCRIPT)
MOCK = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MOCK)


def _payload(packet: bytes) -> tuple[int, int, bytes]:
    magic, frame_type, timestamp, payload_len = struct.unpack("<BBqI", packet[:14])
    assert magic == 0xAB
    assert payload_len == len(packet) - 14
    return frame_type, timestamp, packet[14:]


def test_valid_payload_is_fixed_733_bytes_and_ordered():
    left, right = MOCK.hand_packets(42, 0.042, "valid", 0)
    left_type, left_ts, left_payload = _payload(left)
    right_type, right_ts, right_payload = _payload(right)
    assert (left_type, right_type) == (0x38, 0x39)
    assert left_ts == right_ts == 42
    assert len(left_payload) == len(right_payload) == 733
    assert struct.unpack_from("<Bf", left_payload) == (1, 1.0)
    first = struct.unpack_from("<7f", left_payload, 5)
    last = struct.unpack_from("<7f", left_payload, 5 + 25 * 28)
    assert first[3:] == (0.0, 0.0, 0.0, 1.0)
    assert last[3:] == (0.0, 0.0, 0.0, 1.0)


def test_hand_cases_cover_rejection_and_inactive_contracts():
    left, right = MOCK.hand_packets(10, 0.01, "truncated", 0)
    assert len(_payload(left)[2]) == 732
    assert len(_payload(right)[2]) == 733

    left, right = MOCK.hand_packets(10, 0.01, "timestamp-mismatch", 0)
    assert _payload(left)[1] != _payload(right)[1]

    left, right = MOCK.hand_packets(10, 0.01, "inactive", 0)
    assert struct.unpack_from("<B", _payload(left)[2])[0] == 0
    assert struct.unpack_from("<B", _payload(right)[2])[0] == 0
