#!/usr/bin/env python3
import csv
import hashlib
import json
from pathlib import Path
import struct
import sys
import tempfile
import unittest
import zlib


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from run_pico_trace_algorithm_benchmark import (  # noqa: E402
    build_replay_command,
    build_viewer_command,
    ensure_manifest_compatible,
    inspect_trace,
    slice_trace,
    validate_completed_run,
)


def make_packet(sequence: int, source_ns: int, size: int = 160) -> bytes:
    packet = bytearray(size)
    packet[:4] = b"TJVR"
    struct.pack_into("<HH", packet, 4, 1, size)
    struct.pack_into("<Q", packet, 8, sequence)
    struct.pack_into("<q", packet, 24, source_ns)
    struct.pack_into("<I", packet, size - 4, zlib.crc32(packet[:-4]))
    return bytes(packet)


def write_trace(path: Path, count: int = 4) -> None:
    packet_size = 160
    with path.open("wb") as stream:
        stream.write(struct.pack("<4sHHQ", b"TJVT", 1, packet_size, count))
        for index in range(count):
            relative_ns = index * 10_000_000
            stream.write(struct.pack("<q", relative_ns))
            stream.write(make_packet(index + 1, 1_000_000_000 + relative_ns))


class RunnerUnitTests(unittest.TestCase):
    def test_inspect_trace_reports_provenance(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "input.tjvr"
            write_trace(path)
            metadata = inspect_trace(path)

            self.assertEqual(metadata["frame_count"], 4)
            self.assertEqual(metadata["packet_size"], 160)
            self.assertAlmostEqual(metadata["source_duration_s"], 0.03)
            self.assertEqual(
                metadata["sha256"], hashlib.sha256(path.read_bytes()).hexdigest()
            )

    def test_commands_are_deterministic_and_explicit(self):
        viewer = build_viewer_command(
            viewer=Path("build/viewer"),
            config=Path("config/test.yaml"),
            model=Path("models/test.xml"),
            algorithm="spark_upper_qpoases_velocity_qp",
            port=15123,
            duration_s=12.5,
            telemetry=Path("out/cart.csv"),
            joint_telemetry=Path("out/joint.csv"),
        )
        replay = build_replay_command(
            python=Path("python3"),
            replay_tool=Path("tools/replay.py"),
            trace=Path("trace.tjvr"),
            port=15123,
            lead_s=0.25,
        )

        self.assertEqual(viewer[0], "build/viewer")
        self.assertIn("--model-state-only", viewer)
        self.assertEqual(viewer[viewer.index("--algorithm") + 1],
                         "spark_upper_qpoases_velocity_qp")
        self.assertEqual(viewer[viewer.index("--pico-port") + 1], "15123")
        self.assertEqual(replay[-2:], ["--lead", "0.250000"])

    def test_manifest_provenance_mismatch_is_rejected(self):
        expected = {
            "schema_version": 1,
            "provenance": {"trace_sha256": "abc", "git_commit": "123"},
            "algorithms": ["a", "b"],
        }
        existing = json.loads(json.dumps(expected))
        existing["provenance"]["trace_sha256"] = "different"

        with self.assertRaisesRegex(ValueError, "provenance"):
            ensure_manifest_compatible(existing, expected)

    def test_completed_run_requires_full_clean_trace(self):
        with tempfile.TemporaryDirectory() as directory:
            telemetry = Path(directory) / "telemetry.csv"
            with telemetry.open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=[
                    "algorithm", "pico_datagrams", "pico_malformed_packets",
                    "pico_crc_failures", "pico_reordered_packets",
                    "control_failures",
                ])
                writer.writeheader()
                writer.writerow({
                    "algorithm": "algo", "pico_datagrams": "4",
                    "pico_malformed_packets": "0", "pico_crc_failures": "0",
                    "pico_reordered_packets": "0", "control_failures": "0",
                })
            result = validate_completed_run(telemetry, "algo", 4)
            self.assertEqual(result["pico_datagrams"], 4)

            with telemetry.open() as stream:
                rows = list(csv.DictReader(stream))
            rows[0]["control_failures"] = "1"
            with telemetry.open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
                writer.writeheader()
                writer.writerows(rows)
            with self.assertRaisesRegex(ValueError, "control_failures"):
                validate_completed_run(telemetry, "algo", 4)
            accepted = validate_completed_run(
                telemetry, "algo", 4, allow_control_failures=True
            )
            self.assertEqual(accepted["control_failures"], 1)

    def test_slice_trace_preserves_valid_records(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "source.tjvr"
            destination = Path(directory) / "slice.tjvr"
            write_trace(source, count=7)

            slice_trace(source, destination, frame_count=3)
            metadata = inspect_trace(destination)

            self.assertEqual(metadata["frame_count"], 3)
            self.assertEqual(metadata["packet_size"], 160)
            self.assertAlmostEqual(metadata["source_duration_s"], 0.02)


if __name__ == "__main__":
    unittest.main()
