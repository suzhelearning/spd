"""Bounded WebXR wire input and the existing PICO/FLU observation boundary."""
from __future__ import annotations

from dataclasses import replace
import json
import math
import secrets
import struct
import time

import numpy as np

from pico2_hands.reference.models import (
    PICO_JOINT_NAMES, PicoRawFrame, PicoRawHand, PicoRawJoint,
)

MAX_MESSAGE_BYTES = 48 * 1024
MAX_SAFE_INTEGER = 2**53 - 1
MAX_SOURCE_AGE_NS = 150_000_000
MAX_CLOCK_RTT_NS = 250_000_000
STATE_HEADER = struct.Struct("<4sIIId")
COMMAND_KEYS = frozenset(("r", "s", "d", "q"))
_ZERO_POSE = np.zeros(7, dtype=np.float64)


class ProtocolError(ValueError):
    """The peer must reconnect after malformed, reordered or stale input."""


def positive_integer(value, field, maximum=MAX_SAFE_INTEGER):
    if type(value) is not int or not 0 < value <= maximum:
        raise ProtocolError(f"{field} must be a positive bounded integer")
    return value


def source_time(value):
    if type(value) not in (float, int) or not math.isfinite(value) or not 0 <= value <= MAX_SAFE_INTEGER / 1_000_000:
        raise ProtocolError("time_ms must be a finite nonnegative source timestamp")
    return float(value)


def decode_message(raw):
    if not isinstance(raw, str) or len(raw.encode("utf-8")) > MAX_MESSAGE_BYTES:
        raise ProtocolError("input message exceeds its bound")
    try:
        message = json.loads(raw)
    except (ValueError, RecursionError) as error:
        raise ProtocolError("input is not valid JSON") from error
    if not isinstance(message, dict) or not isinstance(message.get("type"), str):
        raise ProtocolError("input must be a typed JSON object")
    return message


def _pose(value, field):
    if value is None:
        return None
    if not isinstance(value, list) or len(value) != 7 or any(type(x) not in (float, int) for x in value):
        raise ProtocolError(f"{field} must be null or seven finite numbers")
    pose = np.asarray(value, dtype=np.float64)
    if not np.isfinite(pose).all() or np.max(np.abs(pose[:3])) > 100:
        raise ProtocolError(f"{field} exceeds the 100 m tracking bounds")
    norm = float(np.linalg.norm(pose[3:]))
    if not .9 <= norm <= 1.1:
        raise ProtocolError(f"{field} quaternion must be normalized")
    pose[3:] /= norm
    return pose


def tracking_frame(message, *, receiver_instance_id, connection_generation,
                   receiver_frame_sequence, received_timestamp_ns, raw_packet):
    """Map XR wrist + 24 joints to PICO palm + wrist + 24 without retargeting.

    Positions and orientations have already been placed in world FLU by the
    client. A missing wrist never becomes valid through synthesized zero data;
    all other joints retain their own independent validity. The original JSON
    is retained as provenance, not disguised as a native PICO TCP packet.
    """
    timestamp = source_time(message.get("time_ms"))
    head = _pose(message.get("head"), "head")
    values = message.get("hands")
    if not isinstance(values, dict) or set(values) != {"left", "right"}:
        raise ProtocolError("hands must contain left and right")
    hands = {}
    flags = int(head is not None)
    for side, bit in (("left", 2), ("right", 4)):
        joints = values[side]
        if joints is None:
            poses = [None] * 25
        elif isinstance(joints, list) and len(joints) == 25:
            poses = [_pose(value, f"{side}[{index}]") for index, value in enumerate(joints)]
        else:
            raise ProtocolError(f"{side} must be null or exactly 25 joint poses")
        wrist = poses[0]
        valid = wrist is not None
        if valid:
            flags |= bit
        # The pinned pico_to_mediapipe adapter ignores palm index 0.
        raw_joints = tuple(PicoRawJoint(index, PICO_JOINT_NAMES[index], pose is not None,
                                       _ZERO_POSE if pose is None else pose, 0.)
                           for index, pose in enumerate([wrist, *poses]))
        hands[side] = PicoRawHand(valid, valid, _ZERO_POSE if wrist is None else wrist, raw_joints)
    return PicoRawFrame(
        source_timestamp_ms=int(timestamp), protocol_version=1, flags=flags,
        joint_count=26, head_valid=head is not None,
        head_pose=_ZERO_POSE if head is None else head, hands=hands,
        raw_packet=raw_packet, received_timestamp_ns=received_timestamp_ns,
        receiver_instance_id=receiver_instance_id, connection_generation=connection_generation,
        receiver_frame_sequence=receiver_frame_sequence,
    )


class TrackingReceiver:
    """One socket's monotonic source clock; generations never reset its sequence.

    A nonce exchange bounds the unknown browser/host clock offset. Using the
    entire send/reply interval (not a guessed symmetric RTT) rejects delayed
    source samples conservatively. Re-acknowledging an existing scene cannot
    reset this clock or authorize old samples.
    """

    def __init__(self, receiver_instance_id, connection_generation):
        self.receiver_instance_id = receiver_instance_id
        self.connection_generation = connection_generation
        self.generation = 0
        self.sequence = 0
        self.timestamp_ms = -1.
        self.frame_sequence = 0
        self.clock_nonce = None
        self.clock_sent_ns = 0
        self.clock_reply_ns = 0
        self.clock_source_ms = None
        self.latest_frame = None
        self.latest_received_ns = 0
        self.invalidated = True

    def change_scene(self, generation):
        self.generation = generation
        self.clock_nonce = None
        self.clock_source_ms = None
        self.latest_frame = None
        self.latest_received_ns = 0

    def challenge(self, now_ns=None):
        if self.clock_nonce is not None or self.clock_source_ms is not None:
            raise ProtocolError("scene already acknowledged")
        self.clock_nonce = secrets.token_urlsafe(24)
        self.clock_sent_ns = time.monotonic_ns() if now_ns is None else now_ns
        return {"type": "clock", "generation": self.generation, "nonce": self.clock_nonce}

    def acknowledge_clock(self, message, now_ns=None):
        now = time.monotonic_ns() if now_ns is None else now_ns
        if (self.clock_nonce is None or message.get("nonce") != self.clock_nonce
                or message.get("generation") != self.generation):
            raise ProtocolError("unexpected clock acknowledgement")
        if not 0 <= now - self.clock_sent_ns <= MAX_CLOCK_RTT_NS:
            raise ProtocolError("clock handshake timed out; reconnect")
        stamp = source_time(message.get("time_ms"))
        if stamp < self.timestamp_ms:
            raise ProtocolError("source clock rolled back")
        self.clock_reply_ns = now
        self.clock_source_ms = stamp
        self.clock_nonce = None

    def accept(self, message, raw, now_ns=None):
        now = time.monotonic_ns() if now_ns is None else now_ns
        if self.clock_source_ms is None or message.get("generation") != self.generation:
            raise ProtocolError("tracking requires acknowledged scene and clock")
        sequence = positive_integer(message.get("sequence"), "sequence")
        stamp = source_time(message.get("time_ms"))
        if sequence <= self.sequence or stamp <= self.timestamp_ms:
            raise ProtocolError("tracking sequence and source time must strictly increase")
        delta_ns = int((stamp - self.clock_source_ms) * 1_000_000)
        if delta_ns < 0:
            raise ProtocolError("tracking predates the scene acknowledgement")
        if now - (self.clock_sent_ns + delta_ns) > MAX_SOURCE_AGE_NS:
            raise ProtocolError("tracking source is stale")
        if self.clock_sent_ns + delta_ns > now + 20_000_000:
            raise ProtocolError("tracking source clock is in the future")
        self.frame_sequence += 1
        frame = tracking_frame(message, receiver_instance_id=self.receiver_instance_id,
                               connection_generation=self.connection_generation,
                               receiver_frame_sequence=self.frame_sequence,
                               received_timestamp_ns=now, raw_packet=raw.encode("utf-8"))
        inactive = not (frame.head_valid and all(hand.wrist_valid for hand in frame.hands.values()))
        if inactive and not self.invalidated:
            self.connection_generation += 1
            frame = replace(frame, connection_generation=self.connection_generation)
        self.sequence, self.timestamp_ms = sequence, stamp
        self.latest_frame, self.latest_received_ns = frame, now
        self.invalidated = inactive
        return frame

    def invalidate(self, reason, now_ns=None):
        """An identity edge invalidates a bound teleop reference immediately."""
        now = time.monotonic_ns() if now_ns is None else now_ns
        self.connection_generation += 1
        self.frame_sequence += 1
        self.latest_frame = None
        self.latest_received_ns = 0
        self.invalidated = True
        message = {"time_ms": max(0., self.timestamp_ms), "head": None,
                   "hands": {"left": None, "right": None}}
        return tracking_frame(message, receiver_instance_id=self.receiver_instance_id,
                              connection_generation=self.connection_generation,
                              receiver_frame_sequence=self.frame_sequence,
                              received_timestamp_ns=now,
                              raw_packet=json.dumps({"type": "invalid", "reason": reason}).encode())


def encode_state(generation, sequence, sim_time, poses):
    """World body poses, already copied by the physics owner, little-endian."""
    return STATE_HEADER.pack(b"SPDS", generation, sequence, len(poses), sim_time) + poses.tobytes()
