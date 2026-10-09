"""Safety boundaries: stale clocks and missing XR joints cannot arm teleop."""
import json
import unittest

from pico2_hands.reference.official_pico import pico_official_hand_observations
from pico2_hands.reference.runtime import pico_frame_observations
from webxr.protocol import ProtocolError, TrackingReceiver, tracking_frame


def tracking(sequence=1, stamp=1010., generation=1):
    joints = [[i * .01, ((i * i) % 7) * .005, 1 + i * .001, 0., 0., 0., 1.]
              for i in range(25)]
    return {"type": "tracking", "generation": generation, "sequence": sequence,
            "time_ms": stamp, "head": [0., 0., 1.6, 0., 0., 0., 1.],
            "hands": {"left": joints, "right": [pose.copy() for pose in joints]}}


def ready_receiver():
    receiver = TrackingReceiver("test-webxr", 1)
    receiver.change_scene(1)
    challenge = receiver.challenge(now_ns=1_000_000_000)
    receiver.acknowledge_clock({**challenge, "time_ms": 1000.}, now_ns=1_010_000_000)
    return receiver


class WebXRProtocolTests(unittest.TestCase):
    def test_missing_wrist_and_fingertip_cannot_be_valid_retarget_observations(self):
        message = tracking()
        message["hands"]["left"][0] = None
        message["hands"]["right"][9] = None
        frame = tracking_frame(message, receiver_instance_id="webxr", connection_generation=1,
                               receiver_frame_sequence=1, received_timestamp_ns=1_020_000_000,
                               raw_packet=json.dumps(message).encode())
        official = pico_official_hand_observations(frame)
        arms = pico_frame_observations(frame)
        self.assertFalse(official["left"].valid)
        self.assertFalse(official["right"].valid)
        self.assertFalse(arms["left"][1].valid)
        self.assertTrue(arms["right"][1].valid)

    def test_replayed_or_buffered_source_cannot_be_refreshed_by_new_receive_time(self):
        receiver = ready_receiver()
        message = tracking()
        receiver.accept(message, json.dumps(message), now_ns=1_020_000_000)
        duplicate = tracking(sequence=2)
        with self.assertRaises(ProtocolError):
            receiver.accept(duplicate, json.dumps(duplicate), now_ns=1_030_000_000)
        delayed = tracking(sequence=2, stamp=1020.)
        with self.assertRaises(ProtocolError):
            receiver.accept(delayed, json.dumps(delayed), now_ns=1_300_000_000)
        future = tracking(sequence=2, stamp=1400.)
        with self.assertRaises(ProtocolError):
            receiver.accept(future, json.dumps(future), now_ns=1_040_000_000)

    def test_scene_acknowledgement_does_not_reset_sequence_or_accept_old_scene(self):
        receiver = ready_receiver()
        message = tracking()
        receiver.accept(message, json.dumps(message), now_ns=1_020_000_000)
        with self.assertRaises(ProtocolError):
            receiver.challenge(now_ns=1_030_000_000)
        receiver.invalidate("scene switch", now_ns=1_050_000_000)
        receiver.change_scene(2)
        challenge = receiver.challenge(now_ns=1_100_000_000)
        receiver.acknowledge_clock({**challenge, "time_ms": 1100.}, now_ns=1_110_000_000)
        for stale in (tracking(sequence=2, stamp=1110., generation=1),
                      tracking(sequence=1, stamp=1110., generation=2),
                      tracking(sequence=2, stamp=1090., generation=2)):
            with self.assertRaises(ProtocolError):
                receiver.accept(stale, json.dumps(stale), now_ns=1_120_000_000)
        current = tracking(sequence=2, stamp=1110., generation=2)
        frame = receiver.accept(current, json.dumps(current), now_ns=1_120_000_000)
        self.assertTrue(pico_official_hand_observations(frame)["left"].valid)

    def test_inactive_visibility_edge_invalidates_identity_even_before_dispatch(self):
        receiver = ready_receiver()
        before = tracking()
        first = receiver.accept(before, json.dumps(before), now_ns=1_020_000_000)
        inactive = tracking(sequence=2, stamp=1011.)
        inactive["head"] = None
        inactive["hands"] = {"left": None, "right": None}
        receiver.accept(inactive, json.dumps(inactive), now_ns=1_021_000_000)
        after = tracking(sequence=3, stamp=1012.)
        resumed = receiver.accept(after, json.dumps(after), now_ns=1_022_000_000)
        old_observation = pico_official_hand_observations(first)["left"]
        new_observation = pico_official_hand_observations(resumed)["left"]
        self.assertNotEqual(old_observation.source_instance_id, new_observation.source_instance_id)

    def test_delayed_or_wrong_clock_nonce_cannot_establish_source_freshness(self):
        receiver = TrackingReceiver("test-webxr", 1)
        receiver.change_scene(1)
        challenge = receiver.challenge(now_ns=1_000_000_000)
        with self.assertRaises(ProtocolError):
            receiver.acknowledge_clock({**challenge, "nonce": "different", "time_ms": 1000.},
                                       now_ns=1_010_000_000)
        with self.assertRaises(ProtocolError):
            receiver.acknowledge_clock({**challenge, "time_ms": 1000.}, now_ns=1_300_000_000)
        message = tracking()
        with self.assertRaises(ProtocolError):
            receiver.accept(message, json.dumps(message), now_ns=1_310_000_000)


if __name__ == "__main__":
    unittest.main()
