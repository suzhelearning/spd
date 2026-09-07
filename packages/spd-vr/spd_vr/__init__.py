"""PICO_2-driven SPD VR simulation runtime.

The runtime consumes the direct PICO_2 tracking stream and arm targets, then
simulates the Tianji arm and Wuji Hand 2 plant with replayable episode output.
It has no robot-hardware control backend.
"""

SIM_ONLY = True
HARDWARE_BACKENDS = ()
PICO_HAND_SAMPLE_RELATIVE_PATH = "data/pico_hand_samples/raw/pico_hands_60s.npz"

__all__ = ["SIM_ONLY", "HARDWARE_BACKENDS", "PICO_HAND_SAMPLE_RELATIVE_PATH"]
