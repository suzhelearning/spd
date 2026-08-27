"""SPD VR simulation-only runtime.

This package intentionally contains no real-device transport, vendor SDK, or
hardware emergency-stop integration.  Those belong to the separate
``spd-teleop`` branch.  Runtime inputs are PICO frames and the versioned arm
UDP protocol; outputs are MuJoCo state and replayable episode files.
"""

SIM_ONLY = True
HARDWARE_BACKENDS = ()
PICO_HAND_SAMPLE_RELATIVE_PATH = "data/pico_hand_samples/raw/pico_hands_60s.npz"

__all__ = ["SIM_ONLY", "HARDWARE_BACKENDS", "PICO_HAND_SAMPLE_RELATIVE_PATH"]
