"""Canonical Zenoh key space for SPD VR teleoperation v1."""

TRACKING_KEY = "spd/vr/v1/tracking"
ARM_TARGETS_KEY = "spd/vr/v1/arm_targets"
CONTROL_KEY = "spd/vr/v1/control"
BRIDGE_STATUS_KEY = "spd/vr/v1/status/bridge"
IK_STATUS_KEY = "spd/vr/v1/status/ik"
VIEWER_STATUS_KEY = "spd/vr/v1/status/viewer"

__all__ = [
    "ARM_TARGETS_KEY",
    "BRIDGE_STATUS_KEY",
    "CONTROL_KEY",
    "IK_STATUS_KEY",
    "TRACKING_KEY",
    "VIEWER_STATUS_KEY",
]
