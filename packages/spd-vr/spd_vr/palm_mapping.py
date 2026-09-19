"""Frozen operator heading and rigid anatomical palms for paired arm mapping.

The operator explicitly declares fingers-forward/palms-down at calibration.
Tracking wrist quaternion conventions need not match between hands: each local
wrist-to-palm rotation is learned independently from that declared pose.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any
import xml.etree.ElementTree as ET

import numpy as np

from .alignment import SideAlignment, _pose_matrix


_SIDES = {"left": "l", "right": "r"}


def _unit(vector: np.ndarray, description: str) -> np.ndarray:
    length = float(np.linalg.norm(vector))
    if not np.isfinite(length) or length < 1.0e-8:
        raise ValueError(f"degenerate {description}")
    return vector / length


def _robot_palm(root: ET.Element, side: str, prefix: str) -> np.ndarray:
    """Build palm axes from rigid MCP locations in the URDF wrist frame.

    +X runs from wrist to middle MCP. Orthogonalized index-minus-pinky gives
    +Y on the right, -Y on the left. +Z = X cross Y is dorsal for both hands.
    The center is halfway from wrist to middle MCP, not a moving finger link.
    """
    points: dict[str, np.ndarray] = {}
    for finger in ("index_finger", "middle_finger", "pinky"):
        name = f"{prefix}_{finger}_mcp_flex"
        matches = root.findall(f"joint[@name='{name}']")
        if len(matches) != 1:
            raise ValueError(f"expected exactly one URDF joint {name}")
        joint = matches[0]
        parent = joint.find("parent")
        origin = joint.find("origin")
        if parent is None or parent.get("link") != f"{prefix}_wrist":
            raise ValueError(f"{name} must be rooted directly in {prefix}_wrist")
        if origin is None:
            raise ValueError(f"{name} has no rigid MCP origin")
        point = np.asarray([float(value) for value in origin.get("xyz", "").split()])
        if point.shape != (3,) or not np.all(np.isfinite(point)):
            raise ValueError(f"{name} must have a finite three-dimensional origin")
        points[finger] = point
    forward = _unit(points["middle_finger"], f"{side} wrist-to-middle direction")
    transverse = points["index_finger"] - points["pinky"]
    if side == "left":
        transverse = -transverse
    left = _unit(transverse - forward * np.dot(forward, transverse), f"{side} MCP transverse direction")
    dorsal = _unit(np.cross(forward, left), f"{side} dorsal direction")
    result = np.eye(4)
    result[:3, :3] = np.column_stack((forward, left, dorsal))
    result[:3, 3] = 0.5 * points["middle_finger"]
    return _pose_matrix(result)


class PalmMapping:
    """Separate orientation calibration from repeatable position alignment.

    Calibration is atomic and requires both hands. Its epoch is advisory to
    the source owner, which must reset/block calibration after reconnection.
    This object does not infer whether the declared standard pose is correct
    or stable; the source collects stable samples before calling ``calibrate``.
    """

    def __init__(self, robot_wrist_to_palm: dict[str, np.ndarray]) -> None:
        self.robot_wrist_to_palm = {
            side: _pose_matrix(robot_wrist_to_palm[side]) for side in _SIDES
        }
        for transform in self.robot_wrist_to_palm.values():
            transform.setflags(write=False)
        self._basis: np.ndarray | None = None
        self._human_wrist_to_palm: dict[str, np.ndarray] = {}
        self._epoch: int | None = None

    @classmethod
    def from_urdf(cls, path: str | Path) -> PalmMapping:
        root = ET.parse(path).getroot()
        return cls({side: _robot_palm(root, side, prefix) for side, prefix in _SIDES.items()})

    @property
    def calibrated(self) -> bool:
        return self._basis is not None

    @property
    def epoch(self) -> int | None:
        return self._epoch

    def calibrate(
        self,
        head_pose: Any,
        wrists: dict[str, np.ndarray],
        palms: dict[str, np.ndarray],
        epoch: int,
    ) -> None:
        """Freeze head yaw and each hand's declared standard-palm frame.

        PICO is already FLU. Project head-local +X onto world horizontal; do
        not use head roll/pitch as robot axes. Palm positions are PICO palm0,
        wrist poses are PICO wrist1, both in tracking-world coordinates.
        """
        if isinstance(epoch, bool) or not isinstance(epoch, (int, np.integer)) or epoch < 0:
            raise ValueError("calibration epoch must be a non-negative integer")
        head = _pose_matrix(head_pose)
        forward = head[:3, 0].copy()
        forward[2] = 0.0
        forward = _unit(forward, "horizontal head heading")
        up = np.array([0.0, 0.0, 1.0])
        heading = np.column_stack((forward, np.cross(up, forward), up))
        human: dict[str, np.ndarray] = {}
        for side in _SIDES:
            wrist = _pose_matrix(wrists[side])
            palm = np.asarray(palms[side], dtype=float)
            if palm.shape != (3,) or not np.all(np.isfinite(palm)):
                raise ValueError(f"{side} palm must be a finite three-vector")
            transform = np.eye(4)
            transform[:3, :3] = wrist[:3, :3].T @ heading
            transform[:3, 3] = wrist[:3, :3].T @ (palm - wrist[:3, 3])
            human[side] = _pose_matrix(transform)
        self._basis = heading.T.copy()
        self._human_wrist_to_palm = human
        self._epoch = int(epoch)

    def configure_alignment(self, side: str, alignment: SideAlignment) -> None:
        if side not in _SIDES:
            raise ValueError(f"unknown palm side: {side}")
        if self._basis is None:
            raise ValueError("standard-palm calibration is required before position alignment")
        alignment.configure_palm_mapping(
            self._basis, self._human_wrist_to_palm[side], self.robot_wrist_to_palm[side]
        )

    def reset(self) -> None:
        """Invalidate calibration; existing alignment users must be held by owner."""
        self._basis = None
        self._human_wrist_to_palm.clear()
        self._epoch = None

    def status(self) -> dict[str, Any]:
        return {
            "calibrated": self.calibrated,
            "epoch": self.epoch,
            "basis": None if self._basis is None else self._basis.tolist(),
            "human_wrist_to_palm": {
                side: transform.tolist() for side, transform in self._human_wrist_to_palm.items()
            },
            "robot_wrist_to_palm": {
                side: transform.tolist() for side, transform in self.robot_wrist_to_palm.items()
            },
        }


__all__ = ["PalmMapping"]
