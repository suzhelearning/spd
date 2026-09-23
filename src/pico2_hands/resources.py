"""Repository-local resources for the standalone PICO-to-SPD route.

The editable package, native sources, installed workers, and isolated Hand2
runtime all belong to this repository. Resolution never uses the process cwd,
ROS package overlays, environment overrides, or another source checkout.
"""
from __future__ import annotations

import os
from pathlib import Path

PACKAGE = Path(__file__).resolve().parent
ROOT = PACKAGE.parents[1]
NATIVE = ROOT / "src/teleop_native"
INSTALL = ROOT / ".teleop/install"
HAND_ROOT = ROOT / "tools/wuji_hand_native"
DISPLAY_MODEL = "marvin_m6_wuji2_shared_root_ceres.xml"


class ResourceNotFound(FileNotFoundError):
    """A required resource is missing from this repository."""


def _file(path: Path) -> Path:
    if not path.is_file():
        raise ResourceNotFound(f"missing standalone teleop resource: {path}")
    return path


def controller_profile(name: str) -> Path:
    """Resolve a vendored arm controller profile or calibration artifact."""
    return _file(NATIVE / "config" / name)


def controller_resource(profile: Path, relative_path: str) -> Path:
    """Resolve a path declared by a local controller YAML file."""
    return _file((Path(profile).parent / relative_path).resolve())


def display_model_path(name: str = DISPLAY_MODEL) -> Path:
    """Resolve shared display MJCF and arm kinematics URDF resources."""
    return _file(NATIVE / "description/models" / name)


def native_executable(name: str) -> Path:
    """Resolve only this repository's installed native worker or viewer."""
    path = INSTALL / "bin" / name
    if not path.is_file() or not os.access(path, os.X_OK):
        raise ResourceNotFound(f"native teleop executable missing; run bash/build_teleop.sh: {path}")
    return path


def hand_runtime_python() -> Path:
    """Pinned Hand2 interpreter, isolated from the arm's Pinocchio/Eigen ABI."""
    return HAND_ROOT / ".pixi/envs/default/bin/python"


def hand_runtime_launcher() -> Path:
    """Cold-path manifest compiler run by the isolated Hand2 interpreter."""
    return PACKAGE / "scripts/wuji_hand_native_launcher.py"


def hand_retargeting_root() -> Path:
    """Vendored official Hand2 bridge and model resources."""
    return HAND_ROOT / "third_party/wuji_hand_retargeting"
