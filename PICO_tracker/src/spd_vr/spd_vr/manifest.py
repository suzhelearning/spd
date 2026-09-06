"""Strict name/address validation for the unified 54-DoF MuJoCo plant."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any

import yaml


class ManifestError(ValueError):
    """Raised when a model and manifest do not describe the same plant."""


DEFAULT_ARM_HOME_RAD = {
    "left": (0.9599310886, -1.1344640138, -1.2217304764, -1.0471975512, 1.0471975512, 0.0, 0.0),
    "right": (-0.9599310886, -1.1344640138, 1.2217304764, -1.0471975512, -1.0471975512, 0.0, 0.0),
}


@dataclass(frozen=True)
class ManifestJoint:
    index: int
    side: str
    group: str
    joint: str
    actuator: str
    qpos_address: int
    dof_address: int
    range: tuple[float, float]
    velocity_limit: float | None


def load_manifest(path: str | Path) -> dict[str, Any]:
    """Load and structurally validate a Task7 ``model_manifest.yaml``."""
    path = Path(path)
    if not path.is_file():
        raise FileNotFoundError(path)
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ManifestError("manifest root must be a mapping")
    if document.get("version") != 1 or document.get("dof") != 54:
        raise ManifestError("manifest version/dof must be 1/54")
    joints = document.get("joints")
    if not isinstance(joints, list) or len(joints) != 54:
        raise ManifestError("manifest must contain exactly 54 joints")
    joint_order = document.get("joint_order")
    actuator_order = document.get("actuator_order")
    if joint_order != [entry.get("joint") for entry in joints]:
        raise ManifestError("joint_order does not match joints entries")
    if actuator_order != [entry.get("actuator") for entry in joints]:
        raise ManifestError("actuator_order does not match joints entries")
    if len(set(joint_order)) != 54 or len(set(actuator_order)) != 54:
        raise ManifestError("manifest joint and actuator names must be unique")
    for expected, entry in enumerate(joints):
        if not isinstance(entry, dict) or entry.get("index") != expected:
            raise ManifestError(f"manifest index mismatch at {expected}")
        if entry.get("side") not in {"left", "right"} or entry.get("group") not in {"arm", "hand"}:
            raise ManifestError(f"invalid side/group at {expected}")
        if not isinstance(entry.get("range"), list) or len(entry["range"]) != 2:
            raise ManifestError(f"invalid range at {expected}")
        if not float(entry["range"][1]) > float(entry["range"][0]):
            raise ManifestError(f"invalid range at {expected}")
    arm_order = document.get("arm_joint_order")
    expected_arm_order = [entry["joint"] for entry in joints if entry.get("group") == "arm"]
    if arm_order != expected_arm_order or len(arm_order) != 14 or len(set(arm_order)) != 14:
        raise ManifestError("manifest arm_joint_order must exactly match the 14 arm joints")
    arm_home = document.get("arm_home_rad", DEFAULT_ARM_HOME_RAD)
    if not isinstance(arm_home, dict) or set(arm_home) != {"left", "right"}:
        raise ManifestError("manifest arm_home_rad must contain left/right")
    for side in ("left", "right"):
        values = arm_home[side]
        entries = [entry for entry in joints if entry.get("side") == side and entry.get("group") == "arm"]
        if not isinstance(values, (list, tuple)) or len(values) != 7:
            raise ManifestError(f"manifest {side} arm home must contain seven values")
        if any(isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)) for value in values):
            raise ManifestError(f"manifest {side} arm home must be finite")
        if any(not float(entry["range"][0]) <= float(value) <= float(entry["range"][1]) for entry, value in zip(entries, values)):
            raise ManifestError(f"manifest {side} arm home exceeds a joint range")
    wrist = document.get("wrist_targets")
    if wrist != {
        "left_body": "l_wrist",
        "left_site": "l_wrist_target",
        "right_body": "r_wrist",
        "right_site": "r_wrist_target",
    }:
        raise ManifestError("manifest wrist targets are incomplete")
    outputs = document.get("outputs")
    if not isinstance(outputs, dict) or not {"unified_plant.xml", "arm_ik.xml", "collision_manifest.yaml", "actuator_calibration.yaml"} <= set(outputs):
        raise ManifestError("manifest output hashes are incomplete")
    return document


def arm_home_for_side(manifest: dict[str, Any], side: str) -> tuple[float, ...]:
    if side not in {"left", "right"}:
        raise ValueError(f"unknown arm side: {side}")
    return tuple(float(value) for value in manifest.get("arm_home_rad", DEFAULT_ARM_HOME_RAD)[side])


def resolve_home_positions(joints: list[ManifestJoint], manifest: dict[str, Any]) -> tuple[float, ...]:
    arm_by_joint: dict[str, float] = {}
    for side in ("left", "right"):
        names = [entry["joint"] for entry in manifest["joints"] if entry.get("side") == side and entry.get("group") == "arm"]
        arm_by_joint.update(zip(names, arm_home_for_side(manifest, side)))
    home = [0.0] * len(joints)
    for entry in joints:
        home[entry.index] = arm_by_joint.get(entry.joint, (entry.range[0] + entry.range[1]) * 0.5)
    return tuple(home)


def resolve_model_addresses(
    model: Any,
    manifest: dict[str, Any],
    *,
    allow_scene_dofs: bool = False,
) -> list[ManifestJoint]:
    """Resolve robot addresses by name; optionally allow free scene bodies."""
    try:
        import mujoco
    except ImportError as exc:  # pragma: no cover
        raise ImportError("mujoco is required for model manifest validation") from exc
    if model.nu != 54 or model.nq < 54 or model.nv < 54:
        raise ManifestError(f"model must have at least nq/nv=54 and nu=54, got {model.nq}/{model.nv}/{model.nu}")
    if not allow_scene_dofs and (model.nq != 54 or model.nv != 54):
        raise ManifestError(f"model must have nq=nv=54, got {model.nq}/{model.nv}")
    entries = manifest["joints"]
    model_joint_names: list[str] = []
    model_actuator_names: list[str] = []
    for joint_id in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
        if name is not None:
            model_joint_names.append(name)
    for actuator_id in range(model.nu):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_id)
        if name is not None:
            model_actuator_names.append(name)
    expected_joints = [entry["joint"] for entry in entries]
    expected_actuators = [entry["actuator"] for entry in entries]
    if allow_scene_dofs:
        model_joint_ids = {
            name: joint_id
            for joint_id in range(model.njnt)
            if (name := mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)) is not None
        }
        extras = set(model_joint_ids) - set(expected_joints)
        for name in extras:
            joint_id = model_joint_ids[name]
            if not name.endswith("_free") or model.jnt_type[joint_id] != mujoco.mjtJoint.mjJNT_FREE:
                raise ManifestError(f"unexpected non-scene joint: {name}")
        if set(model_joint_names) - extras != set(expected_joints):
            raise ManifestError("model robot joint names differ from manifest")
    elif set(model_joint_names) != set(expected_joints):
        raise ManifestError("model joint names differ from manifest")
    if set(model_actuator_names) != set(expected_actuators):
        raise ManifestError("model actuator names differ from manifest")

    resolved: list[ManifestJoint] = []
    for expected, entry in enumerate(entries):
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, entry["joint"])
        actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, entry["actuator"])
        if joint_id < 0 or actuator_id < 0:
            raise ManifestError(f"missing manifest name at {expected}")
        if model.jnt_type[joint_id] != mujoco.mjtJoint.mjJNT_HINGE:
            raise ManifestError(f"manifest joint is not hinge: {entry['joint']}")
        if int(model.actuator_trnid[actuator_id, 0]) != joint_id:
            raise ManifestError(f"actuator does not target manifest joint: {entry['actuator']}")
        qpos = int(model.jnt_qposadr[joint_id])
        dof = int(model.jnt_dofadr[joint_id])
        if qpos != int(entry["qpos_address"]) or dof != int(entry["dof_address"]):
            raise ManifestError(
                f"address mismatch for {entry['joint']}: "
                f"model={qpos}/{dof}, manifest={entry['qpos_address']}/{entry['dof_address']}"
            )
        resolved.append(ManifestJoint(
            index=expected,
            side=entry["side"],
            group=entry["group"],
            joint=entry["joint"],
            actuator=entry["actuator"],
            qpos_address=qpos,
            dof_address=dof,
            range=(float(entry["range"][0]), float(entry["range"][1])),
            velocity_limit=(float(entry["velocity_limit"]) if entry.get("velocity_limit") is not None else None),
        ))
    return resolved


def validate_model_manifest(
    model_path: str | Path,
    manifest_path: str | Path,
    *,
    allow_scene_dofs: bool = False,
) -> list[ManifestJoint]:
    try:
        import mujoco
    except ImportError as exc:  # pragma: no cover
        raise ImportError("mujoco is required for model manifest validation") from exc
    manifest = load_manifest(manifest_path)
    model = mujoco.MjModel.from_xml_path(str(model_path))
    return resolve_model_addresses(model, manifest, allow_scene_dofs=allow_scene_dofs)


__all__ = [
    "DEFAULT_ARM_HOME_RAD",
    "ManifestError",
    "ManifestJoint",
    "arm_home_for_side",
    "load_manifest",
    "resolve_home_positions",
    "resolve_model_addresses",
    "validate_model_manifest",
]
