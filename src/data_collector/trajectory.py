"""Portable, state-only whole-scene snapshots; binary models require exact MuJoCo versions."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import tempfile
from typing import Any
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from interfaces.ros_joint_command import JOINT_NAMES

PHYSICS_HZ = 480
STATE_RATE_HZ = 60
HAND_NAMES = ("l_wrist", "r_wrist")
_DIMENSIONS = ("nq", "nv", "na", "nmocap", "neq", "nbody", "njnt", "ngeom", "ncam", "nmesh", "ntex", "nplugin")


class TrajectoryError(ValueError):
    """A model, mapping or frame cannot satisfy the trajectory contract."""


def _canonical(value: Any) -> str:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False, allow_nan=False)


def _name(model: Any, kind: Any, index: int) -> str | None:
    return mujoco.mj_id2name(model, kind, index)


def _require_model(model: Any) -> None:
    if model.nplugin or getattr(model, "npluginstate", 0):
        raise TrajectoryError("plugin models are unsupported: plugin state is not captured")
    if not np.isclose(model.opt.timestep, 1.0 / PHYSICS_HZ, rtol=0, atol=1e-15):
        raise TrajectoryError("trajectory models must use a 480 Hz physics timestep")


def _fields(model: Any, object_count: int) -> dict[str, dict[str, Any]]:
    def spec(dtype: str, *shape: int) -> dict[str, Any]:
        return {"dtype": dtype, "shape": list(shape)}

    fields = {
        "tick": spec("int64"), "monotonic_ns": spec("int64"), "sim_time": spec("float64"),
        "qpos": spec("float64", model.nq), "qvel": spec("float64", model.nv),
        "robot_qpos": spec("float64", 54), "robot_qvel": spec("float64", 54),
        "hand_contact": spec("bool", 2),
    }
    if model.na:
        fields["act"] = spec("float64", model.na)
    if model.nmocap:
        fields["mocap_pos"] = spec("float64", model.nmocap, 3)
        fields["mocap_quat"] = spec("float64", model.nmocap, 4)
    if model.neq:
        fields["eq_active"] = spec("bool", model.neq)
    if object_count:
        fields["object_pose"] = spec("float64", object_count, 7)
        fields["hand_object"] = spec("bool", 2, object_count)
    return fields


def _subtree_geoms(model: Any, root: int) -> list[int]:
    bodies = {root}
    # Compiled MuJoCo bodies are ordered parent before child.
    for body in range(root + 1, model.nbody):
        if int(model.body_parentid[body]) in bodies:
            bodies.add(body)
    return [geom for geom in range(model.ngeom) if int(model.geom_bodyid[geom]) in bodies]


def _mapping(model: Any, object_names: list[str]) -> dict[str, Any]:
    joint_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in JOINT_NAMES]
    if any(index < 0 or model.jnt_type[index] != mujoco.mjtJoint.mjJNT_HINGE for index in joint_ids):
        raise TrajectoryError("model must expose all 54 canonical robot hinge joints")
    hand_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name) for name in HAND_NAMES]
    if any(index <= 0 for index in hand_ids):
        raise TrajectoryError("model must contain l_wrist and r_wrist hand roots")
    if any(not isinstance(name, str) or not name for name in object_names) or len(set(object_names)) != len(object_names):
        raise TrajectoryError("object names must be unique nonempty strings")
    object_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name) for name in object_names]
    if any(index <= 0 for index in object_ids):
        raise TrajectoryError("every task object must have a named non-world root body")
    hand_geoms = [_subtree_geoms(model, root) for root in hand_ids]
    object_geoms = [_subtree_geoms(model, root) for root in object_ids]
    robot_bodies = {int(model.jnt_bodyid[index]) for index in joint_ids}
    for root in object_ids:
        for robot_body in robot_bodies:
            body = robot_body
            while body:
                if body == root:
                    raise TrajectoryError("task object subtree must not contain robot joints")
                body = int(model.body_parentid[body])
        body = root
        while body:
            if body in robot_bodies or body in hand_ids:
                raise TrajectoryError("task object must not be part of the robot")
            body = int(model.body_parentid[body])
    occupied: set[int] = set()
    for geoms in [*hand_geoms, *object_geoms]:
        if occupied.intersection(geoms):
            raise TrajectoryError("hand and object geometry subtrees must be disjoint")
        occupied.update(geoms)
    return {
        "robot_joint_names": list(JOINT_NAMES),
        "robot_joint_ids": joint_ids,
        "robot_qpos_indices": [int(model.jnt_qposadr[index]) for index in joint_ids],
        "robot_qvel_indices": [int(model.jnt_dofadr[index]) for index in joint_ids],
        "hand_names": list(HAND_NAMES), "hand_body_ids": hand_ids, "hand_geom_ids": hand_geoms,
        "object_names": object_names, "object_body_ids": object_ids, "object_geom_ids": object_geoms,
        "bodies": [
            {"id": index, "name": _name(model, mujoco.mjtObj.mjOBJ_BODY, index),
             "parent_id": int(model.body_parentid[index]), "mocap_id": int(model.body_mocapid[index])}
            for index in range(model.nbody)
        ],
        "joints": [
            {"id": index, "name": _name(model, mujoco.mjtObj.mjOBJ_JOINT, index),
             "type": int(model.jnt_type[index]), "body_id": int(model.jnt_bodyid[index]),
             "qpos_address": int(model.jnt_qposadr[index]), "dof_address": int(model.jnt_dofadr[index])}
            for index in range(model.njnt)
        ],
    }


def _cameras(model: Any) -> dict[str, Any]:
    # Store every compiled camera array, including calibration and initial poses.
    return {
        "names": [_name(model, mujoco.mjtObj.mjOBJ_CAMERA, index) for index in range(model.ncam)],
        "arrays": {key: getattr(model, key).tolist() for key in sorted(dir(model))
                   if key.startswith("cam_") and isinstance(getattr(model, key), np.ndarray)},
        "offwidth": int(model.vis.global_.offwidth), "offheight": int(model.vis.global_.offheight),
        "znear": float(model.vis.map.znear), "zfar": float(model.vis.map.zfar),
    }


def _validate_camera_config(model: Any, document: Any) -> None:
    if document is None:
        return
    if not isinstance(document, dict) or document.get("version") != 1 or not isinstance(document.get("cameras"), dict):
        raise TrajectoryError("invalid camera configuration metadata")
    from cameras.camera import look_at_rotation, rotation_to_mujoco_quat

    for name, config in document["cameras"].items():
        camera = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, name)
        if camera < 0 or not isinstance(config, dict):
            raise TrajectoryError(f"camera configuration is not present in compiled model: {name}")
        parent = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, config["parent"])
        position = np.asarray(config["position"], dtype=np.float64)
        target = np.asarray(config["look_at"], dtype=np.float64)
        if position.shape != (3,) or target.shape != (3,) or not np.all(np.isfinite(position)) or not np.all(np.isfinite(target)):
            raise TrajectoryError(f"invalid camera coordinates: {name}")
        quaternion = np.asarray(rotation_to_mujoco_quat(look_at_rotation(position, target)))
        if (int(model.cam_bodyid[camera]) != parent
                or not np.allclose(model.cam_pos[camera], position, rtol=0, atol=1e-12)
                or min(np.max(np.abs(model.cam_quat[camera] - quaternion)),
                       np.max(np.abs(model.cam_quat[camera] + quaternion))) > 1e-12
                or not np.isclose(model.cam_fovy[camera], document["fovy_deg"], rtol=0, atol=1e-12)):
            raise TrajectoryError(f"camera configuration disagrees with compiled model: {name}")


def _object_names(scene_manifest: dict[str, Any]) -> list[str]:
    objects = scene_manifest.get("objects", [])
    if not isinstance(objects, list) or any(not isinstance(item, dict) or "name" not in item for item in objects):
        raise TrajectoryError("scene_manifest.objects must contain named object mappings")
    return [item["name"] for item in objects]


def _provenance(plant: Any, task_manifest: dict[str, Any]) -> tuple[dict[str, Any], Any]:
    sources: list[dict[str, Any]] = []
    visited: set[Path] = set()

    def add(path: Path, role: str, *, asset_root: Path | None = None, meshdir: str = "", texturedir: str = "") -> None:
        path = path.expanduser().resolve()
        if path in visited or not path.is_file():
            return
        visited.add(path)
        contents = path.read_bytes()
        sources.append({"role": role, "path": str(path), "sha256": hashlib.sha256(contents).hexdigest()})
        if path.suffix.lower() != ".xml":
            return
        try:
            root = ET.fromstring(contents)
        except ET.ParseError:
            return
        asset_root = asset_root or path.parent
        compiler = root.find("compiler")
        if compiler is not None:
            meshdir = compiler.get("meshdir", compiler.get("assetdir", meshdir))
            texturedir = compiler.get("texturedir", compiler.get("assetdir", texturedir))
        for element in root.iter():
            filename = element.get("file")
            if not filename:
                continue
            if element.tag == "include":
                add(path.parent / filename, "include", asset_root=asset_root, meshdir=meshdir, texturedir=texturedir)
            elif element.tag in ("mesh", "texture", "hfield", "skin"):
                directory = meshdir if element.tag == "mesh" else texturedir if element.tag == "texture" else ""
                add(asset_root / directory / filename, element.tag)

    for role in ("model_path", "full_model_path", "scene_model_path", "manifest_path", "scene_manifest_path", "urdf_path", "camera_config_path", "collection_config_path"):
        value = getattr(plant, role, None) or task_manifest.get(role)
        if value is not None:
            add(Path(value), role)
    camera_path = getattr(plant, "camera_config_path", None) or task_manifest.get("camera_config_path")
    camera_config = None
    if camera_path is not None:
        from cameras.camera import load_camera_config
        camera_config, _ = load_camera_config(camera_path)
    return {"sources": sources, "artifact_hash": getattr(plant, "artifact_hash", None)}, camera_config



class TrajectorySource:
    """Capture owned snapshots on the physics thread without changing live data."""

    def __init__(self, plant: Any, task_manifest: dict[str, Any]) -> None:
        self._plant = plant
        self._model = plant.model
        _require_model(self._model)
        if not isinstance(task_manifest, dict):
            raise TrajectoryError("task_manifest must be a JSON mapping")
        task_manifest = json.loads(_canonical(task_manifest))
        scene_manifest = json.loads(_canonical(getattr(plant, "scene_manifest", None) or {}))
        mapping = _mapping(self._model, _object_names(scene_manifest))
        provenance, camera_config = _provenance(plant, task_manifest)
        _validate_camera_config(self._model, camera_config)
        # MJB embeds compiled meshes/textures; source paths are provenance only.
        with tempfile.TemporaryDirectory(prefix="spd-model-") as temporary:
            path = Path(temporary) / "model.mjb"
            mujoco.mj_saveModel(self._model, str(path), None)
            self.model_bytes = path.read_bytes()
        self.metadata = {
            "snapshot_format": "mujoco_mjb", "mujoco_version": mujoco.mj_versionString(),
            "model_sha256": hashlib.sha256(self.model_bytes).hexdigest(),
            "physics_hz": PHYSICS_HZ, "state_rate_hz": STATE_RATE_HZ,
            "dimensions": {name: int(getattr(self._model, name)) for name in _DIMENSIONS},
            "fields": _fields(self._model, len(mapping["object_names"])),
            "task_manifest": task_manifest, "scene_manifest": scene_manifest,
            "cameras": _cameras(self._model), "camera_config": camera_config,
            "provenance": provenance, "object_pose_convention": "world_xyz_wxyz",
            "contact_convention": "solver_active_hand_object_contact_any_physics_step_since_previous_capture",
            **mapping,
        }
        _canonical(self.metadata)
        self._qpos_indices = np.asarray(mapping["robot_qpos_indices"], dtype=np.intp)
        self._qvel_indices = np.asarray(mapping["robot_qvel_indices"], dtype=np.intp)
        self._object_ids = np.asarray(mapping["object_body_ids"], dtype=np.intp)
        self._pose_data = mujoco.MjData(self._model)
        from _spd_native import ContactCollector

        self._contact_collector = ContactCollector(
            self._model, plant.data, mapping["hand_geom_ids"], mapping["object_geom_ids"],
        )

    def reset_contacts(self) -> None:
        self._contact_collector.reset()

    def capture_contact_state(self) -> Any:
        """Copy the unfinished recording interval with native source identity."""
        return self._contact_collector.capture_state()

    def restore_contact_state(self, snapshot: Any) -> None:
        self._contact_collector.restore_state(snapshot)

    def has_hand_object_contact(self) -> bool:
        """Check current contacts, not the previous step or accumulated recording interval."""
        return self._contact_collector.current()

    def observe_contacts(self) -> None:
        """Accumulate solver-active contacts immediately after each recorded mj_step."""
        self._contact_collector.observe()

    def capture(self, tick: int, monotonic_ns: int) -> dict[str, Any]:
        data = self._plant.data
        frame = {
            "tick": np.int64(tick), "monotonic_ns": np.int64(monotonic_ns),
            "sim_time": np.float64(data.time), "qpos": data.qpos.copy(), "qvel": data.qvel.copy(),
            "robot_qpos": data.qpos[self._qpos_indices], "robot_qvel": data.qvel[self._qvel_indices],
            "hand_contact": self._contact_collector.hand_contact(),
        }
        for key in ("act", "mocap_pos", "mocap_quat", "eq_active"):
            if key in self.metadata["fields"]:
                frame[key] = np.array(getattr(data, key), dtype=self.metadata["fields"][key]["dtype"], copy=True)
        if len(self._object_ids):
            # mj_step leaves position-derived arrays at the pre-integration state.
            # Kinematics on separate data recomputes poses from the captured qpos.
            self._pose_data.qpos[:] = frame["qpos"]
            if self._model.nmocap:
                self._pose_data.mocap_pos[:] = frame["mocap_pos"]
                self._pose_data.mocap_quat[:] = frame["mocap_quat"]
            mujoco.mj_kinematics(self._model, self._pose_data)
            frame["object_pose"] = np.concatenate(
                (self._pose_data.xpos[self._object_ids], self._pose_data.xquat[self._object_ids]), axis=1,
            )
            frame["hand_object"] = self._contact_collector.contacts()
        self.reset_contacts()
        return frame


def load_model(model_bytes: bytes, metadata: dict[str, Any], *, expected_model_sha256: str | None = None) -> Any:
    """Validate and load a self-contained snapshot, never consulting source paths."""
    if not isinstance(model_bytes, bytes) or not model_bytes or not isinstance(metadata, dict):
        raise TrajectoryError("model snapshot must contain nonempty bytes and metadata mapping")
    try:
        _canonical(metadata)
        digest = hashlib.sha256(model_bytes).hexdigest()
        if metadata.get("model_sha256") != digest or (expected_model_sha256 is not None and digest != expected_model_sha256):
            raise TrajectoryError("compiled model SHA-256 mismatch")
        if metadata.get("snapshot_format") != "mujoco_mjb":
            raise TrajectoryError("unsupported model snapshot format")
        if metadata.get("mujoco_version") != mujoco.mj_versionString():
            raise TrajectoryError("MJB restoration requires the exact recorded MuJoCo version")
        if metadata.get("physics_hz") != PHYSICS_HZ or metadata.get("state_rate_hz") != STATE_RATE_HZ:
            raise TrajectoryError("trajectory rate metadata mismatch")
        if not isinstance(metadata.get("task_manifest"), dict) or not isinstance(metadata.get("scene_manifest"), dict):
            raise TrajectoryError("task and scene manifests must be JSON mappings")
        with tempfile.TemporaryDirectory(prefix="spd-restore-") as temporary:
            path = Path(temporary) / "model.mjb"
            path.write_bytes(model_bytes)
            model = mujoco.MjModel.from_binary_path(str(path))
        _require_model(model)
        expected = _mapping(model, _object_names(metadata["scene_manifest"]))
        expected.update({
            "dimensions": {name: int(getattr(model, name)) for name in _DIMENSIONS},
            "fields": _fields(model, len(expected["object_names"])), "cameras": _cameras(model),
            "object_pose_convention": "world_xyz_wxyz",
            "contact_convention": "solver_active_hand_object_contact_any_physics_step_since_previous_capture",
        })
        for key, value in expected.items():
            if _canonical(metadata.get(key)) != _canonical(value):
                raise TrajectoryError(f"model metadata mismatch: {key}")
        if not isinstance(metadata.get("provenance"), dict) or "camera_config" not in metadata:
            raise TrajectoryError("missing source provenance or camera configuration metadata")
        _validate_camera_config(model, metadata["camera_config"])
        return model
    except TrajectoryError:
        raise
    except (ValueError, TypeError, KeyError, OSError) as exc:
        raise TrajectoryError(f"invalid model snapshot: {exc}") from exc


def restore_frame(model: Any, data: Any, frame: dict[str, Any], metadata: dict[str, Any]) -> None:
    """Restore physical arrays and forward derived state, without stepping or ctrl replay."""
    fields = metadata["fields"]
    if set(frame) != set(fields):
        raise TrajectoryError("frame fields do not match model metadata")
    for key, spec in fields.items():
        value = np.asarray(frame[key])
        if value.shape != tuple(spec["shape"]) or value.dtype != np.dtype(spec["dtype"]):
            raise TrajectoryError(f"wrong frame dtype or shape: {key}")
        if not np.all(np.isfinite(value)):
            raise TrajectoryError(f"nonfinite frame field: {key}")
    # Reset transient solver, force and control state between independently restored rows.
    mujoco.mj_resetData(model, data)
    data.time = float(frame["sim_time"])
    data.qpos[:] = frame["qpos"]
    data.qvel[:] = frame["qvel"]
    for key in ("act", "mocap_pos", "mocap_quat", "eq_active"):
        if key in fields:
            getattr(data, key)[:] = frame[key]
    # Replay/render restore poses, not integrate forces; policy remains in the MJB.
    # Live/resumed dynamics must use the material-aware native physics entrypoint.
    mujoco.mj_forward(model, data)


__all__ = ["TrajectoryError", "TrajectorySource", "load_model", "restore_frame"]
