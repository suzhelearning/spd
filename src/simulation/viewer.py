"""Single MuJoCo physics owner for externally supplied 54-joint targets."""
from __future__ import annotations

import json
from pathlib import Path
import tempfile
from typing import Any, Sequence

import numpy as np

from _spd_native import Physics, PhysicsCheckpoint, PhysicsStep

from description.manifest import ManifestError, ManifestJoint, load_manifest, resolve_home_positions, resolve_model_addresses
from description.model_builder import description_root
from description.model_compiler.artifacts import ArtifactError, verify_artifacts
from interfaces.ros_joint_command import JOINT_NAME_TUPLE, VALID_READY_MASK

PHYSICS_HZ = 480
RENDER_HZ = 60


class PlantController:
    """Own model/data and integrate retained actuator targets at physics ticks.

    Authorization, session changes and freshness are owned by the command
    executor. Holding a group retains its last target; it never writes qpos.
    """

    physics_hz = PHYSICS_HZ
    render_hz = RENDER_HZ

    def __init__(
        self,
        model_path: str | Path | None = None,
        manifest_path: str | Path | None = None,
        *,
        model: Any | None = None,
        data: Any | None = None,
        joints: Sequence[ManifestJoint] | None = None,
        strict_artifacts: bool | None = None,
        urdf_path: str | Path | None = None,
        camera_config_path: str | Path | None = None,
        scene_result: Any | None = None,
        scene_output_dir: str | Path | None = None,
    ) -> None:
        import mujoco

        self._mujoco = mujoco
        production_model = model is None
        verified = None
        self.scene_manifest = scene_result.manifest() if scene_result is not None else None
        self.scene_model_path: Path | None = None
        self.scene_manifest_path: Path | None = None
        self._scene_temp: tempfile.TemporaryDirectory | None = None
        if scene_result is not None and not production_model:
            raise ArtifactError("scene composition requires a verified production model")
        if strict_artifacts is None:
            strict_artifacts = production_model
        if production_model:
            generated = description_root() / "generated"
            urdf_path = Path(urdf_path) if urdf_path is not None else description_root() / "assets" / "tianji_wuji2.urdf"
            if model_path is None:
                model_path = generated / "unified_plant.xml"
            if manifest_path is None:
                manifest_path = generated / "model_manifest.yaml"
            if strict_artifacts:
                verified = verify_artifacts(manifest_path, urdf_path)
                if Path(model_path).resolve() != verified.full_model.resolve():
                    raise ArtifactError("viewer must load manifest unified_plant.xml")
            if scene_result is not None:
                if verified is None:
                    raise ArtifactError("scene composition requires verified model artifacts")
                from spd_envs.model_scene import write_scene_model

                if scene_output_dir is None:
                    self._scene_temp = tempfile.TemporaryDirectory(prefix="spd-scene-")
                    scene_output_dir = self._scene_temp.name
                destination = (
                    Path(scene_output_dir).resolve() / scene_result.scene
                    / scene_result.task / f"seed_{scene_result.seed}"
                )
                self.scene_model_path = write_scene_model(
                    verified.full_model, scene_result, destination / "scene.xml",
                )
                self.scene_manifest_path = destination / "scene_manifest.json"
                self.scene_manifest_path.write_text(json.dumps({
                    **self.scene_manifest,
                    "base_model": str(verified.full_model.resolve()),
                    "base_manifest_sha256": verified.manifest.get("manifest_sha256"),
                    "model": str(self.scene_model_path),
                }, indent=2, sort_keys=True) + "\n", encoding="utf-8")
                model_path = self.scene_model_path
            if camera_config_path is None:
                model = mujoco.MjModel.from_xml_path(str(model_path))
            else:
                from cameras.camera import load_camera_model

                model = load_camera_model(model_path, camera_config_path)
        self.model_path = Path(model_path).resolve() if model_path is not None else None
        self.manifest_path = Path(manifest_path).resolve() if manifest_path is not None else None
        self.urdf_path = Path(urdf_path).resolve() if urdf_path is not None else None
        self.camera_config_path = Path(camera_config_path).resolve() if camera_config_path is not None else None
        self.full_model_path: Path | None = verified.full_model.resolve() if verified is not None else None
        self.model = model
        self.data = mujoco.MjData(model) if data is None else data
        manifest = None
        if joints is not None:
            self.joints = sorted(joints, key=lambda entry: entry.index)
        elif manifest_path is not None and Path(manifest_path).is_file():
            manifest = load_manifest(manifest_path)
            self.joints = resolve_model_addresses(model, manifest, allow_scene_dofs=model.nq > 54)
            self.joints.sort(key=lambda entry: entry.index)
        else:
            raise ManifestError("full plant requires a 54-DoF manifest")
        if len(self.joints) != 54 or [entry.index for entry in self.joints] != list(range(54)):
            raise ManifestError("full plant must expose exactly 54 indexed robot joints")
        self._actuator_ids = {
            entry.actuator: int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, entry.actuator))
            for entry in self.joints
        }
        if any(value < 0 for value in self._actuator_ids.values()):
            raise ManifestError("manifest actuator address resolution failed")
        self._home = np.asarray(
            resolve_home_positions(self.joints, manifest)
            if manifest is not None
            else [(entry.range[0] + entry.range[1]) * 0.5 for entry in self.joints],
            dtype=np.float64,
        )
        self.artifact_hash = (
            str(verified.manifest.get("manifest_sha256", "unknown"))
            if verified is not None else "unverified"
        )
        self._prepare_joint_commands()
        self._physics = Physics(
            self.model, self.data, self._command_qpos, self._command_dof,
            self._command_actuators, self._command_limits, self._command_home,
        )
        self._closed = False

    def _prepare_joint_commands(self) -> None:
        """Resolve the wire contract by names, independently of scene DOFs."""
        by_name = {entry.joint: entry for entry in self.joints}
        if len(by_name) != 54 or set(by_name) != set(JOINT_NAME_TUPLE):
            raise ManifestError("joint command manifest must contain the canonical robot names")
        entries = [by_name[name] for name in JOINT_NAME_TUPLE]
        joint_ids = np.asarray([
            self._mujoco.mj_name2id(self.model, self._mujoco.mjtObj.mjOBJ_JOINT, name)
            for name in JOINT_NAME_TUPLE
        ])
        if np.any(joint_ids < 0) or np.any(
            self.model.jnt_type[joint_ids] != self._mujoco.mjtJoint.mjJNT_HINGE
        ):
            raise ManifestError("joint commands require named hinge joints")
        self._command_joints = joint_ids
        self._command_qpos = self.model.jnt_qposadr[joint_ids].copy()
        self._command_dof = self.model.jnt_dofadr[joint_ids].copy()
        self._command_actuators = np.asarray([self._actuator_ids[e.actuator] for e in entries])
        self._command_home = np.asarray([self._home[e.index] for e in entries], dtype=np.float64)
        self._command_limits = np.asarray([e.range for e in entries], dtype=np.float64)
        for wire_index, actuator_id in enumerate(self._command_actuators):
            if self.model.actuator_ctrllimited[actuator_id]:
                low, high = self.model.actuator_ctrlrange[actuator_id]
                self._command_limits[wire_index, 0] = max(self._command_limits[wire_index, 0], low)
                self._command_limits[wire_index, 1] = min(self._command_limits[wire_index, 1], high)
        for side, group, bit in (
            ("left", "arm", 1), ("right", "arm", 1),
            ("left", "hand", 4), ("right", "hand", 2),
        ):
            wire = np.asarray([i for i, entry in enumerate(entries) if entry.side == side and entry.group == group])
            slots = [entries[i].index - (27 if side == "right" else 0) - (7 if group == "hand" else 0) for i in wire]
            size = 7 if group == "arm" else 20
            if len(wire) != size or set(slots) != set(range(size)):
                raise ManifestError("invalid joint command target group")

    def inherit_robot_state(self, previous: PlantController) -> None:
        """Carry only robot state into a fresh scene, without stepping or authorizing it.

        Call on the physics owner thread while neither plant is being integrated.
        Scene joints, fixture state and the new scene clock retain their initial values.
        """
        if not isinstance(previous, PlantController) or previous is self:
            raise ValueError("robot state requires a different source plant")
        if self.model is previous.model or self.data is previous.data:
            raise ValueError("robot state requires a new model and independent physics data")
        if self._closed or previous._closed:
            raise RuntimeError("cannot inherit robot state from or into a shut down plant")
        if not self._physics.fresh or self.tick != 0 or self.data.time != 0:
            raise ValueError("robot state can only be inherited by a fresh plant")
        if (
            self.artifact_hash in ("", "unknown", "unverified")
            or self.artifact_hash != previous.artifact_hash
        ):
            raise ValueError("robot state requires matching verified base artifacts")

        initial_qpos = self.model.qpos0.copy()
        initial_qpos[self._command_qpos] = self._command_home
        initial_ctrl = np.zeros(self.model.nu, dtype=np.float64)
        initial_ctrl[self._command_actuators] = self._command_home
        if (
            not np.array_equal(self.data.qpos, initial_qpos)
            or np.any(self.data.qvel)
            or not np.array_equal(self.data.ctrl, initial_ctrl)
            or not np.array_equal(self.joint_command_targets(), self._command_home)
            or self.hold_mask != VALID_READY_MASK
        ):
            raise ValueError("destination robot and scene state must still be initial")

        for plant in (previous, self):
            model, ids = plant.model, plant._command_actuators
            gain, bias = model.actuator_gainprm[ids], model.actuator_biasprm[ids]
            gear = model.actuator_gear[ids]
            if (
                len(np.unique(ids)) != len(JOINT_NAME_TUPLE)
                or np.any(model.actuator_trntype[ids] != self._mujoco.mjtTrn.mjTRN_JOINT)
                or not np.array_equal(model.actuator_trnid[ids, 0], plant._command_joints)
                or np.any(model.actuator_dyntype[ids] != self._mujoco.mjtDyn.mjDYN_NONE)
                or np.any(model.actuator_gaintype[ids] != self._mujoco.mjtGain.mjGAIN_FIXED)
                or np.any(model.actuator_biastype[ids] != self._mujoco.mjtBias.mjBIAS_AFFINE)
                or not np.all(np.isfinite(gain))
                or not np.all(np.isfinite(bias))
                or np.any(gain[:, 0] <= 0)
                or np.any(gain[:, 1:])
                or np.any(bias[:, 0])
                or not np.array_equal(bias[:, 1], -gain[:, 0])
                or np.any(bias[:, 2] > 0)
                or np.any(bias[:, 3:])
                or np.any(gear[:, 0] != 1)
                or np.any(gear[:, 1:])
            ):
                raise ValueError("robot state requires direct stateless position actuators")
            state_spec = self._mujoco.mjtState.mjSTATE_INTEGRATION
            state = np.empty(self._mujoco.mj_stateSize(model, state_spec), dtype=np.float64)
            self._mujoco.mj_getState(model, plant.data, state, state_spec)
            if not np.all(np.isfinite(state)) or not np.all(np.isfinite(plant.data.qacc)):
                raise ValueError("robot state contains non-finite physics data")
        for name in (
            "actuator_gainprm", "actuator_biasprm", "actuator_gear",
            "actuator_ctrllimited", "actuator_ctrlrange",
            "actuator_forcelimited", "actuator_forcerange",
        ):
            if not np.array_equal(
                getattr(self.model, name)[self._command_actuators],
                getattr(previous.model, name)[previous._command_actuators],
            ):
                raise ValueError("robot position actuator semantics do not match")
        targets = previous.joint_command_targets()
        if not np.all(np.isfinite(targets)) or any(
            np.any(targets < plant._command_limits[:, 0])
            or np.any(targets > plant._command_limits[:, 1])
            for plant in (previous, self)
        ):
            raise ValueError("retained robot targets are outside the joint/actuator limits")

        self._physics.inherit_robot_state(previous._physics)

    @property
    def tick(self) -> int:
        return self._physics.tick

    @property
    def hold_mask(self) -> int:
        return self._physics.hold_mask

    @hold_mask.setter
    def hold_mask(self, value: int) -> None:
        self._physics.set_hold(value)

    @property
    def sim_time_ns(self) -> int:
        return self._physics.sim_time_ns

    def joint_command_positions(self) -> np.ndarray:
        """Return owned simulated joint positions in canonical wire order."""
        return self._physics.positions()

    def joint_command_start_positions(self) -> np.ndarray:
        """Project measured soft-limit overshoot into the legal servo envelope."""
        return self._physics.start_positions()

    def joint_command_velocities(self) -> np.ndarray:
        """Return owned velocities by named joint DOFs, not qpos addresses."""
        return self._physics.velocities()

    def joint_command_targets(self) -> np.ndarray:
        """Return owned retained targets in canonical wire order."""
        return self._physics.targets()

    def validate_joint_command(self, snapshot: Any) -> np.ndarray:
        """Validate the command and return bounded targets without mutating state."""
        return self._physics.validate_joint_command(snapshot)

    def set_joint_command_hold(self, hold_mask: int) -> None:
        """Update status only; retained targets remain untouched."""
        self._physics.set_hold(hold_mask)

    def submit_joint_command(self, snapshot: Any, *, hold_mask: int = 0) -> None:
        """Saturate finite targets and apply unheld ready groups atomically."""
        self._physics.submit_joint_command(snapshot, hold_mask=hold_mask)

    def capture_checkpoint(self) -> PhysicsCheckpoint:
        """Copy full integration state and retained commands on the physics thread."""
        return self._physics.capture_checkpoint()

    def restore_checkpoint(self, snapshot: PhysicsCheckpoint) -> None:
        """Restore exactly, without a forward pass or physics step."""
        if not isinstance(snapshot, PhysicsCheckpoint):
            raise ValueError("checkpoint belongs to a different plant or model")
        self._physics.restore_checkpoint(snapshot)

    def physics_tick(self) -> PhysicsStep:
        return self._physics.physics_tick()

    def close(self) -> None:
        self._closed = True
        self._physics.close()
        if self._scene_temp is not None:
            self._scene_temp.cleanup()
            self._scene_temp = None


__all__ = ["PHYSICS_HZ", "PlantController", "RENDER_HZ"]
