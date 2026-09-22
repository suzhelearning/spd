"""Single MuJoCo physics owner for externally supplied 54-joint targets."""
from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
import tempfile
from typing import Any, Sequence

import numpy as np

from description.manifest import ManifestError, ManifestJoint, load_manifest, resolve_home_positions, resolve_model_addresses
from description.model_builder import description_root
from description.model_compiler.artifacts import ArtifactError, verify_artifacts
from interfaces.ros_joint_command import JOINT_NAME_TUPLE, VALID_READY_MASK

PHYSICS_HZ = 480
RENDER_HZ = 60


@dataclass(frozen=True, slots=True)
class PlantStep:
    tick: int
    sim_time_ns: int
    finite: bool


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
        self.hold_mask = VALID_READY_MASK
        self._closed = False
        self.tick = 0
        self._set_home_state()

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
        self._command_qpos = self.model.jnt_qposadr[joint_ids].copy()
        self._command_dof = self.model.jnt_dofadr[joint_ids].copy()
        self._command_actuators = np.asarray([self._actuator_ids[e.actuator] for e in entries])
        self._command_home = np.asarray([self._home[e.index] for e in entries], dtype=np.float64)
        self._command_targets = self._command_home.copy()
        self._command_limits = np.asarray([e.range for e in entries], dtype=np.float64)
        for wire_index, actuator_id in enumerate(self._command_actuators):
            if self.model.actuator_ctrllimited[actuator_id]:
                low, high = self.model.actuator_ctrlrange[actuator_id]
                self._command_limits[wire_index, 0] = max(self._command_limits[wire_index, 0], low)
                self._command_limits[wire_index, 1] = min(self._command_limits[wire_index, 1], high)
        groups = []
        for side, group, bit in (
            ("left", "arm", 1), ("right", "arm", 1),
            ("left", "hand", 4), ("right", "hand", 2),
        ):
            wire = np.asarray([i for i, entry in enumerate(entries) if entry.side == side and entry.group == group])
            slots = [entries[i].index - (27 if side == "right" else 0) - (7 if group == "hand" else 0) for i in wire]
            size = 7 if group == "arm" else 20
            if len(wire) != size or set(slots) != set(range(size)):
                raise ManifestError("invalid joint command target group")
            groups.append((side, group, bit, wire))
        self._command_groups = tuple(groups)

    def _set_home_state(self) -> None:
        # Preserve scene free-joint initial poses; only named robot joints use HOME.
        self.data.qpos[:] = self.model.qpos0
        self.data.qvel[:] = 0.0
        self.data.ctrl[:] = 0.0
        self._command_targets[:] = self._command_home
        self.data.qpos[self._command_qpos] = self._command_home
        self.data.ctrl[self._command_actuators] = self._command_home
        if getattr(self.data, "act", None) is not None:
            self.data.act[:] = 0.0
        self.data.time = 0.0
        self._mujoco.mj_forward(self.model, self.data)

    @property
    def sim_time_ns(self) -> int:
        return int(round(float(self.data.time) * 1_000_000_000.0))

    def joint_command_positions(self) -> np.ndarray:
        """Return simulated joint positions in canonical wire order."""
        return self.data.qpos[self._command_qpos]

    def joint_command_velocities(self) -> np.ndarray:
        """Return simulated velocities by named joint DOFs, not qpos addresses."""
        return self.data.qvel[self._command_dof]

    def joint_command_targets(self) -> np.ndarray:
        """Return the complete retained targets in canonical wire order."""
        return self._command_targets.copy()

    def validate_joint_command(self, snapshot: Any) -> np.ndarray:
        """Validate every ready group before any target or freshness mutation."""
        snapshot.validate()
        values = np.asarray(snapshot.position_rad, dtype=np.float64)
        for side, group, bit, wire in self._command_groups:
            if snapshot.ready_mask & bit and np.any(
                (values[wire] < self._command_limits[wire, 0])
                | (values[wire] > self._command_limits[wire, 1])
            ):
                raise ValueError(f"{side} {group} command is outside the manifest/actuator limits")
        return values

    def set_joint_command_hold(self, hold_mask: int) -> None:
        """Update status only; retained targets remain untouched (physics thread)."""
        self.hold_mask = int(hold_mask)

    def submit_joint_command(self, snapshot: Any, *, hold_mask: int = 0) -> None:
        """Apply validated targets atomically at the physics boundary."""
        values = self.validate_joint_command(snapshot)
        effective_hold = (VALID_READY_MASK ^ snapshot.ready_mask) | hold_mask
        for _side, _group, bit, wire in self._command_groups:
            if not effective_hold & bit:
                self._command_targets[wire] = values[wire]
        self.set_joint_command_hold(effective_hold)

    def physics_tick(self) -> PlantStep:
        if self._closed:
            raise RuntimeError("plant is shut down")
        self.data.ctrl[self._command_actuators] = self._command_targets
        self._mujoco.mj_step(self.model, self.data)
        self.tick += 1
        finite = bool(
            np.all(np.isfinite(self.data.qpos))
            and np.all(np.isfinite(self.data.qvel))
            and np.all(np.isfinite(self.data.ctrl))
            and np.isfinite(self.data.time)
        )
        return PlantStep(self.tick, self.sim_time_ns, finite)

    def close(self) -> None:
        self._closed = True
        if self._scene_temp is not None:
            self._scene_temp.cleanup()
            self._scene_temp = None


__all__ = ["PHYSICS_HZ", "PlantController", "PlantStep", "RENDER_HZ"]
