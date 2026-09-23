"""Hand-only drawing geometry, independent of live simulation state."""
from __future__ import annotations

import copy
from typing import Any

import numpy as np

from interfaces.ros_joint_command import JOINT_NAME_TUPLE


class HandGhost:
    """Cache render templates and run only kinematics on private MjData.

    This object has no renderer or GL context. Construct it lazily for a visible
    viewer, and discard it with that viewer's model/data pair.
    """

    # MjvScene.geoms is an immutable tuple of mutable MjvGeom views. Copy the
    # complete rendering descriptor, not geom_dataid: mesh rendering IDs encode
    # both the mesh and whether its convex hull is requested.
    _SCALARS = (
        "type", "dataid", "objtype", "objid", "category", "matid", "texid",
        "texuniform", "texcoord", "segid", "emission", "specular", "shininess",
        "reflectance", "label", "camdist", "modelrbound", "transparent",
    )
    _ARRAYS = ("size", "pos", "mat", "rgba", "texrepeat")

    def __init__(self, model: Any, data: Any) -> None:
        import mujoco

        self.model = model
        self.data = data
        self._mujoco = mujoco
        joint_ids = np.asarray([
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            for name in JOINT_NAME_TUPLE
        ])
        if np.any(joint_ids < 0) or np.any(
            model.jnt_type[joint_ids] != mujoco.mjtJoint.mjJNT_HINGE
        ):
            raise ValueError("hand ghosts require the canonical 54 named hinge joints")
        self._qpos = model.jnt_qposadr[joint_ids].copy()
        roots = {
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
            for name in ("l_wrist", "r_wrist")
        }
        if -1 in roots:
            raise ValueError("hand ghosts require l_wrist and r_wrist bodies")
        hand_bodies = np.zeros(model.nbody, dtype=bool)
        # MuJoCo stores parents before children. Start strictly at the wrists:
        # mounts, arms and task objects are never part of this visual overlay.
        for body_id in range(1, model.nbody):
            hand_bodies[body_id] = (
                body_id in roots or hand_bodies[model.body_parentid[body_id]]
            )
        selected = set(np.flatnonzero(
            hand_bodies[model.geom_bodyid] & (model.geom_group == 1)
        ))
        self._scratch = mujoco.MjData(model)
        self._scratch.qpos[:] = data.qpos
        self._scratch.mocap_pos[:] = data.mocap_pos
        self._scratch.mocap_quat[:] = data.mocap_quat
        mujoco.mj_kinematics(model, self._scratch)
        option = mujoco.MjvOption()
        option.flags[:] = 0
        option.geomgroup[:] = 0
        option.geomgroup[1] = 1
        option.sitegroup[:] = 0
        scene = mujoco.MjvScene(model, maxgeom=model.ngeom)
        mujoco.mjv_addGeoms(
            model, self._scratch, option, mujoco.MjvPerturb(),
            mujoco.mjtCatBit.mjCAT_ALL, scene,
        )
        templates = []
        for geom in scene.geoms[:scene.ngeom]:
            if geom.objtype != mujoco.mjtObj.mjOBJ_GEOM or geom.objid not in selected:
                continue
            template = copy.copy(geom)
            template.rgba[:] = (0.05, 0.65, 1.0, 0.35)
            template.matid = -1
            template.texid = -1
            template.texcoord = 0
            template.emission = 0.2
            template.reflectance = 0.0
            template.category = mujoco.mjtCatBit.mjCAT_DECOR
            template.segid = -1
            template.transparent = 1
            templates.append(template)
        self._templates = tuple(templates)
        self._installed_scene: Any | None = None

    def draw(self, scene: Any, position_rad: np.ndarray, *, start_index: int = 0) -> None:
        """Draw into a ghost-only scene or append after refreshed physical geometry."""
        if start_index < 0 or start_index + len(self._templates) > scene.maxgeom:
            raise RuntimeError("viewer scene cannot hold the hand ghost geometry")
        scratch = self._scratch
        scratch.qpos[:] = self.data.qpos
        scratch.qpos[self._qpos] = position_rad
        scratch.mocap_pos[:] = self.data.mocap_pos
        scratch.mocap_quat[:] = self.data.mocap_quat
        self._mujoco.mj_kinematics(self.model, scratch)
        install = start_index != 0 or self._installed_scene is not scene
        for index, template in enumerate(self._templates):
            geom = scene.geoms[start_index + index]
            if install:
                for name in self._SCALARS:
                    setattr(geom, name, getattr(template, name))
                for name in self._ARRAYS:
                    getattr(geom, name)[:] = getattr(template, name)
            geom.pos[:] = scratch.geom_xpos[template.objid]
            geom.mat[:] = scratch.geom_xmat[template.objid].reshape(3, 3)
        scene.ngeom = start_index + len(self._templates)
        self._installed_scene = scene
