"""Scene-geometry arm avoidance, not a continuous-time collision certificate.

Only observed arm joints and their rigid descendants are represented. Articulated
fingers and movable props are excluded: the arm feedback contract cannot observe
those configurations. Mesh distances use MuJoCo's collision (convex) geometry.
"""
from __future__ import annotations

import math
from typing import Iterable

import mujoco
import numpy as np


class ArmCollisionScene:
    margin = 0.005
    influence = 0.08
    max_constraints = 96

    def __init__(self, model: mujoco.MjModel, joint_names: Iterable[str]) -> None:
        self.model = model
        self.data = mujoco.MjData(model)
        ids = np.array([mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in joint_names])
        if ids.shape != (14,) or np.any(ids < 0) or np.any(model.jnt_type[ids] != mujoco.mjtJoint.mjJNT_HINGE):
            raise ValueError("collision scene must contain the fourteen observed arm hinges")
        self.qpos_indices = model.jnt_qposadr[ids].copy()
        self.dof_indices = model.jnt_dofadr[ids].copy()
        arm_ids = set(ids.tolist())
        # Classify descendants once. Unknown moving branches (fingers/props) must
        # not masquerade as known static collision geometry.
        affected = np.zeros(model.nbody, dtype=bool)
        unknown = np.zeros(model.nbody, dtype=bool)
        for body in range(1, model.nbody):
            parent = model.body_parentid[body]
            affected[body] = affected[parent]
            unknown[body] = unknown[parent] or bool(model.body_mocapid[body] >= 0)
            for joint in range(model.body_jntadr[body], model.body_jntadr[body] + model.body_jntnum[body]):
                affected[body] |= joint in arm_ids
                unknown[body] |= joint not in arm_ids
        excluded = set(int(value) for value in model.exclude_signature)
        pairs = []
        groups = {}
        pair_groups = []
        for first in range(model.ngeom):
            b1 = int(model.geom_bodyid[first])
            if unknown[b1]:
                continue
            for second in range(first + 1, model.ngeom):
                b2 = int(model.geom_bodyid[second])
                if unknown[b2] or not (affected[b1] or affected[b2]):
                    continue
                if not ((model.geom_contype[first] & model.geom_conaffinity[second]) or (model.geom_contype[second] & model.geom_conaffinity[first])):
                    continue
                w1, w2 = int(model.body_weldid[b1]), int(model.body_weldid[b2])
                if w1 == w2:
                    continue
                # MuJoCo filters parent-child welded groups, in addition to
                # explicit compiled body exclusions.
                if w1 != 0 and w2 != 0 and (int(model.body_weldid[model.body_parentid[w1]]) == w2 or int(model.body_weldid[model.body_parentid[w2]]) == w1):
                    continue
                if (min(b1, b2) << 16) + max(b1, b2) in excluded:
                    continue
                pairs.append((first, second))
                key = (min(w1, w2), max(w1, w2))
                pair_groups.append(groups.setdefault(key, len(groups)))
        self.pairs = np.asarray(pairs, dtype=int).reshape(-1, 2)
        self._pair_groups = np.asarray(pair_groups, dtype=int)
        self._radii = model.geom_rbound[self.pairs[:, 0]] + model.geom_rbound[self.pairs[:, 1]]
        self._planes = np.any(model.geom_type[self.pairs] == mujoco.mjtGeom.mjGEOM_PLANE, axis=1)
        self._delta = np.empty((len(pairs), 3))
        self._squared = np.empty(len(pairs))
        self._distances = np.full(len(groups), self.influence)
        self._baseline = np.empty(len(groups))
        self._nearest_pair = np.full(len(groups), -1, dtype=int)
        self._nearest_points = np.empty((len(groups), 6))
        self._fromto = np.empty(6)
        self._jac1 = np.zeros((3, model.nv))
        self._jac2 = np.zeros((3, model.nv))
        self.rows = np.zeros((self.max_constraints, 14))
        self.lower = np.zeros(self.max_constraints)
        self.minimum_distance: float | None = None
        self.detail = ""

    def set_state(self, q: np.ndarray) -> None:
        self.data.qpos[self.qpos_indices] = q
        mujoco.mj_kinematics(self.model, self.data)
        mujoco.mj_comPos(self.model, self.data)

    def _candidates(self, threshold: float) -> np.ndarray:
        np.subtract(self.data.geom_xpos[self.pairs[:, 1]], self.data.geom_xpos[self.pairs[:, 0]], out=self._delta)
        np.einsum("ij,ij->i", self._delta, self._delta, out=self._squared)
        return np.flatnonzero(self._planes | (self._squared <= (self._radii + threshold) ** 2))

    def distances(self, q: np.ndarray) -> np.ndarray:
        self.set_state(q)
        self._distances.fill(self.influence)
        self._nearest_pair.fill(-1)
        for index in self._candidates(self.influence):
            first, second = self.pairs[index]
            distance = mujoco.mj_geomDistance(self.model, self.data, int(first), int(second), self.influence, self._fromto)
            group = self._pair_groups[index]
            if distance < self._distances[group]:
                self._distances[group] = distance
                self._nearest_pair[group] = index
                self._nearest_points[group] = self._fromto
        nearest = float(np.min(self._distances)) if len(self._distances) else self.influence
        self.minimum_distance = nearest if nearest < self.influence else None
        return self._distances

    def constraints(self, q: np.ndarray, dt: float) -> tuple[np.ndarray, np.ndarray]:
        # A decomposed rigid body is one union, not hundreds of independent
        # obstacles. Linearize its nearest piece; verify the union along the
        # full proposed segment to catch changes of the active closest piece.
        distances = self.distances(q)
        count = 0
        for group in np.flatnonzero(self._nearest_pair >= 0):
            first, second = (int(value) for value in self.pairs[self._nearest_pair[group]])
            distance = distances[group]
            self._fromto[:] = self._nearest_points[group]
            if count == self.max_constraints:
                raise ValueError("collision constraint capacity exceeded")
            direction = self._fromto[3:] - self._fromto[:3]
            if abs(distance) > 1e-9:
                direction = direction / distance
            else:
                direction = self.data.geom_xpos[second] - self.data.geom_xpos[first]
                length = float(np.linalg.norm(direction))
                if length < 1e-9:
                    raise ValueError("collision normal is undefined")
                direction = direction / length
            mujoco.mj_jac(self.model, self.data, self._jac1, None, self._fromto[:3], int(self.model.geom_bodyid[first]))
            mujoco.mj_jac(self.model, self.data, self._jac2, None, self._fromto[3:], int(self.model.geom_bodyid[second]))
            self.rows[count] = direction @ (self._jac2[:, self.dof_indices] - self._jac1[:, self.dof_indices])
            # Slow down over the influence shell, escape existing penetration.
            self.lower[count] = (self.margin - distance) / max(0.1, dt)
            count += 1
        return self.rows[:count], self.lower[:count]

    def verify_step(self, start: np.ndarray, proposed: np.ndarray) -> bool:
        """Check both arms together along a sampled joint-space segment.

        Existing overlap may escape but cannot worsen. Samples <=0.01rad apart
        catch nonlinear linearization errors; this is not swept-volume CCD.
        """
        self._baseline[:] = np.minimum(self.distances(start), self.margin)
        steps = max(1, int(math.ceil(float(np.max(np.abs(proposed - start))) / 0.01)))
        if steps > 128:
            self.detail = "collision verification segment exceeds bounded range"
            return False
        delta = proposed - start
        for sample in range(1, steps + 1):
            distances = self.distances(start + (sample / steps) * delta)
            if np.any(~np.isfinite(distances)) or np.any(distances < self._baseline - 1e-7):
                self.detail = "proposed dual-arm segment reduces protected scene separation"
                return False
        self.detail = ""
        return True
