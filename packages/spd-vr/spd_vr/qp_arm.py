"""Single-arm MuJoCo velocity IK with persistent hierarchical QP workspaces."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Iterable

import numpy as np
import osqp
from scipy import sparse
from scipy.spatial.transform import Rotation

try:
    import mujoco
except ImportError:  # pragma: no cover - package dependency is required at runtime
    mujoco = None  # type: ignore[assignment]

from .alignment import _pose_matrix


@dataclass(frozen=True, slots=True)
class ArmSolveResult:
    dq: np.ndarray
    success: bool
    status: str
    position_error_m: float = math.inf
    orientation_error_rad: float = math.inf
    state: str = "tracking"

    def __post_init__(self) -> None:
        value = np.array(self.dq, dtype=float, copy=True).reshape(-1)
        value.setflags(write=False)
        object.__setattr__(self, "dq", value)

    @property
    def solved(self) -> bool:
        return self.success

    @property
    def failure(self) -> bool:
        return not self.success


@dataclass(frozen=True, slots=True)
class QPConfig:
    position_weight: float = 1.0
    orientation_weight: float = 0.5
    lambda_damp: float = 1.0e-4
    max_linear_speed: float = 2.0
    max_angular_speed: float = 4.0
    max_position_error_m: float = 2.0
    max_orientation_error_rad: float = math.pi
    acceleration_limit: float = 8.0
    task_gain: float = 8.0
    position_tolerance_m: float = 0.002
    orientation_tolerance_rad: float = 0.02
    joint_center_gain: float = 0.3
    joint_center_weight: float = 0.05
    elbow_clearance_m: float = 0.08
    elbow_gain: float = 2.0
    elbow_weight: float = 10.0


def _rotation_log(rotation: np.ndarray) -> np.ndarray:
    # Quaternion-based extraction remains well-conditioned at a half turn.
    # acos(trace(R)) / sin(angle) can inflate a valid residual beyond pi and
    # falsely reject the target as unreachable.
    return Rotation.from_matrix(rotation).as_rotvec()


class ArmQPSolver:
    """Solve a 7-variable bounded Cartesian velocity QP for one arm."""

    def __init__(
        self,
        model: Any,
        data: Any | None = None,
        *,
        side: str | None = None,
        site_name: str | None = None,
        joint_ids: Iterable[int] | None = None,
        qpos_indices: Iterable[int] | None = None,
        dof_indices: Iterable[int] | None = None,
        position_limits: Iterable[Iterable[float]] | None = None,
        velocity_limits: float | Iterable[float] = 2.0,
        home: Iterable[float] | None = None,
        config: QPConfig | None = None,
    ) -> None:
        if mujoco is None:
            raise ImportError("mujoco is required for ArmQPSolver")
        self.model = model
        self.data = data if data is not None else mujoco.MjData(model)
        self.side = side
        self.config = config or QPConfig()
        if site_name is None:
            site_name = f"{side[0]}_wrist_target" if side in {"left", "right"} else None
        if site_name is None:
            if model.nsite != 1:
                raise ValueError("site_name is required when the model has multiple sites")
            self.site_id = 0
        else:
            self.site_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name))
            if self.site_id < 0:
                raise ValueError(f"unknown site: {site_name}")

        if joint_ids is None:
            joint_ids = self._discover_joint_ids(side)
        self.joint_ids = np.asarray(tuple(int(index) for index in joint_ids), dtype=int)
        if self.joint_ids.shape != (7,):
            raise ValueError("ArmQPSolver requires exactly seven hinge joints")
        if np.any(model.jnt_type[self.joint_ids] != mujoco.mjtJoint.mjJNT_HINGE):
            raise ValueError("ArmQPSolver joint_ids must reference hinge joints")
        self.qpos_indices = np.asarray(
            tuple(int(value) for value in qpos_indices)
            if qpos_indices is not None
            else tuple(int(model.jnt_qposadr[index]) for index in self.joint_ids),
            dtype=int,
        )
        self.dof_indices = np.asarray(
            tuple(int(value) for value in dof_indices)
            if dof_indices is not None
            else tuple(int(model.jnt_dofadr[index]) for index in self.joint_ids),
            dtype=int,
        )
        if self.qpos_indices.shape != (7,) or self.dof_indices.shape != (7,):
            raise ValueError("qpos_indices and dof_indices must each have seven values")
        if position_limits is None:
            limits = np.asarray(model.jnt_range[self.joint_ids], dtype=float)
            limited = np.asarray(model.jnt_limited[self.joint_ids], dtype=bool)
            limits = np.where(limited[:, None], limits, np.array((-math.inf, math.inf)))
        else:
            limits = np.asarray(tuple(position_limits), dtype=float)
        if limits.shape != (7, 2) or np.any(limits[:, 1] <= limits[:, 0]):
            raise ValueError("position_limits must be seven increasing pairs")
        self.position_limits = limits
        if np.isscalar(velocity_limits):
            self.velocity_limits = np.full(7, float(velocity_limits), dtype=float)
        else:
            self.velocity_limits = np.asarray(tuple(velocity_limits), dtype=float)
        if self.velocity_limits.shape != (7,) or not np.all(np.isfinite(self.velocity_limits)) or np.any(self.velocity_limits <= 0.0):
            raise ValueError("velocity_limits must contain seven positive finite values")
        self._p_rows = np.asarray([row for col in range(7) for row in range(col + 1)], dtype=int)
        self._p_cols = np.asarray([col for col in range(7) for row in range(col + 1)], dtype=int)
        self.home = np.asarray(tuple(home), dtype=float) if home is not None else np.asarray(self.data.qpos[self.qpos_indices], dtype=float).copy()
        if self.home.shape != (7,) or not np.all(np.isfinite(self.home)):
            raise ValueError("home must contain seven finite values")

        if not math.isfinite(self.config.acceleration_limit) or self.config.acceleration_limit <= 0:
            raise ValueError("acceleration_limit must be positive and finite")
        self._jacp = np.zeros((3, model.nv))
        self._jacr = np.zeros((3, model.nv))
        self._jacobian = np.zeros((6, 7))
        self._posture_p = np.eye(7)[self._p_rows, self._p_cols]
        # Robot world is FLU. The fourth arm joint body is the anatomical
        # elbow (Link4_L/R in production), the first is the shoulder.
        self._elbow_body = int(model.jnt_bodyid[self.joint_ids[3]])
        self._shoulder_body = int(model.jnt_bodyid[self.joint_ids[0]])
        self._elbow_sign = 1.0 if side == "left" else (-1.0 if side == "right" else 0.0)
        self._elbow_jac = np.zeros((3, model.nv))
        self._shoulder_jac = np.zeros((3, model.nv))
        self._posture_hessian = np.eye(7)
        self._bounded_joints = np.all(np.isfinite(self.position_limits), axis=1)
        self._joint_midpoint = np.zeros(7)
        self._joint_half_range = np.ones(7)
        self._joint_midpoint[self._bounded_joints] = np.mean(self.position_limits[self._bounded_joints], axis=1)
        self._joint_half_range[self._bounded_joints] = np.diff(self.position_limits[self._bounded_joints], axis=1).ravel() / 2
        self._weights = np.array((self.config.position_weight,) * 3 + (self.config.orientation_weight,) * 3)
        self._row_count = 7 + 96 + 6
        self._a = np.zeros((self._row_count, 7), order="F")
        self._a[:7] = np.eye(7)
        self._p_values = np.zeros(28, dtype=float)
        self._q_values = np.zeros(7, dtype=float)
        self._lower = np.full(self._row_count, -np.inf, dtype=float)
        self._upper = np.full(self._row_count, np.inf, dtype=float)
        structure = sparse.csc_matrix(np.ones_like(self._a))
        structure.data[:] = self._a.ravel(order="F")
        self._workspace = osqp.OSQP()
        self._posture_workspace = osqp.OSQP()
        # Each objective retains its own scaling, adaptive rho and dual state.
        # Alternating task Hessians with identity posture Hessians in one OSQP
        # workspace contaminates the next task solve's ADMM conditioning.
        for workspace in (self._workspace, self._posture_workspace):
            initial_p = sparse.csc_matrix((self._posture_p.copy(), (self._p_rows, self._p_cols)), shape=(7, 7))
            workspace.setup(
                P=initial_p, q=self._q_values, A=structure,
                l=self._lower, u=self._upper, verbose=False, polishing=False,
                eps_abs=1.0e-8, eps_rel=1.0e-8, max_iter=4000,
                warm_starting=True,
            )
        self._last_dq = np.zeros(7, dtype=float)
        self._last_q: np.ndarray | None = None

    def _discover_joint_ids(self, side: str | None) -> tuple[int, ...]:
        ids: list[int] = []
        for index in range(int(self.model.njnt)):
            if self.model.jnt_type[index] != mujoco.mjtJoint.mjJNT_HINGE:
                continue
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, index) or ""
            lowered = name.lower()
            if side == "left" and not (lowered.startswith(("l_", "left")) or "_left" in lowered):
                continue
            if side == "right" and not (lowered.startswith(("r_", "right")) or "_right" in lowered):
                continue
            ids.append(index)
        if len(ids) != 7 and side is not None:
            raise ValueError(f"authoritative manifest must bind exactly seven {side} joints")
        return tuple(ids)

    @property
    def workspace(self) -> Any:
        return self._workspace

    @property
    def last_q(self) -> np.ndarray | None:
        return None if self._last_q is None else self._last_q.copy()

    def _failure(self, status: str, position_error: float = math.inf, orientation_error: float = math.inf) -> ArmSolveResult:
        return ArmSolveResult(np.zeros(7), False, status, position_error, orientation_error, "invalid")

    def _blocked(self, status: str, position_error: float, orientation_error: float) -> ArmSolveResult:
        # A bounded hold is a valid coordinate target, not a calibration fault.
        self._last_dq.fill(0.0)
        return ArmSolveResult(np.zeros(7), True, status, position_error, orientation_error, "blocked")

    def solve(
        self, q: Iterable[float], target_pose: Any, dt: float, *,
        collision_rows: np.ndarray | None = None,
        collision_lower: np.ndarray | None = None,
    ) -> ArmSolveResult:
        try:
            q_value = np.asarray(q, dtype=float)
            if q_value.shape != (7,) or not np.all(np.isfinite(q_value)):
                return self._failure("invalid q")
            if not math.isfinite(float(dt)) or float(dt) <= 0.0:
                return self._failure("invalid dt")
            target = _pose_matrix(target_pose)
            self.data.qpos[self.qpos_indices] = q_value
            mujoco.mj_kinematics(self.model, self.data)
            mujoco.mj_comPos(self.model, self.data)
            current_position = self.data.site_xpos[self.site_id]
            current_rotation = self.data.site_xmat[self.site_id].reshape(3, 3)
            position_error_vector = target[:3, 3] - current_position
            orientation_error_vector = _rotation_log(target[:3, :3] @ current_rotation.T)
            position_error = float(np.linalg.norm(position_error_vector))
            orientation_error = min(math.pi, float(np.linalg.norm(orientation_error_vector)))
            if position_error > self.config.max_position_error_m or orientation_error > self.config.max_orientation_error_rad:
                self._last_q = q_value.copy()
                return self._blocked("target beyond configured tracking envelope", position_error, orientation_error)
            # Fixed feedback gain avoids target/velocity chatter as dt varies.
            v_des = self.config.task_gain * np.concatenate((position_error_vector, orientation_error_vector))
            for part, limit in ((slice(0, 3), self.config.max_linear_speed), (slice(3, 6), self.config.max_angular_speed)):
                norm = float(np.linalg.norm(v_des[part]))
                if norm > limit:
                    v_des[part] *= limit / norm
            mujoco.mj_jacSite(self.model, self.data, self._jacp, self._jacr, self.site_id)
            self._jacobian[:3] = self._jacp[:, self.dof_indices]
            self._jacobian[3:] = self._jacr[:, self.dof_indices]
            jacobian = self._jacobian
            lower = np.maximum(-self.velocity_limits, (self.position_limits[:, 0] - q_value) / dt)
            upper = np.minimum(self.velocity_limits, (self.position_limits[:, 1] - q_value) / dt)
            # Leave one command interval of braking room instead of using a
            # continuous stopping distance that can overshoot at discrete ticks.
            acceleration_step = self.config.acceleration_limit * dt
            lower = np.maximum(lower, -(np.sqrt(acceleration_step**2 + 2 * self.config.acceleration_limit * np.maximum(0, q_value - self.position_limits[:, 0])) - acceleration_step))
            upper = np.minimum(upper, np.sqrt(acceleration_step**2 + 2 * self.config.acceleration_limit * np.maximum(0, self.position_limits[:, 1] - q_value)) - acceleration_step)
            lower = np.maximum(lower, self._last_dq - self.config.acceleration_limit * dt)
            upper = np.minimum(upper, self._last_dq + self.config.acceleration_limit * dt)
            if not np.all(np.isfinite(lower)) or not np.all(np.isfinite(upper)):
                return self._failure("invalid bounds", position_error, orientation_error)
            if np.any(lower > upper):
                return self._blocked("joint/acceleration bounds conflict", position_error, orientation_error)
            self._lower.fill(-np.inf)
            self._upper.fill(np.inf)
            self._lower[:7], self._upper[:7] = lower, upper
            self._a[7:] = 0
            if collision_rows is not None:
                rows = np.asarray(collision_rows, dtype=float)
                limits = np.asarray(collision_lower, dtype=float)
                if rows.ndim != 2 or rows.shape[1] != 7 or len(rows) > 96 or limits.shape != (len(rows),) or not np.all(np.isfinite(rows)) or not np.all(np.isfinite(limits)):
                    return self._failure("invalid collision constraints", position_error, orientation_error)
                self._a[7:7 + len(rows)] = rows
                self._lower[7:7 + len(rows)] = limits
            hessian = jacobian.T @ (self._weights[:, None] * jacobian)
            hessian.flat[::8] += self.config.lambda_damp
            self._p_values[:] = hessian[self._p_rows, self._p_cols]
            self._q_values[:] = -(jacobian.T @ (self._weights * v_des))
            self._workspace.update(Px=self._p_values, Ax=self._a.ravel(order="F"), q=self._q_values, l=self._lower, u=self._upper)
            self._workspace.warm_start(x=self._last_dq)
            result = self._workspace.solve(raise_error=False)
            status = str(result.info.status).lower()
            if "primal infeasible" in status:
                return self._blocked("constraints prevent requested motion", position_error, orientation_error)
            if status not in {"solved", "solved inaccurate"}:
                return self._failure(status, position_error, orientation_error)
            dq = np.asarray(result.x, dtype=float)
            if not self._feasible(dq):
                return self._failure("invalid primary solution", position_error, orientation_error)
            primary = dq.copy()
            # Preserve the primary twist using independent, unit-length task
            # rows. Raw Cartesian rows become nearly dependent at singular
            # poses; tiny inequality bands make ADMM falsely infeasible there.
            # Exact orthonormal equalities avoid both conditioning problems.
            _, singular_values, task_basis = np.linalg.svd(jacobian, full_matrices=False)
            rank = int(np.count_nonzero(singular_values > 1e-8))
            self._a[-6:] = 0.0
            self._a[self._row_count - 6:self._row_count - 6 + rank] = task_basis[:rank]
            achieved = task_basis[:rank] @ primary
            self._lower[-6:] = -np.inf
            self._upper[-6:] = np.inf
            self._lower[self._row_count - 6:self._row_count - 6 + rank] = achieved
            self._upper[self._row_count - 6:self._row_count - 6 + rank] = achieved
            self._posture_objective(q_value)
            self._posture_workspace.update(Px=self._p_values, Ax=self._a.ravel(order="F"), q=self._q_values, l=self._lower, u=self._upper)
            self._posture_workspace.warm_start(x=primary)
            secondary = self._posture_workspace.solve(raise_error=False)
            secondary_status = str(secondary.info.status).lower()
            if secondary_status not in {"solved", "solved inaccurate"}:
                return self._failure("posture QP: " + secondary_status, position_error, orientation_error)
            dq = np.asarray(secondary.x, dtype=float)
            if not self._feasible(dq):
                return self._failure("invalid posture solution", position_error, orientation_error)
            self._last_dq[:] = dq
            self._last_q = q_value + dq * float(dt)
            converged = position_error <= self.config.position_tolerance_m and orientation_error <= self.config.orientation_tolerance_rad
            state = "converged" if converged else ("blocked" if np.linalg.norm(dq) < 1e-5 else "tracking")
            return ArmSolveResult(dq, True, "wrist target reached" if converged else ("best effort: target constrained or unreachable" if state == "blocked" else "tracking best-effort wrist target"), position_error, orientation_error, state)
        except (TypeError, ValueError, FloatingPointError, np.linalg.LinAlgError) as exc:
            return self._failure(str(exc))

    def _posture_objective(self, q: np.ndarray) -> None:
        # Continuity plus normalized midrange bias keeps away from joint
        # extremes without mistaking an arbitrary HOME vector for anatomy.
        center_velocity = np.clip(
            self.config.joint_center_gain * (self._joint_midpoint - q) / self._joint_half_range,
            -self.config.joint_center_gain, self.config.joint_center_gain,
        )
        center_velocity[~self._bounded_joints] = 0.0
        self._posture_hessian.fill(0.0)
        self._posture_hessian.flat[::8] = 1 + self.config.joint_center_weight
        self._q_values[:] = -self._last_dq - self.config.joint_center_weight * center_velocity
        if self._elbow_sign:
            mujoco.mj_jacBody(self.model, self.data, self._elbow_jac, None, self._elbow_body)
            mujoco.mj_jacBody(self.model, self.data, self._shoulder_jac, None, self._shoulder_body)
            lateral_jacobian = self._elbow_sign * (self._elbow_jac[1, self.dof_indices] - self._shoulder_jac[1, self.dof_indices])
            clearance = self._elbow_sign * (self.data.xpos[self._elbow_body, 1] - self.data.xpos[self._shoulder_body, 1])
            # Only request outward motion when inside the preferred clearance;
            # do not pull an already comfortably wide elbow back toward torso.
            if clearance < self.config.elbow_clearance_m:
                desired = self.config.elbow_gain * (self.config.elbow_clearance_m - clearance)
                self._posture_hessian += self.config.elbow_weight * np.outer(lateral_jacobian, lateral_jacobian)
                self._q_values -= self.config.elbow_weight * desired * lateral_jacobian
        self._p_values[:] = self._posture_hessian[self._p_rows, self._p_cols]

    def _feasible(self, dq: np.ndarray) -> bool:
        if dq.shape != (7,) or not np.all(np.isfinite(dq)):
            return False
        values = self._a @ dq
        return bool(np.all(values >= self._lower - 2e-6) and np.all(values <= self._upper + 2e-6))

    def reset(self) -> None:
        self._last_dq.fill(0.0)
        self._last_q = None
        self._workspace.warm_start(x=self._last_dq, y=np.zeros(self._row_count, dtype=float))
        self._posture_workspace.warm_start(x=self._last_dq, y=np.zeros(self._row_count, dtype=float))


__all__ = ["ArmQPSolver", "ArmSolveResult", "QPConfig"]
