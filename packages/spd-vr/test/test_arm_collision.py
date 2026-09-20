import mujoco
import numpy as np
import pytest

from spd_vr.collision_avoidance import ArmCollisionScene
from spd_vr.qp_arm import ArmQPSolver


def collision_fixture(obstacle, *, pieces=1):
    # Seven real hinges per arm. Only the distal sphere is collidable, so the
    # tests isolate a genuine table, torso, self or other-arm closest pair.
    def chain(side, root, self_geom=""):
        bodies = ""
        for index in reversed(range(7)):
            distal = (''.join(f'<geom name="{side}_tip_{piece}" type="sphere" pos="0.1 0 0" size="0.03"/>' for piece in range(pieces))
                      + f'<site name="{side}_wrist" pos="0.1 0 0"/>') if index == 6 else ''
            geometry = self_geom if index == 0 else ''
            axis = "0 1 0" if index == 0 else "0 0 1"
            bodies = f'<body name="{side}_{index}" pos="{root if index == 0 else "0.1 0 0"}"><joint name="{side}_j{index}" axis="{axis}" range="-2 2"/><geom type="sphere" size="0.005" contype="0" conaffinity="0" mass="0.1"/>{geometry}{distal}{bodies}</body>'
        return bodies
    static = {
        "table": '<geom name="table" type="box" pos="0.7 0 0.33" size="0.3 0.3 0.025"/>',
        "torso": '<body name="torso">' + ''.join(f'<geom name="torso_geom_{piece}" type="sphere" pos="0.7 0.075 0.4" size="0.03"/>' for piece in range(pieces)) + '</body>',
    }.get(obstacle, "")
    self_geom = '<geom name="proximal_self" type="sphere" pos="0.7 0.075 0" size="0.03"/>' if obstacle == "self" else ""
    right_root = "0 0.075 0.4" if obstacle == "other_arm" else "0 2 0.4"
    # Movable intended-contact object deliberately overlaps the left tip.
    prop = '<body name="prop" pos="0.7 0 0.4"><freejoint/><geom name="prop_geom" type="sphere" size="0.05"/></body>'
    xml = f'<mujoco><compiler angle="radian"/><worldbody>{static}{chain("l", "0 0 0.4", self_geom)}{chain("r", right_root)}{prop}</worldbody></mujoco>'
    model = mujoco.MjModel.from_xml_string(xml)
    names = [f"{side}_j{i}" for side in ("l", "r") for i in range(7)]
    return model, ArmCollisionScene(model, names)


@pytest.mark.parametrize("obstacle", ["table", "torso", "self", "other_arm"])
def test_real_geometry_produces_separation_rows_and_rejects_penetrating_step(obstacle):
    model, scene = collision_fixture(obstacle)
    initial = np.zeros(14)
    rows, lower = scene.constraints(initial, 0.005)
    assert scene.minimum_distance == pytest.approx(0.015, abs=1e-6)
    toward = initial.copy()
    toward[0 if obstacle == "table" else 6] = 0.05 if obstacle == "table" else 0.4
    assert np.any(rows @ (toward / 0.005) < lower)
    assert not scene.verify_step(initial, toward)
    assert scene.verify_step(initial, -toward)


def test_scene_constraints_prevent_repeated_table_approach():
    model, scene = collision_fixture("table")
    solver = ArmQPSolver(model, site_name="l_wrist", joint_ids=range(7))
    q = np.zeros(14)
    target = np.eye(4)
    target[:3, 3] = [0.7, 0, 0.3]
    for _ in range(150):
        rows, lower = scene.constraints(q, 0.005)
        relevant = np.any(np.abs(rows[:, :7]) > 1e-10, axis=1)
        result = solver.solve(q[:7], target, 0.005, collision_rows=rows[relevant, :7], collision_lower=lower[relevant])
        assert result.success, result.status
        proposed = q.copy()
        proposed[:7] += result.dq * 0.005
        assert scene.verify_step(q, proposed)
        q = proposed
    assert scene.minimum_distance >= scene.margin - 1e-6
    assert result.position_error_m > 0.05  # bounded best effort, not convergence


def test_simultaneous_other_arm_step_is_verified_together():
    _, scene = collision_fixture("other_arm")
    q = np.zeros(14)
    left_only = q.copy()
    left_only[6] = 0.065
    right_only = q.copy()
    right_only[13] = -0.065
    assert scene.verify_step(q, left_only)
    assert scene.verify_step(q, right_only)
    assert not scene.verify_step(q, left_only + right_only)


def test_movable_contact_objects_are_not_treated_as_static_obstacles():
    _, scene = collision_fixture("none")
    q = np.zeros(14)
    proposed = q.copy()
    proposed[6] = 0.2
    assert scene.verify_step(q, proposed)
    assert scene.minimum_distance is None


def test_convex_piece_count_does_not_turn_clear_motion_into_capacity_failure():
    _, scene = collision_fixture("torso", pieces=16)
    initial = np.zeros(14)
    scene.constraints(initial, 0.005)
    away = initial.copy()
    away[6] = -0.04
    assert scene.verify_step(initial, away)
    into = initial.copy()
    into[6] = 0.4
    assert not scene.verify_step(initial, into)
