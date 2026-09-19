import numpy as np
import mujoco
import pytest
from scipy.spatial.transform import Rotation

from spd_vr.qp_arm import ArmQPSolver


def make_arm():
    xml = """
    <mujoco>
      <option gravity="0 0 0"/>
      <worldbody><body name="base">
        <body name="link0"><joint name="joint0" type="hinge" axis="0 0 1" range="-1 1"/><geom type="capsule" fromto="0 0 0 0.15 0 0" size="0.03"/>
          <body name="link1" pos="0.15 0 0"><joint name="joint1" type="hinge" axis="0 1 0" range="-1 1"/><geom type="capsule" fromto="0 0 0 0.15 0 0" size="0.03"/>
            <body name="link2" pos="0.15 0 0"><joint name="joint2" type="hinge" axis="0 1 0" range="-1 1"/><geom type="capsule" fromto="0 0 0 0.15 0 0" size="0.03"/>
              <body name="link3" pos="0.15 0 0"><joint name="joint3" type="hinge" axis="1 0 0" range="-1 1"/><geom type="capsule" fromto="0 0 0 0.15 0 0" size="0.03"/>
                <body name="link4" pos="0.15 0 0"><joint name="joint4" type="hinge" axis="0 1 0" range="-1 1"/><geom type="capsule" fromto="0 0 0 0.15 0 0" size="0.03"/>
                  <body name="link5" pos="0.15 0 0"><joint name="joint5" type="hinge" axis="1 0 0" range="-1 1"/><geom type="capsule" fromto="0 0 0 0.15 0 0" size="0.03"/>
                    <body name="link6" pos="0.15 0 0"><joint name="joint6" type="hinge" axis="0 1 0" range="-1 1"/><geom type="capsule" fromto="0 0 0 0.15 0 0" size="0.03"/><site name="wrist" pos="0.15 0 0" size="0.01"/></body>
                  </body>
                </body>
              </body>
            </body>
          </body>
        </body>
      </body></worldbody>
    </mujoco>
    """
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data


def test_solver_moves_toward_target_and_respects_box_bounds():
    model, data = make_arm()
    solver = ArmQPSolver(model, data, site_name="wrist", joint_ids=list(range(7)), velocity_limits=0.2)
    q = np.zeros(7)
    target = np.array(data.site_xpos[0], dtype=float)
    target[2] += 0.03
    result = solver.solve(q, np.block([[np.eye(3), target[:, None]], [np.zeros((1, 3)), np.ones((1, 1))]]), 0.01)

    assert result.success
    assert np.all(np.isfinite(result.dq))
    assert np.all(result.dq <= 0.2 + 1e-7)
    assert np.all(result.dq >= -0.2 - 1e-7)


def test_invalid_target_fails_without_mutating_last_q():
    model, data = make_arm()
    solver = ArmQPSolver(model, data, site_name="wrist", joint_ids=list(range(7)))
    q = np.zeros(7)
    target = np.eye(4)
    good = solver.solve(q, target, 0.01)
    previous = solver.last_q.copy()
    bad = solver.solve(q + 0.1, np.full((4, 4), np.nan), 0.01)
    assert good.success
    np.testing.assert_allclose(solver.last_q, q + good.dq * 0.01)
    assert not bad.success
    np.testing.assert_array_equal(solver.last_q, previous)


def test_velocity_is_continuous_through_target_reversal_and_hold_reset():
    model, data = make_arm()
    solver = ArmQPSolver(model, data, site_name="wrist", joint_ids=range(7), position_limits=[(-2.0, 2.0)] * 7)
    target = np.eye(4)
    target[:3, 3] = data.site_xpos[0]
    q = np.zeros(7)
    previous = np.zeros(7)
    dt = 0.005
    for step in range(120):
        target[2, 3] = 0.08 if step < 60 else -0.08
        result = solver.solve(q, target, dt)
        assert result.success, result.status
        assert np.max(np.abs(result.dq - previous)) <= solver.config.acceleration_limit * dt + 2e-6
        q += result.dq * dt
        previous = result.dq
    solver.reset()
    restarted = solver.solve(q, target, dt)
    assert restarted.success
    assert np.max(np.abs(restarted.dq)) <= solver.config.acceleration_limit * dt + 2e-6


def test_side_selection_does_not_fallback_to_other_side():
    model, data = make_arm()
    with pytest.raises(ValueError, match="authoritative"):
        ArmQPSolver(model, data, side="right", site_name="wrist")


@pytest.mark.parametrize(
    "axis, angle",
    [
        ((1.0, 2.0, 3.0), np.pi - 2.0e-7),
        ((2.0e-6, 0.6, -0.8), np.pi),
        ((1.0, 2.0, 3.0), np.pi),
    ],
)
def test_half_turn_orientation_is_not_falsely_unreachable(axis, angle):
    model, data = make_arm()
    solver = ArmQPSolver(model, data, site_name="wrist", joint_ids=range(7))
    axis = np.asarray(axis) / np.linalg.norm(axis)
    target = np.eye(4)
    target[:3, 3] = data.site_xpos[0]
    target[:3, :3] = Rotation.from_rotvec(axis * angle).as_matrix()

    result = solver.solve(np.zeros(7), target, 0.005)

    assert result.success, result.status
    assert result.orientation_error_rad == pytest.approx(angle, abs=1.0e-12)


def test_reachable_pose_converges_without_posture_bias_overriding_wrist():
    model, data = make_arm()
    # The fixture's XML ranges are degrees; select a reachable small pose.
    desired_q = np.array([0.008, -0.012, 0.009, 0.004, -0.006, 0.003, 0.008])
    data.qpos[:] = desired_q
    mujoco.mj_forward(model, data)
    target = np.eye(4)
    target[:3, 3] = data.site_xpos[0]
    target[:3, :3] = data.site_xmat[0].reshape(3, 3)
    solver = ArmQPSolver(model, data, site_name="wrist", joint_ids=range(7), home=np.zeros(7))
    q = np.zeros(7)
    for _ in range(350):
        result = solver.solve(q, target, 0.005)
        assert result.success, result.status
        q += result.dq * 0.005
    assert result.position_error_m < 0.0005
    assert result.orientation_error_rad < 0.002
    assert result.state == "converged"


def test_unreachable_is_bounded_best_effort_not_invalid_input():
    model, data = make_arm()
    solver = ArmQPSolver(model, data, site_name="wrist", joint_ids=range(7))
    target = np.eye(4)
    target[:3, 3] = [100.0, 0.0, 0.0]
    result = solver.solve(np.zeros(7), target, 0.005)
    assert result.success
    assert result.state == "blocked"
    assert result.position_error_m > 90
    np.testing.assert_array_equal(result.dq, np.zeros(7))
    invalid = solver.solve(np.zeros(7), np.full((4, 4), np.nan), 0.005)
    assert not invalid.success
    assert invalid.state == "invalid"


@pytest.mark.parametrize("side, sign", [("left", 1), ("right", -1)])
def test_redundant_elbow_moves_outward_without_moving_the_wrist(side, sign):
    # A bent arm with root/wrist roll redundancy: its wrist lies on the root
    # roll axis while the elbow is 20cm below it. Swiveling moves the elbow
    # laterally without changing the reachable wrist pose.
    positions = ["0 0 0", "0 0 0", "0.15 0 -0.1", "0.15 0 -0.1",
                 "0.15 0 0.1", "0.15 0 0.1", "0 0 0"]
    axes = ["1 0 0", "0 1 0", "0 0 1", "0 1 0", "0 0 1", "0 1 0", "1 0 0"]
    chain = '<site name="wrist"/>'
    for index in reversed(range(7)):
        chain = f'<body name="link{index}" pos="{positions[index]}"><joint axis="{axes[index]}" range="-2 2"/><geom type="sphere" size="0.02" mass="0.1"/>{chain}</body>'
    model = mujoco.MjModel.from_xml_string(f'<mujoco><compiler angle="radian"/><worldbody>{chain}</worldbody></mujoco>')
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    target = np.eye(4)
    target[:3, 3] = data.site_xpos[0]
    solver = ArmQPSolver(model, data, side=side, site_name="wrist", joint_ids=range(7))
    elbow = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "link3")
    initial_clearance = sign * data.xpos[elbow, 1]
    q = np.zeros(7)
    for _ in range(200):
        result = solver.solve(q, target, 0.005)
        assert result.success, result.status
        q += result.dq * 0.005
    data.qpos[:] = q
    mujoco.mj_forward(model, data)
    assert sign * data.xpos[elbow, 1] > initial_clearance + 0.04
    np.testing.assert_allclose(data.site_xpos[0], target[:3, 3], atol=0.001)
    assert np.linalg.norm(Rotation.from_matrix(data.site_xmat[0].reshape(3, 3)).as_rotvec()) < 0.001
