import numpy as np
import mujoco

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
    assert not bad.success
    np.testing.assert_array_equal(solver.last_q, previous)


def test_workspace_is_persistent_for_repeated_solves(monkeypatch):
    model, data = make_arm()
    import osqp
    setup = osqp.OSQP.setup
    calls = {"setup": 0, "update": 0, "warm_start": 0, "solve": 0}

    def count_setup(self, *args, **kwargs):
        calls["setup"] += 1
        return setup(self, *args, **kwargs)

    monkeypatch.setattr(osqp.OSQP, "setup", count_setup)
    solver = ArmQPSolver(model, data, site_name="wrist", joint_ids=list(range(7)))
    target = np.eye(4)
    solver.solve(np.zeros(7), target, 0.01)
    solver.solve(np.zeros(7), target, 0.01)
    assert calls["setup"] == 1
