from dataclasses import asdict
from types import ModuleType, SimpleNamespace
import sys

import numpy as np
import pytest

from spd_vr.interfaces.ros_executor import JointCommandMailbox, RosJointCommandExecutor
from spd_vr.interfaces.ros_joint_command import ARMS_READY, JOINT_NAMES, JointCommandError, JointCommandSnapshot


def _message(sequence: int, session: str = "session-a", stamp_ns: int = 1_000_000_000,
             ready_mask: int = ARMS_READY, positions=None):
    return SimpleNamespace(
        schema_version=1,
        robot_config="tianji_wuji2_v1",
        session_id=session,
        sequence=sequence,
        stamp=SimpleNamespace(sec=stamp_ns // 1_000_000_000, nanosec=stamp_ns % 1_000_000_000),
        ready_mask=ready_mask,
        joint_names=list(JOINT_NAMES),
        position_rad=[0.0] * 54 if positions is None else list(positions),
    )


def test_snapshot_rejects_noncanonical_name_order():
    snapshot = JointCommandSnapshot.from_values(
        session_id="session-a", sequence=1, ready_mask=ARMS_READY, position_rad=[0.0] * 54,
        stamp_ns=1_000_000_000,
    )
    with pytest.raises(JointCommandError):
        snapshot.__class__(
            snapshot.schema_version, snapshot.robot_config, snapshot.session_id,
            snapshot.sequence, snapshot.stamp_ns, snapshot.ready_mask,
            tuple(reversed(snapshot.joint_names)), snapshot.position_rad,
        ).validate()


def test_mailbox_session_conflict_revokes_pending_but_keeps_new_candidate():
    mailbox = JointCommandMailbox()
    assert not mailbox.authorize(True, now_ns=1_000_000_000)
    assert mailbox.receive(_message(1), now_ns=1_000_000_000)
    assert mailbox.take_pending() is None
    assert mailbox.authorize(True, now_ns=1_000_000_000)
    assert mailbox.receive(_message(2), now_ns=1_000_000_000)
    assert not mailbox.receive(_message(1), now_ns=1_000_000_000)
    assert mailbox.receive(_message(1, session="session-b"), now_ns=1_000_000_000)
    assert not mailbox.enabled
    assert mailbox.take_pending() is None
    assert mailbox.latest.session_id == "session-b"
    assert mailbox.authorize(True, now_ns=1_000_000_000)
    assert mailbox.take_pending().session_id == "session-b"


@pytest.fixture
def command_plant():
    mujoco = pytest.importorskip("mujoco")
    from spd_vr.description.manifest import ManifestJoint, resolve_model_addresses
    from spd_vr.simulation.viewer import PlantController

    # Reverse model and actuator ordering, and prepend an unrelated scene free joint.
    bodies = ['<body name="scene" pos="0 0 1"><freejoint name="scene_free"/><geom type="sphere" size=".01"/></body>']
    for name in reversed(JOINT_NAMES):
        bodies.append(f'<body><joint name="{name}" type="hinge" range="-2 2"/>'
                      '<geom type="sphere" size=".01"/></body>')
    actuators = [f'<position name="a_{name}" joint="{name}" kp="1" ctrllimited="true" ctrlrange="-2 2"/>'
                 for name in reversed(JOINT_NAMES)]
    xml = ('<mujoco><compiler angle="radian"/><option gravity="0 0 0" timestep=".002"/>'
           '<worldbody><site name="l_wrist_target"/><site name="r_wrist_target"/>'
           + ''.join(bodies) + '</worldbody><actuator>' + ''.join(actuators) + '</actuator></mujoco>')
    model = mujoco.MjModel.from_xml_string(xml)
    manifest_order = JOINT_NAMES[:7] + JOINT_NAMES[14:34] + JOINT_NAMES[7:14] + JOINT_NAMES[34:]
    joints = []
    for index, name in enumerate(manifest_order):
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        joints.append(ManifestJoint(index=index, side="left" if index < 27 else "right",
                                    group="arm" if index % 27 < 7 else "hand", joint=name,
                                    actuator=f"a_{name}", qpos_address=int(model.jnt_qposadr[joint_id]),
                                    dof_address=int(model.jnt_dofadr[joint_id]), range=(-2.0, 2.0),
                                    velocity_limit=10.0))
    manifest = {"joints": [
        asdict(entry) | {"qpos_address": entry.index, "dof_address": entry.index}
        for entry in joints
    ]}
    joints = resolve_model_addresses(model, manifest, allow_scene_dofs=True)
    plant = PlantController(model=model, joints=list(reversed(joints)),
                            strict_artifacts=False)
    yield plant
    plant.close()


@pytest.fixture
def receiver(command_plant, monkeypatch):
    import spd_vr.interfaces.ros_executor as module

    messages = ModuleType("tianji_spd_interfaces.msg")
    messages.JointCommand = object
    monkeypatch.setitem(sys.modules, "tianji_spd_interfaces.msg", messages)
    monkeypatch.setattr(module, "best_effort_qos", lambda: None)
    clock = SimpleNamespace(utc=1_000_000_000, mono=10_000_000_000)
    monkeypatch.setattr(module.time, "time_ns", lambda: clock.utc)
    monkeypatch.setattr(module.time, "monotonic_ns", lambda: clock.mono)
    node = SimpleNamespace(create_subscription=lambda *args: None)
    return RosJointCommandExecutor(node, command_plant), clock


def test_atomic_rejection_preserves_targets_and_does_not_refresh_timeout(receiver, command_plant):
    executor, clock = receiver
    values = np.full(54, 0.1)
    assert executor.mailbox.receive(_message(1, ready_mask=7, positions=values))
    assert executor.authorize(True)
    executor.apply_pending()
    before = command_plant.joint_command_targets()
    values[:14] = 0.2
    values[-1] = 3.0  # Last ready group fails after arms would previously have been written.
    bad = JointCommandSnapshot.from_values(session_id="session-a", sequence=2, ready_mask=7,
                                           position_rad=values, stamp_ns=clock.utc)
    with pytest.raises(ValueError):
        command_plant.submit_joint_command(bad)
    np.testing.assert_array_equal(command_plant.joint_command_targets(), before)
    clock.utc += 90_000_000
    clock.mono += 90_000_000
    assert not executor.mailbox.receive(_message(2, stamp_ns=clock.utc, ready_mask=7, positions=values))
    clock.utc += 11_000_000
    clock.mono += 11_000_000
    executor.apply_pending()
    assert executor.hold_mask == 7
    np.testing.assert_array_equal(command_plant.joint_command_targets(), before)


def test_command_expiring_between_callback_and_tick_is_not_applied(receiver, command_plant):
    executor, clock = receiver
    assert executor.mailbox.receive(_message(1, positions=np.full(54, 0.1)))
    assert executor.authorize(True)
    # UTC may advance while the supplied simulation scheduling clock stays fixed.
    clock.utc += 100_000_001
    assert executor.apply_pending() is None
    np.testing.assert_array_equal(command_plant.joint_command_targets(), np.zeros(54))
    assert not executor.authorize(True)


def test_timeout_latches_only_stale_group_until_explicit_enable(receiver, command_plant):
    executor, clock = receiver
    assert executor.mailbox.receive(_message(1, ready_mask=7, positions=np.full(54, .1)))
    assert executor.authorize(True)
    executor.apply_pending()
    clock.utc += 90_000_000
    clock.mono += 90_000_000
    assert executor.mailbox.receive(_message(2, stamp_ns=clock.utc, ready_mask=3, positions=np.full(54, .12)))
    applied = executor.apply_pending()
    assert applied.position_rad[14:34] == pytest.approx([.1] * 20)
    clock.utc += 11_000_000
    clock.mono += 11_000_000
    assert executor.mailbox.receive(_message(3, stamp_ns=clock.utc, ready_mask=7, positions=np.full(54, .14)))
    applied = executor.apply_pending()
    assert applied.hold_mask == 4
    assert applied.position_rad[:14] == pytest.approx([.14] * 14)
    assert applied.position_rad[14:34] == pytest.approx([.1] * 20)
    assert executor.authorize(True)
    applied = executor.apply_pending()
    assert applied.hold_mask == 0
    assert applied.position_rad == pytest.approx([.14] * 54)


def test_enable_delta_uses_retained_target_and_clear_does_not_reset_physics(receiver, command_plant):
    executor, clock = receiver
    assert executor.mailbox.receive(_message(1, positions=np.full(54, .16)))
    assert not executor.authorize(True)
    assert executor.mailbox.receive(_message(2, positions=np.full(54, .1)))
    assert executor.authorize(True)
    executor.apply_pending()
    command_plant.physics_tick()
    positions = command_plant.data.qpos.copy()
    targets = command_plant.joint_command_targets()
    executor.clear()
    np.testing.assert_array_equal(command_plant.data.qpos, positions)
    np.testing.assert_array_equal(command_plant.joint_command_targets(), targets)
    # .24 is within .15 of the retained .1 target, but not the near-zero qpos.
    assert executor.mailbox.receive(_message(3, positions=np.full(54, .24)))
    assert executor.authorize(True)


def test_name_mapping_with_scene_free_joint_preserves_scene_and_integrates(command_plant):
    values = np.linspace(-.1, .1, 54)
    snapshot = JointCommandSnapshot.from_values(session_id="session-a", sequence=1, ready_mask=7,
                                                position_rad=values)
    scene_pose = command_plant.data.qpos[:7].copy()
    command_plant.submit_joint_command(snapshot)
    np.testing.assert_array_equal(command_plant.joint_command_targets(), values)
    np.testing.assert_array_equal(command_plant.joint_command_positions(), np.zeros(54))
    step = command_plant.physics_tick()
    np.testing.assert_array_equal(command_plant.data.qpos[:7], scene_pose)
    for name, value in zip(JOINT_NAMES, values):
        assert command_plant.data.actuator(f"a_{name}").ctrl[0] == pytest.approx(value)
        assert command_plant.data.joint(name).qpos[0] == pytest.approx(
            command_plant.joint_command_positions()[JOINT_NAMES.index(name)])
    assert step.sim_time_ns == 2_000_000
    assert np.any(command_plant.joint_command_positions() != 0)
