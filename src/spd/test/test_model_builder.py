from pathlib import Path

import numpy as np

from spd_vr.description.model_builder import build_model




def test_generated_wrist_pose_matches_control_model_at_same_joint_state(
    tmp_path: Path,
):
    """Wrist poses use MuJoCo's wxyz convention, not source xyzw order."""
    import mujoco

    generated_path, _, _ = build_model(tmp_path)
    control_path = Path(__file__).parent / "fixtures" / "legacy_arm_kinematics.xml"
    generated = mujoco.MjModel.from_xml_path(str(generated_path))
    control = mujoco.MjModel.from_xml_path(str(control_path))
    generated_data = mujoco.MjData(generated)
    control_data = mujoco.MjData(control)

    # Copy one non-home arm state by joint name into both models. The hand
    # roots have no joints, so these are the same physical arm configuration.
    for index, joint_name in enumerate(
        ("Joint1_L", "Joint2_L", "Joint3_L", "Joint4_L", "Joint5_L", "Joint6_L", "Joint7_L")
    ):
        control_id = mujoco.mj_name2id(control, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        generated_id = mujoco.mj_name2id(
            generated, mujoco.mjtObj.mjOBJ_JOINT, joint_name
        )
        value = 0.03 * (index + 1)
        control_data.qpos[control.jnt_qposadr[control_id]] = value
        generated_data.qpos[generated.jnt_qposadr[generated_id]] = value
    for index, joint_name in enumerate(
        ("Joint1_R", "Joint2_R", "Joint3_R", "Joint4_R", "Joint5_R", "Joint6_R", "Joint7_R")
    ):
        control_id = mujoco.mj_name2id(control, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        generated_id = mujoco.mj_name2id(
            generated, mujoco.mjtObj.mjOBJ_JOINT, joint_name
        )
        value = -0.02 * (index + 1)
        control_data.qpos[control.jnt_qposadr[control_id]] = value
        generated_data.qpos[generated.jnt_qposadr[generated_id]] = value
    mujoco.mj_forward(control, control_data)
    mujoco.mj_forward(generated, generated_data)

    for body_name in ("l_wrist", "r_wrist"):
        control_id = mujoco.mj_name2id(control, mujoco.mjtObj.mjOBJ_BODY, body_name)
        generated_id = mujoco.mj_name2id(
            generated, mujoco.mjtObj.mjOBJ_BODY, body_name
        )
        np.testing.assert_allclose(
            generated_data.xpos[generated_id],
            control_data.xpos[control_id],
            # The retained legacy baseline stores transforms with fewer decimal
            # places than the generated URDF model.
            atol=1e-6,
        )
        # Quaternion signs are equivalent; compare the represented rotation.
        assert abs(
            float(
                np.dot(
                    generated_data.xquat[generated_id],
                    control_data.xquat[control_id],
                )
            )
        ) > 1.0 - 1e-8
