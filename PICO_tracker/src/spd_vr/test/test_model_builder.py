from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import yaml

from spd_vr.model_builder import build_model


def test_generated_model_exposes_explicit_wrist_targets(tmp_path: Path):
    model_path, manifest_path, _ = build_model(tmp_path)

    root = ET.parse(model_path).getroot()
    body_names = {
        body.attrib["name"] for body in root.iter("body") if "name" in body.attrib
    }
    site_names = {
        site.attrib["name"] for site in root.iter("site") if "name" in site.attrib
    }
    assert {"l_wrist", "r_wrist"} <= body_names
    assert {"l_wrist_target", "r_wrist_target"} <= site_names

    manifest = yaml.safe_load(manifest_path.read_text())
    assert manifest["dof"] == 54
    assert manifest["wrist_targets"] == {
        "left_body": "l_wrist",
        "left_site": "l_wrist_target",
        "right_body": "r_wrist",
        "right_site": "r_wrist_target",
    }
    import mujoco

    model = mujoco.MjModel.from_xml_path(str(model_path))
    assert model.nq == model.nv == model.nu == 54
    for body_name, site_name in (
        ("l_wrist", "l_wrist_target"),
        ("r_wrist", "r_wrist_target"),
    ):
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
        assert body_id >= 0
        assert site_id >= 0
        assert int(model.site_bodyid[site_id]) == body_id


def test_generated_wrist_pose_matches_control_model_at_same_joint_state(
    tmp_path: Path,
):
    """Wrist poses use MuJoCo's wxyz convention, not source xyzw order."""
    import mujoco

    generated_path, _, _ = build_model(tmp_path)
    control_path = (
        Path(__file__).resolve().parents[4]
        / "TJ_arm_control"
        / "models"
        / "marvin_m6_qp_pico_fast.xml"
    )
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
            # The legacy hand-authored arm MJCF stores transforms with
            # fewer decimal places than the generated URDF model.
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
