from pathlib import Path
import xml.etree.ElementTree as ET

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
    assert manifest["wrist_targets"] == {
        "left_body": "l_wrist",
        "left_site": "l_wrist_target",
        "right_body": "r_wrist",
        "right_site": "r_wrist_target",
    }

    import mujoco

    model = mujoco.MjModel.from_xml_path(str(model_path))
    assert model.nu == 54
    for body_name, site_name in (
        ("l_wrist", "l_wrist_target"),
        ("r_wrist", "r_wrist_target"),
    ):
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
        assert body_id >= 0
        assert site_id >= 0
        assert int(model.site_bodyid[site_id]) == body_id
