from __future__ import annotations

from pathlib import Path
import textwrap

import numpy as np
import pytest

from spd_vr.model_compiler.urdf_model import aggregate_fixed_point_masses, load_urdf


ROOT = Path(__file__).resolve().parents[4]
AUTHORITATIVE_URDF = ROOT / "assets" / "tianji_wuji2" / "tianji_wuji2.urdf"


def test_authoritative_cardinality_order_and_point_mass_aggregation():
    model = load_urdf(AUTHORITATIVE_URDF)
    assert model.root == "Link_Base"
    assert len(model.links) == 80
    assert len(model.joints) == 79
    assert len(model.revolute_joints) == 54
    assert len(model.arm_joint_names) == 14
    assert len(model.hand_joint_names("left")) == 20
    assert len(model.hand_joint_names("right")) == 20
    assert model.joints[0].child == "Link_Stand"
    assert model.manifest["joints"]["Joint_Stand"] == 2

    tcp = model.link("TCP_Link_L")
    assert tcp.inertial is not None
    assert tcp.inertial.mass == pytest.approx(0.05)
    aggregated = aggregate_fixed_point_masses(model)
    link7 = aggregated.link("Link7_L")
    assert link7.inertial is not None
    assert link7.inertial.mass == pytest.approx(model.link("Link7_L").inertial.mass + 0.05)
    assert aggregated.link("TCP_Link_L").inertial is None

    parent_com = np.asarray(model.link("Link7_L").inertial.com)
    child_com = np.zeros(3)
    translation, rotation = model.fixed_transform("Link7_L", "TCP_Link_L")
    translation = np.asarray(translation)
    rotation = np.asarray(rotation)
    old_mass = model.link("Link7_L").inertial.mass
    expected_com = (old_mass * parent_com + 0.05 * (translation + rotation @ child_com)) / (
        old_mass + 0.05
    )
    assert np.allclose(link7.inertial.com, expected_com)
    old_tensor = np.asarray(model.link("Link7_L").inertial.inertia)
    parent_shift = parent_com - expected_com
    child_position = translation
    child_shift = child_position - expected_com
    expected_tensor = old_tensor + old_mass * (
        (parent_shift @ parent_shift) * np.eye(3) - np.outer(parent_shift, parent_shift)
    ) + 0.05 * ((child_shift @ child_shift) * np.eye(3) - np.outer(child_shift, child_shift))
    assert np.allclose(link7.inertial.inertia, expected_tensor)


def test_malformed_validation_rejects_each_invalid_variant(tmp_path: Path):
    base = """
    <robot name="fixture">
      <link name="base"><inertial><mass value="1"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial><visual><geometry><mesh filename="base.stl"/></geometry></visual></link>
      <link name="child"><inertial><mass value="1"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial><visual><geometry><mesh filename="child.stl"/></geometry></visual></link>
      <joint name="j" type="revolute"><parent link="base"/><child link="child"/><axis xyz="0 0 1"/><limit lower="-1" upper="1"/></joint>
    </robot>
    """
    variants = (
        base.replace("<robot", "<not-robot", 1),
        base.replace('<link name="child">', '<link name="base">', 1),
        base.replace('filename="child.stl"', 'filename="missing.stl"'),
        base.replace('lower="-1" upper="1"', 'lower="nan" upper="1"'),
        base.replace('ixx="1" ixy="0"', 'ixx="-1" ixy="0"'),
        base.replace('<link name="child">', '<link name="orphan"><inertial><mass value="1"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial></link>\n<link name="child">', 1),
    )
    for index, xml in enumerate(variants):
        path = tmp_path / f"bad-{index}.urdf"
        path.write_text(textwrap.dedent(xml))
        (tmp_path / "base.stl").write_bytes(b"solid base")
        (tmp_path / "child.stl").write_bytes(b"solid child")
        with pytest.raises(ValueError):
            load_urdf(path)
