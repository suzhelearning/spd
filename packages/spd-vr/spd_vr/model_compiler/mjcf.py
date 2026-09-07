"""Render validated :class:`UrdfModel` data as deterministic MuJoCo XML."""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Mapping, Sequence

import numpy as np

from .urdf_model import UrdfJoint, UrdfLink, UrdfModel


# Wuji Hand 2 beta2's official MuJoCo position-servo calibration.  Both hands
# use the same gains; only the ``l_``/``r_`` joint prefix differs.  Keep these
# values exact instead of replacing them with one generic hand gain.
_WUJI2_HAND_GAINS: dict[str, tuple[float, float]] = {
    "thumb_cmc_flex": (0.40844710645048043, 0.020882010257063675),
    "thumb_cmc_abd": (0.6858601063643346, 0.030610939996373314),
    "thumb_mcp": (0.2391112196099482, 0.010181475050560962),
    "thumb_ip": (0.20736128319711747, 0.00909698844675045),
    "index_finger_mcp_flex": (0.37352218155860073, 0.01882274330029718),
    "index_finger_mcp_abd": (0.45592448909027794, 0.019798167597016643),
    "index_finger_pip": (0.24368366649522863, 0.010477031953162727),
    "index_finger_dip": (0.18026971340925335, 0.008240212147903584),
    "middle_finger_mcp_flex": (0.3687093483646485, 0.01848622487024593),
    "middle_finger_mcp_abd": (0.4164253443634641, 0.018032947229953678),
    "middle_finger_pip": (0.22218607502059182, 0.009592200014076666),
    "middle_finger_dip": (0.19427606072023446, 0.009152994605972402),
    "ring_finger_mcp_flex": (0.35718151495111794, 0.018376606800780005),
    "ring_finger_mcp_abd": (0.42977315313086895, 0.01867700966212433),
    "ring_finger_pip": (0.24930151196247122, 0.01059512121009555),
    "ring_finger_dip": (0.2285032688178066, 0.009917602877441107),
    "pinky_mcp_flex": (0.3655325975433942, 0.018616960278988272),
    "pinky_mcp_abd": (0.41393113081120425, 0.018732177029667153),
    "pinky_pip": (0.22729367621965954, 0.00951005441616486),
    "pinky_dip": (0.1964723550341816, 0.009017295241756363),
}


def _fmt(values: Sequence[float]) -> str:
    return " ".join(f"{float(value):.17g}" for value in values)


def _rpy_matrix(rpy: Sequence[float]) -> np.ndarray:
    roll, pitch, yaw = (float(item) for item in rpy)
    sr, cr = math.sin(roll), math.cos(roll)
    sp, cp = math.sin(pitch), math.cos(pitch)
    sy, cy = math.sin(yaw), math.cos(yaw)
    return np.array(
        (
            (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr),
        ),
        dtype=float,
    )


def rpy_to_mujoco_quat(rpy: Sequence[float]) -> tuple[float, float, float, float]:
    """Convert URDF fixed-axis RPY to MuJoCo's ``w x y z`` quaternion."""
    matrix = _rpy_matrix(rpy)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        root = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * root
        x = (matrix[2, 1] - matrix[1, 2]) / root
        y = (matrix[0, 2] - matrix[2, 0]) / root
        z = (matrix[1, 0] - matrix[0, 1]) / root
    else:
        diagonal = np.diag(matrix)
        index = int(np.argmax(diagonal))
        if index == 0:
            root = math.sqrt(max(0.0, 1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2])) * 2.0
            x = 0.25 * root
            y = (matrix[0, 1] + matrix[1, 0]) / root
            z = (matrix[0, 2] + matrix[2, 0]) / root
            w = (matrix[2, 1] - matrix[1, 2]) / root
        elif index == 1:
            root = math.sqrt(max(0.0, 1.0 - matrix[0, 0] + matrix[1, 1] - matrix[2, 2])) * 2.0
            x = (matrix[0, 1] + matrix[1, 0]) / root
            y = 0.25 * root
            z = (matrix[1, 2] + matrix[2, 1]) / root
            w = (matrix[0, 2] - matrix[2, 0]) / root
        else:
            root = math.sqrt(max(0.0, 1.0 - matrix[0, 0] - matrix[1, 1] + matrix[2, 2])) * 2.0
            x = (matrix[0, 2] + matrix[2, 0]) / root
            y = (matrix[1, 2] + matrix[2, 1]) / root
            z = 0.25 * root
            w = (matrix[1, 0] - matrix[0, 1]) / root
    quat = np.asarray((w, x, y, z), dtype=float)
    quat /= np.linalg.norm(quat)
    if quat[0] < 0.0:
        quat *= -1.0
    return tuple(float(item) for item in quat)


def _joint_map(model: UrdfModel) -> dict[str, UrdfJoint]:
    return {joint.child: joint for joint in model.joints}


def arm_projection_links(model: UrdfModel) -> frozenset[str]:
    """Return base, arm and fixed wrist-frame links for the 14-DoF view."""
    by_child = _joint_map(model)
    keep = {model.root, "l_wrist", "r_wrist"}
    for wrist in ("l_wrist", "r_wrist"):
        cursor = wrist
        while cursor != model.root:
            joint = by_child.get(cursor)
            if joint is None:
                raise ValueError(f"missing wrist chain for {wrist!r}")
            keep.add(cursor)
            keep.add(joint.parent)
            cursor = joint.parent
    # Fixed arm-base children and their seven revolute links per side.
    changed = True
    while changed:
        changed = False
        for joint in model.joints:
            if joint.parent in keep and joint.child not in keep and (
                joint.type == "fixed" or joint.name.startswith("Joint")
            ):
                keep.add(joint.child)
                changed = True
    return frozenset(keep)


def _add_inertial(parent: ET.Element, link: UrdfLink) -> None:
    if link.inertial is None:
        return
    inertia = np.asarray(link.inertial.inertia, dtype=float)
    values = (inertia[0, 0], inertia[1, 1], inertia[2, 2], inertia[0, 1], inertia[0, 2], inertia[1, 2])
    ET.SubElement(
        parent,
        "inertial",
        pos=_fmt(link.inertial.com),
        mass=f"{link.inertial.mass:.17g}",
        fullinertia=_fmt(values),
    )


def _add_geometry(
    body: ET.Element,
    link: UrdfLink,
    mesh_assets: Mapping[str, tuple[str, str, tuple[float, float, float]]],
    collision_assets: Mapping[tuple[str, int], Sequence[str]],
) -> None:
    for geometry in link.visuals:
        asset_name, _, _ = mesh_assets[str(geometry.path.resolve())]
        ET.SubElement(
            body,
            "geom",
            type="mesh",
            mesh=asset_name,
            pos=_fmt(geometry.origin),
            quat=_fmt(rpy_to_mujoco_quat(geometry.rpy)),
            contype="0",
            conaffinity="0",
            group="1",
            density="0",
            rgba="0.75 0.75 0.75 1",
        )
    for index, geometry in enumerate(link.collisions):
        pieces = collision_assets.get((link.name, index))
        if not pieces:
            raise ValueError(f"missing validated collision artifact for {link.name}[{index}]")
        pose = {"pos": _fmt(geometry.origin), "quat": _fmt(rpy_to_mujoco_quat(geometry.rpy))}
        for piece in pieces:
            ET.SubElement(
                body,
                "geom",
                type="mesh",
                mesh=piece,
                contype="1",
                conaffinity="1",
                group="0",
                density="0",
                **pose,
            )


def _children(model: UrdfModel) -> dict[str, tuple[UrdfJoint, ...]]:
    result: dict[str, list[UrdfJoint]] = {}
    for joint in model.joints:
        result.setdefault(joint.parent, []).append(joint)
    return {key: tuple(value) for key, value in result.items()}


def render_mjcf(
    model: UrdfModel,
    output_path: str | Path,
    *,
    mesh_assets: Mapping[str, tuple[str, str, tuple[float, float, float]]],
    collision_assets: Mapping[tuple[str, int], Sequence[str]],
    mode: str,
) -> tuple[tuple[str, ...], tuple[tuple[str, str], ...]]:
    """Write one full or arm-projection MJCF and return joint/exclude maps."""
    if mode not in {"full", "arm"}:
        raise ValueError("mode must be 'full' or 'arm'")
    allowed = frozenset(link.name for link in model.links) if mode == "full" else arm_projection_links(model)
    by_name = {link.name: link for link in model.links}
    child_map = _children(model)
    root = ET.Element("mujoco", model="tianji_wuji2_unified" if mode == "full" else "tianji_wuji2_arm")
    ET.SubElement(root, "compiler", angle="radian", meshdir=".", inertiafromgeom="false")
    ET.SubElement(root, "option", timestep=f"{1.0 / 480.0:.17g}", integrator="implicitfast")
    ET.SubElement(root, "size", nuser_jnt="1")
    asset_element = ET.SubElement(root, "asset")
    for source, (asset_name, asset_path, scale) in sorted(mesh_assets.items()):
        del source
        attributes = {"name": asset_name, "file": asset_path}
        if tuple(scale) != (1.0, 1.0, 1.0):
            attributes["scale"] = _fmt(scale)
        ET.SubElement(asset_element, "mesh", inertia="shell", **attributes)
    source_mesh_names = {asset_name for asset_name, _, _ in mesh_assets.values()}
    collision_names = sorted(
        {piece for pieces in collision_assets.values() for piece in pieces} - source_mesh_names
    )
    for piece_name in collision_names:
        ET.SubElement(asset_element, "mesh", name=piece_name, file=f"collision/{piece_name}.stl", inertia="shell")

    worldbody = ET.SubElement(root, "worldbody")
    joint_order: list[str] = []
    excludes: list[tuple[str, str]] = []

    def visit(parent_element: ET.Element, link_name: str, parent_joint: UrdfJoint | None) -> None:
        if link_name not in allowed:
            return
        link = by_name[link_name]
        attributes = {"name": link_name}
        if parent_joint is not None:
            attributes.update(pos=_fmt(parent_joint.origin), quat=_fmt(rpy_to_mujoco_quat(parent_joint.rpy)))
        body = ET.SubElement(parent_element, "body", **attributes)
        _add_inertial(body, link)
        if parent_joint is not None and parent_joint.type == "revolute":
            if parent_joint.limit is None:
                raise ValueError(f"revolute joint {parent_joint.name!r} has no limit")
            is_arm = parent_joint.name.startswith("Joint")
            ET.SubElement(
                body,
                "joint",
                name=parent_joint.name,
                type="hinge",
                axis=_fmt(parent_joint.axis),
                range=_fmt(parent_joint.limit),
                limited="true",
                # The official Hand 2 servo calibration uses actuator ``kv``
                # with zero passive joint damping.  Keeping the arm damping is
                # intentional and independent from the hand calibration.
                damping="0.1" if is_arm else "0",
            )
            joint_order.append(parent_joint.name)
        _add_geometry(body, link, mesh_assets, collision_assets)
        if link_name in {"l_wrist", "r_wrist"}:
            ET.SubElement(body, "site", name=f"{link_name}_target", pos="0 0 0", size="0.008", rgba="1 0.2 0.2 1")
        for child_joint in child_map.get(link_name, ()):
            if child_joint.child not in allowed:
                continue
            excludes.append((link_name, child_joint.child))
            visit(body, child_joint.child, child_joint)

    visit(worldbody, model.root, None)
    if mode == "full":
        ET.SubElement(worldbody, "geom", name="ground", type="plane", size="5 5 0.1", contype="1", conaffinity="1")
        ET.SubElement(worldbody, "light", name="key_light", pos="1 -1 3", dir="-1 1 -3", directional="true")
        ET.SubElement(worldbody, "camera", name="overview", pos="2 -2 1.8", quat="1 0 0 0", fovy="45")

    actuator = ET.SubElement(root, "actuator")
    joint_by_name = {joint.name: joint for joint in model.joints}
    for name in joint_order:
        joint = joint_by_name[name]
        is_arm = name.startswith("Joint")
        effort = abs(float(joint.effort)) if joint.effort is not None and abs(float(joint.effort)) > 0 else 1.0
        if is_arm:
            kp = 500.0
            kv = None
        else:
            hand_joint_name = name[2:] if name.startswith(("l_", "r_")) else name
            try:
                kp, kv = _WUJI2_HAND_GAINS[hand_joint_name]
            except KeyError as exc:
                raise ValueError(
                    f"missing official Wuji Hand 2 gains for {name!r}"
                ) from exc
        attributes = {
            "name": f"{name}_position",
            "joint": name,
            "kp": f"{kp:.17g}",
            "forcerange": f"{-effort:.17g} {effort:.17g}",
            "forcelimited": "true",
        }
        if is_arm:
            attributes["dampratio"] = "1"
        else:
            assert kv is not None
            attributes["kv"] = f"{kv:.17g}"
        if joint.limit is not None:
            attributes.update(ctrlrange=_fmt(joint.limit), ctrllimited="true")
        ET.SubElement(actuator, "position", **attributes)

    contact = ET.SubElement(root, "contact")
    for parent, child in excludes:
        ET.SubElement(contact, "exclude", body1=parent, body2=child)
    ET.indent(root, space="  ")
    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(output, encoding="utf-8", xml_declaration=True, short_empty_elements=True)
    return tuple(joint_order), tuple(excludes)


__all__ = ["arm_projection_links", "render_mjcf", "rpy_to_mujoco_quat"]
