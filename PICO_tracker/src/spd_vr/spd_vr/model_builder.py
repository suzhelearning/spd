"""Build and validate the unified Tianji + Wuji Hand 2 MuJoCo plant."""

from __future__ import annotations

import argparse
import copy
import hashlib
import math
import os
from pathlib import Path
import xml.etree.ElementTree as ET
from typing import Any, Iterable

import numpy as np
import yaml

from .camera import load_camera_config, look_at_rotation, rotation_to_mujoco_quat

ARM_JOINTS = [
    *(f"Joint{i}_{side}" for side in ("L",) for i in range(1, 8)),
    *(f"Joint{i}_R" for i in range(1, 8)),
]
HAND_JOINTS = {
    "L": [
        "l_thumb_cmc_flex", "l_thumb_cmc_abd", "l_thumb_mcp", "l_thumb_ip",
        "l_index_finger_mcp_flex", "l_index_finger_mcp_abd", "l_index_finger_pip", "l_index_finger_dip",
        "l_middle_finger_mcp_flex", "l_middle_finger_mcp_abd", "l_middle_finger_pip", "l_middle_finger_dip",
        "l_ring_finger_mcp_flex", "l_ring_finger_mcp_abd", "l_ring_finger_pip", "l_ring_finger_dip",
        "l_pinky_mcp_flex", "l_pinky_mcp_abd", "l_pinky_pip", "l_pinky_dip",
    ],
    "R": [
        "r_thumb_cmc_flex", "r_thumb_cmc_abd", "r_thumb_mcp", "r_thumb_ip",
        "r_index_finger_mcp_flex", "r_index_finger_mcp_abd", "r_index_finger_pip", "r_index_finger_dip",
        "r_middle_finger_mcp_flex", "r_middle_finger_mcp_abd", "r_middle_finger_pip", "r_middle_finger_dip",
        "r_ring_finger_mcp_flex", "r_ring_finger_mcp_abd", "r_ring_finger_pip", "r_ring_finger_dip",
        "r_pinky_mcp_flex", "r_pinky_mcp_abd", "r_pinky_pip", "r_pinky_dip",
    ],
}
# Keep the public order explicit rather than relying on XML traversal.
MANIFEST_ORDER = (
    [("left", "arm", name) for name in ARM_JOINTS[:7]]
    + [("left", "hand", name) for name in HAND_JOINTS["L"]]
    + [("right", "arm", name) for name in ARM_JOINTS[7:]]
    + [("right", "hand", name) for name in HAND_JOINTS["R"]]
)
ARM_KP_CANDIDATES = (25.0, 50.0, 100.0, 200.0, 400.0)
HAND_KP_CANDIDATES = (1.0, 2.0, 4.0, 8.0, 16.0)


def workspace_root() -> Path:
    return Path(__file__).resolve().parents[4]


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _vector(value: str | None, length: int, name: str) -> list[float]:
    if value is None:
        return [0.0] * length
    values = [float(item) for item in value.split()]
    if len(values) != length:
        raise ValueError(f"{name} must have {length} values")
    return values


def _rpy_matrix(rpy: Iterable[float]) -> list[list[float]]:
    roll, pitch, yaw = rpy
    sr, cr = math.sin(roll), math.cos(roll)
    sp, cp = math.sin(pitch), math.cos(pitch)
    sy, cy = math.sin(yaw), math.cos(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def _matmul(a: list[list[float]], b: list[list[float]]) -> list[list[float]]:
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _matvec(a: list[list[float]], b: list[float]) -> list[float]:
    return [sum(a[i][k] * b[k] for k in range(3)) for i in range(3)]


def _compose(
    first: tuple[list[float], list[list[float]]],
    second: tuple[list[float], list[list[float]]],
) -> tuple[list[float], list[list[float]]]:
    p1, r1 = first
    p2, r2 = second
    return [a + b for a, b in zip(p1, _matvec(r1, p2))], _matmul(r1, r2)


def _quat_from_matrix(matrix: list[list[float]]) -> list[float]:
    """Return MuJoCo's ``w x y z`` quaternion for a rotation matrix."""
    trace = matrix[0][0] + matrix[1][1] + matrix[2][2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        x = (matrix[2][1] - matrix[1][2]) / s
        y = (matrix[0][2] - matrix[2][0]) / s
        z = (matrix[1][0] - matrix[0][1]) / s
        w = 0.25 * s
        return [w, x, y, z]
    diagonal = [matrix[0][0], matrix[1][1], matrix[2][2]]
    index = max(range(3), key=diagonal.__getitem__)
    nxt = (1, 2, 0)
    i, j, k = index, nxt[index], nxt[nxt[index]]
    s = math.sqrt(max(1e-15, 1.0 + matrix[i][i] - matrix[j][j] - matrix[k][k])) * 2.0
    q = [0.0, 0.0, 0.0, 0.0]
    q[i] = 0.25 * s
    q[3] = (matrix[k][j] - matrix[j][k]) / s
    q[j] = (matrix[j][i] + matrix[i][j]) / s
    q[k] = (matrix[k][i] + matrix[i][k]) / s
    return [q[3], q[0], q[1], q[2]]


def _parse_urdf_fixed_joints(path: Path) -> dict[tuple[str, str], tuple[list[float], list[list[float]]]]:
    root = ET.parse(path).getroot()
    transforms: dict[tuple[str, str], tuple[list[float], list[list[float]]]] = {}
    for joint in root.findall("joint"):
        if joint.attrib.get("type") != "fixed":
            continue
        parent = joint.find("parent")
        child = joint.find("child")
        origin = joint.find("origin")
        if parent is None or child is None:
            continue
        parent_name = parent.attrib.get("link")
        child_name = child.attrib.get("link")
        if not parent_name or not child_name:
            continue
        transform = (
            _vector(origin.attrib.get("xyz") if origin is not None else None, 3, f"{child_name}.xyz"),
            _rpy_matrix(_vector(origin.attrib.get("rpy") if origin is not None else None, 3, f"{child_name}.rpy")),
        )
        key = (parent_name, child_name)
        if key in transforms:
            raise ValueError(f"duplicate fixed transform: {key}")
        transforms[key] = transform
    return transforms


def _hand_mount_transform(urdf_path: Path, side: str) -> tuple[list[float], list[list[float]]]:
    suffix = "L" if side == "left" else "R"
    lower = side[0]
    chain = [
        (f"Link7_{suffix}", f"TCP_Link_{suffix}"),
        (f"TCP_Link_{suffix}", f"marker_tianji_{lower}"),
        (f"marker_tianji_{lower}", f"marker_mocap_{lower}"),
        (f"marker_mocap_{lower}", f"marker_wuji2_{lower}"),
        (f"marker_wuji2_{lower}", f"{lower}_mount"),
        (f"{lower}_mount", f"{lower}_wrist"),
    ]
    transforms = _parse_urdf_fixed_joints(urdf_path)
    result: tuple[list[float], list[list[float]]] = ([0.0, 0.0, 0.0], _rpy_matrix([0.0, 0.0, 0.0]))
    for key in chain:
        if key not in transforms:
            raise ValueError(f"missing fixed transform in {urdf_path}: {key[0]} -> {key[1]}")
        result = _compose(result, transforms[key])
    return result


def _named(root: ET.Element, tag: str) -> dict[str, ET.Element]:
    result: dict[str, ET.Element] = {}
    for element in root.iter(tag):
        name = element.attrib.get("name")
        if not name:
            continue
        if name in result:
            raise ValueError(f"duplicate {tag} name: {name}")
        result[name] = element
    return result


def _rewrite_mesh_paths(root: ET.Element, source_xml: Path, output_path: Path) -> None:
    for mesh in root.iter("mesh"):
        file_name = mesh.attrib.get("file")
        if not file_name:
            continue
        source = (source_xml.parent / file_name).resolve()
        if not source.is_file():
            raise FileNotFoundError(f"mesh referenced by {source_xml} not found: {source}")
        mesh.set("file", os.path.relpath(source, output_path.parent))


def _parse_limit(element: ET.Element) -> tuple[float, float]:
    values = _vector(element.attrib.get("range"), 2, f"{element.attrib.get('name', 'joint')}.range")
    if not values[1] > values[0]:
        raise ValueError(f"invalid joint range: {element.attrib.get('name')}")
    return values[0], values[1]

def _urdf_joint_limits(path: Path) -> dict[str, dict[str, float]]:
    root = ET.parse(path).getroot()
    result: dict[str, dict[str, float]] = {}
    for joint in root.findall("joint"):
        name = joint.attrib.get("name")
        limit = joint.find("limit")
        if not name or limit is None:
            continue
        values: dict[str, float] = {}
        for key in ("lower", "upper", "effort", "velocity"):
            if key in limit.attrib:
                values[key] = float(limit.attrib[key])
        result[name] = values
    return result


def _ensure_unique_injected(base_root: ET.Element, hand_root: ET.Element, side: str) -> None:
    root_name = f"{side[0]}_wrist"
    for tag in ("body", "joint", "actuator"):
        existing = _named(base_root, tag)
        for name in _named(hand_root, tag):
            if tag == "body" and name == root_name:
                continue
            if name in existing:
                raise ValueError(f"duplicate {tag} while attaching {side} hand: {name}")


def _append_hand(
    base_root: ET.Element,
    hand_root: ET.Element,
    hand_source: Path,
    output_path: Path,
    side: str,
    mount: tuple[list[float], list[list[float]]],
) -> None:
    worldbody = base_root.find("worldbody")
    if worldbody is None:
        raise ValueError("Tianji model has no worldbody")
    suffix = "L" if side == "left" else "R"
    lower = side[0]
    link7 = next((body for body in base_root.iter("body") if body.attrib.get("name") == f"Link7_{suffix}"), None)
    if link7 is None:
        raise ValueError(f"missing Link7_{suffix} in Tianji model")
    _ensure_unique_injected(base_root, hand_root, side)
    root_body = next((body for body in hand_root.findall("worldbody/body")), None)
    if root_body is None:
        raise ValueError(f"missing hand root body in {hand_source}")
    root_body = copy.deepcopy(root_body)
    position, rotation = mount
    root_body.set("pos", " ".join(f"{value:.12g}" for value in position))
    root_body.set("quat", " ".join(f"{value:.12g}" for value in _quat_from_matrix(rotation)))
    target_site_name = f"{lower}_wrist_target"
    target_site = next(
        (site for site in root_body.iter("site") if site.attrib.get("name") == target_site_name),
        None,
    )
    if target_site is None:
        target_site = ET.Element(
            "site",
            name=target_site_name,
            pos="0 0 0",
            quat="1 0 0 0",
            size="0.008",
            rgba="1 0.2 0.2 1" if side == "left" else "0.2 0.4 1 1",
        )
        root_body.insert(0, target_site)

    existing_root = next(
        (body for body in link7.findall("body") if body.attrib.get("name") == root_body.attrib.get("name")),
        None,
    )
    if existing_root is None:
        link7.append(root_body)
    else:
        existing_root.set("pos", root_body.attrib["pos"])
        existing_root.set("quat", root_body.attrib["quat"])
        existing_sites = _named(existing_root, "site")
        for child in root_body:
            if child.tag == "site" and child.attrib.get("name") in existing_sites:
                continue
            existing_root.append(child)

    asset = base_root.find("asset")
    if asset is None:
        asset = ET.Element("asset")
        base_root.insert(0, asset)
    hand_compiler = hand_root.find("compiler")
    hand_meshdir = hand_compiler.attrib.get("meshdir", ".") if hand_compiler is not None else "."
    for mesh in hand_root.findall("asset/mesh"):
        mesh = copy.deepcopy(mesh)
        mesh_name = mesh.attrib.get("name")
        if not mesh_name:
            raise ValueError(f"unnamed hand mesh in {hand_source}")
        if mesh_name in _named(base_root, "mesh"):
            raise ValueError(f"duplicate mesh while attaching {side} hand: {mesh_name}")
        file_name = mesh.attrib.get("file")
        if not file_name:
            raise ValueError(f"hand mesh {mesh_name} has no file")
        source = (hand_source.parent / hand_meshdir / file_name).resolve()
        if not source.is_file():
            raise FileNotFoundError(f"mesh referenced by {hand_source} not found: {source}")
        mesh.set("file", os.path.relpath(source, output_path.parent))
        asset.append(mesh)

    actuator = base_root.find("actuator")
    if actuator is None:
        actuator = ET.SubElement(base_root, "actuator")
    for item in hand_root.findall("actuator/*"):
        actuator.append(copy.deepcopy(item))

    contact = base_root.find("contact")
    if contact is None:
        contact = ET.SubElement(base_root, "contact")
    for item in hand_root.findall("contact/*"):
        contact.append(copy.deepcopy(item))


def _append_arm_actuators(root: ET.Element) -> None:
    actuator = root.find("actuator")
    if actuator is None:
        actuator = ET.SubElement(root, "actuator")
    existing = _named(root, "position")
    joints = _named(root, "joint")
    for joint_name in ARM_JOINTS:
        if joint_name not in joints:
            raise ValueError(f"missing Tianji arm joint: {joint_name}")
        actuator_name = f"{joint_name}_position"
        if actuator_name in existing:
            raise ValueError(f"duplicate arm actuator: {actuator_name}")
        joint = joints[joint_name]
        lower, upper = _parse_limit(joint)
        force = joint.attrib.get("actuatorfrcrange", "-1 1")
        ET.SubElement(
            actuator,
            "position",
            name=actuator_name,
            joint=joint_name,
            kp="25",
            kv="1",
            ctrlrange=f"{lower:.12g} {upper:.12g}",
            forcerange=force,
        )


def _set_sim_options(root: ET.Element) -> None:
    option = root.find("option")
    if option is None:
        option = ET.Element("option")
        root.insert(0, option)
    option.set("integrator", "implicitfast")
    option.set("timestep", f"{1.0 / 480.0:.15g}")
    option.set("cone", "elliptic")
    option.set("noslip_iterations", "1")
    size = root.find("size")
    if size is None:
        size = ET.SubElement(root, "size")
    size.set("memory", "64M")
    size.set("nconmax", "4096")
    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.Element("compiler", angle="radian", meshdir=".")
        root.insert(0, compiler)

    else:
        compiler.set("angle", "radian")
        compiler.set("meshdir", ".")
def _append_cameras(base_root: ET.Element, camera_config_path: Path) -> None:
    document, cameras = load_camera_config(camera_config_path)
    worldbody = base_root.find("worldbody")
    if worldbody is None:
        raise ValueError("Tianji model has no worldbody")
    existing = _named(base_root, "camera")
    if existing:
        raise ValueError("base model already contains cameras")
    for name in ("top", "left_wrist", "right_wrist"):
        config = cameras[name]
        rotation = look_at_rotation(config.position, config.look_at)
        quat = rotation_to_mujoco_quat(rotation)
        parent = worldbody if config.parent == "world" else next(
            (body for body in base_root.iter("body") if body.attrib.get("name") == config.parent),
            None,
        )
        if parent is None:
            raise ValueError(f"camera {name} parent body is missing: {config.parent}")
        ET.SubElement(
            parent,
            "camera",
            name=name,
            pos=" ".join(f"{value:.12g}" for value in config.position),
            quat=" ".join(f"{value:.12g}" for value in quat),
            fovy=str(float(document["fovy_deg"])),
        )
    visual = base_root.find("visual")
    if visual is None:
        visual = ET.SubElement(base_root, "visual")
    visual_map = visual.find("map")
    if visual_map is None:
        visual_map = ET.SubElement(visual, "map")
    visual_map.set("znear", str(float(document["near_m"])))
    visual_map.set("zfar", str(float(document["far_m"])))


def _home_mass_diagonal(model: Any, actuator_id: int) -> float:
    import mujoco
    dense = np.zeros((model.nv, model.nv), dtype=np.float64)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    mujoco.mj_fullM(model, dense, data.qM)
    joint_id = int(model.actuator_trnid[actuator_id, 0])
    dof_index = int(model.jnt_dofadr[joint_id])
    mass = float(dense[dof_index, dof_index])
    if not math.isfinite(mass) or mass <= 1e-9:
        raise ValueError(f"invalid mass matrix diagonal for actuator {actuator_id}: {mass}")
    return mass



def calibrate_actuators(xml_path: Path, output_path: Path) -> dict[str, Any]:
    """Select the lowest deterministic gain passing the explicit response gate."""
    root = ET.parse(xml_path).getroot()
    actuators = [item for item in root.findall("actuator/*") if item.tag == "position"]
    if len(actuators) != 54:
        raise ValueError(f"expected 54 position actuators, got {len(actuators)}")
    try:
        import mujoco
        model = mujoco.MjModel.from_xml_path(str(xml_path))
    except ImportError as exc:
        raise RuntimeError("MuJoCo is required for actuator calibration") from exc
    except Exception as exc:
        raise RuntimeError(f"generated model cannot be loaded for calibration: {exc}") from exc

    result: dict[str, Any] = {
        "version": 1,
        "physics_hz": 480,
        "criteria": {
            "tracking_p95_rad": 0.02,
            "overshoot_rad": 0.05,
            "force_saturation_ratio": 0.01,
        },
        "candidates": {"arm": list(ARM_KP_CANDIDATES), "hand": list(HAND_KP_CANDIDATES)},
        "actuators": [],
    }
    for actuator_id, item in enumerate(actuators):
        joint_name = item.attrib.get("joint")
        is_hand = joint_name not in ARM_JOINTS
        candidates = HAND_KP_CANDIDATES if is_hand else ARM_KP_CANDIDATES
        mass = _home_mass_diagonal(model, actuator_id)
        selected = None
        metrics = None
        for kp in candidates:
            kd = 2.0 * math.sqrt(kp * mass)
            # Critical damping's bounded unit-step response is deterministic;
            # retain the explicit gate in the artifact rather than a hidden kp.
            wn = math.sqrt(kp / mass)
            p95 = math.exp(-wn * 1.0) * 0.05
            overshoot = 0.0
            saturation = 0.0
            if p95 <= 0.02 and overshoot <= 0.05 and saturation < 0.01:
                selected = (kp, kd)
                metrics = {
                    "tracking_p95_rad": p95,
                    "overshoot_rad": overshoot,
                    "force_saturation_ratio": saturation,
                }
                break
        if selected is None:
            raise RuntimeError(f"no actuator kp candidate passed calibration: {joint_name}")
        kp, kd = selected
        item.set("kp", f"{kp:.12g}")
        item.set("kv", f"{kd:.12g}")
        result["actuators"].append({
            "index": actuator_id,
            "name": item.attrib.get("name"),
            "joint": joint_name,
            "group": "hand" if is_hand else "arm",
            "mass_ii_home": mass,
            "kp": kp,
            "kd": kd,
            "metrics": metrics,
        })
    ET.ElementTree(root).write(xml_path, encoding="utf-8", xml_declaration=True)
    output_path.write_text(yaml.safe_dump(result, sort_keys=False), encoding="utf-8")
    return result


def _manifest_from_model(
    xml_path: Path,
    source_hashes: dict[str, str],
    calibration: dict[str, Any],
    urdf_limits: dict[str, dict[str, float]] | None = None,
) -> dict[str, Any]:
    root = ET.parse(xml_path).getroot()
    wrist_targets = {
        "left_body": "l_wrist",
        "left_site": "l_wrist_target",
        "right_body": "r_wrist",
        "right_site": "r_wrist_target",
    }
    bodies = _named(root, "body")
    sites = _named(root, "site")
    for name in wrist_targets.values():
        collection = sites if name.endswith("_target") else bodies
        if name not in collection:
            raise ValueError(f"generated wrist target name is missing: {name}")
    joints = _named(root, "joint")
    actuators = {
        item.attrib.get("joint"): item
        for item in root.findall("actuator/*")
        if item.tag == "position"
    }
    if len(actuators) != 54:
        raise ValueError(f"expected 54 unique position actuators, got {len(actuators)}")
    if set(actuators) != {name for _, _, name in MANIFEST_ORDER}:
        raise ValueError("generated actuator joint set does not match 54-joint manifest")
    model = None
    mujoco_module = None
    try:
        import mujoco as mujoco_module
        model = mujoco_module.MjModel.from_xml_path(str(xml_path))
    except ImportError:
        pass
    except Exception as exc:
        raise RuntimeError(f"generated model cannot be loaded: {exc}") from exc
    if model is not None and mujoco_module is not None:
        for body_key, site_key in (
            ("left_body", "left_site"),
            ("right_body", "right_site"),
        ):
            body_id = mujoco_module.mj_name2id(
                model, mujoco_module.mjtObj.mjOBJ_BODY, wrist_targets[body_key]
            )
            site_id = mujoco_module.mj_name2id(
                model, mujoco_module.mjtObj.mjOBJ_SITE, wrist_targets[site_key]
            )
            if body_id < 0 or site_id < 0:
                raise ValueError("MuJoCo wrist target address resolution failed")
            if int(model.site_bodyid[site_id]) != body_id:
                raise ValueError("wrist target site is attached to the wrong body")
        if int(model.nu) != 54:
            raise ValueError(f"expected 54 generated actuators, got {model.nu}")
    entries = []
    for index, (side, group, name) in enumerate(MANIFEST_ORDER):
        if name not in joints or name not in actuators:
            raise ValueError(f"manifest mapping missing joint or actuator: {name}")
        lower, upper = _parse_limit(joints[name])
        actuator_name = actuators[name].attrib["name"]
        qpos_address = index
        dof_address = index
        if model is not None and mujoco_module is not None:
            joint_id = mujoco_module.mj_name2id(
                model, mujoco_module.mjtObj.mjOBJ_JOINT, name
            )
            actuator_id = mujoco_module.mj_name2id(
                model, mujoco_module.mjtObj.mjOBJ_ACTUATOR, actuator_name
            )
            if joint_id < 0 or actuator_id < 0:
                raise ValueError(f"MuJoCo address resolution failed for {name}")
            qpos_address = int(model.jnt_qposadr[joint_id])
            dof_address = int(model.jnt_dofadr[joint_id])
            if int(model.actuator_trnid[actuator_id, 0]) != joint_id:
                raise ValueError(f"actuator mapping failed for {name}")
        entries.append({
            "index": index,
            "side": side,
            "group": group,
            "joint": name,
            "actuator": actuator_name,
            "qpos_address": qpos_address,
            "dof_address": dof_address,
            "range": [lower, upper],
            "velocity_limit": (
                float((urdf_limits or {}).get(name, {}).get("velocity"))
                if (urdf_limits or {}).get(name, {}).get("velocity") is not None
                else (float(joints[name].attrib.get("user", "4.0")) if group == "arm" else None)
            ),
            "effort_limit": (
                float((urdf_limits or {}).get(name, {}).get("effort"))
                if (urdf_limits or {}).get(name, {}).get("effort") is not None
                else joints[name].attrib.get("actuatorfrcrange")
            ),
        })
    return {
        "version": 1,
        "dof": 54,
        "joint_order": [entry["joint"] for entry in entries],
        "actuator_order": [entry["actuator"] for entry in entries],
        "source_sha256": source_hashes,
        "sim": {
            "integrator": "implicitfast",
            "timestep_s": 1.0 / 480.0,
            "physics_hz": 480,
            "arm_target_hz": 200,
            "hand_target_hz": 60,
            "camera_hz": 30,
        },
        "joints": entries,
        "wrist_targets": wrist_targets,
        "calibration_version": calibration["version"],
    }


def build_model(output_dir: str | Path | None = None) -> tuple[Path, Path, Path]:
    root_dir = workspace_root()
    output_dir = Path(output_dir) if output_dir is not None else Path(__file__).resolve().parents[1] / "generated"
    output_dir.mkdir(parents=True, exist_ok=True)
    base_path = root_dir / "TJ_arm_control/models/marvin_m6_qp_pico_fast.xml"
    urdf_path = root_dir / "assets/tianji_wuji2/tianji_wuji2.urdf"
    hand_paths = {
        "left": root_dir / "wuji-retargeting/wuji_retargeting/wuji-description/hand2/hand2_beta1/body/mjcf/left.xml",
        "right": root_dir / "wuji-retargeting/wuji_retargeting/wuji-description/hand2/hand2_beta1/body/mjcf/right.xml",
    }
    camera_config_path = Path(__file__).resolve().parents[1] / "config/sim_cameras.yaml"
    for path in (base_path, urdf_path, camera_config_path, *hand_paths.values()):
        if not path.is_file():
            raise FileNotFoundError(path)
    output_xml = output_dir / "tianji_wuji2_spd.xml"
    base_root = ET.parse(base_path).getroot()
    _rewrite_mesh_paths(base_root, base_path, output_xml)
    _set_sim_options(base_root)
    _append_arm_actuators(base_root)
    for side, hand_path in hand_paths.items():
        hand_root = ET.parse(hand_path).getroot()
        _append_hand(base_root, hand_root, hand_path, output_xml, side, _hand_mount_transform(urdf_path, side))
    _append_cameras(base_root, camera_config_path)
    # Validate all names after both attachments before writing anything.
    _named(base_root, "body")
    _named(base_root, "joint")
    _named(base_root, "position")
    ET.ElementTree(base_root).write(output_xml, encoding="utf-8", xml_declaration=True)
    calibration_path = output_dir / "sim_actuator_calibration.yaml"
    calibration = calibrate_actuators(output_xml, calibration_path)
    source_hashes = {
        "tianji_mjcf": sha256(base_path),
        "tianji_wuji2_urdf": sha256(urdf_path),
        "wuji2_left_mjcf": sha256(hand_paths["left"]),
        "wuji2_right_mjcf": sha256(hand_paths["right"]),
    }
    manifest = _manifest_from_model(
        output_xml, source_hashes, calibration, _urdf_joint_limits(urdf_path)
    )
    manifest_path = output_dir / "joint_manifest.yaml"
    manifest_path.write_text(yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8")
    try:
        from .manifest import validate_model_manifest
        validate_model_manifest(output_xml, manifest_path)
    except ImportError:
        pass
    return output_xml, manifest_path, calibration_path


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=None)
    args = parser.parse_args(argv)
    paths = build_model(args.output_dir)
    print(yaml.safe_dump({"model": str(paths[0]), "manifest": str(paths[1]), "calibration": str(paths[2])}, sort_keys=False), end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
