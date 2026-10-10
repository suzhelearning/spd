"""Deterministic fine-grained task assets with separate visual/contact layers.

Geometry follows task affordances, not unreported paper CAD dimensions.
ABC-derived bottles use local textured meshes and convex collision pieces;
other objects retain physically open cavities under detailed visual surfaces.
"""

from __future__ import annotations

from copy import deepcopy
from dataclasses import asdict, dataclass, field, replace
import math
from pathlib import Path
from typing import Any, Iterable
import xml.etree.ElementTree as ET

import numpy as np

from .abc_assets import BOTTLE_VARIANTS, bottle_geometry
from .visual_details import appearance_count, build_visual_details
from .physical_materials import (
    apply_material_policy, load_task_material_coefficients, material_manifest,
    object_surface_materials, task_material_config_path, validate_material_coefficients,
)

GEOMETRY_REVISION = "paper-aligned-scenes-v2"

# Reference height for task layouts; each build shifts them to its sampled tabletop.
TABLE_Z = 0.75
TABLE_HEIGHT_RANGE = (0.70, 0.80)
TABLE_DISTANCE_RANGE = (0.10, 0.30)
BIN_X_RANGE = (0.40, 0.60)
WORKSPACE_CENTER = (0.45, 0.0, TABLE_Z)
WORKSPACE_X = (0.10, 0.80)
WORKSPACE_Y = (-0.55, 0.55)
MAX_RESET_CANDIDATES = 32

JENGA_BLOCK_SIZE = (0.075, 0.025, 0.015)
DOMINO_SIZE = (0.025, 0.015, 0.075)
LETTER_BLOCK_SIZE = (0.040, 0.040, 0.040)
PLATE_RADIUS = 0.100
PLATE_THICKNESS = 0.008
CUP_OUTER_RADIUS = 0.045
CUP_BOTTOM_RADIUS = 0.028
CUP_HEIGHT = 0.090
CUP_WALL = 0.002
CUP_NEST_STEP = 0.018
VESSEL_SIDES = 32
BOTTLE_RADIUS = 0.035
BOTTLE_HEIGHT = 0.180
BIN_INNER_SIZE = (0.350, 0.250, 0.150)
FIXTURE_CLASSES = frozenset({"rack", "mug_tree", "bin", "cabinet"})
CABINET_SIZE = (0.260, 0.320, 0.426)
DRAWER_SIZE = (0.238, 0.292, 0.085)
DRAWER_FLOORS = (0.018, 0.154, 0.290)
DRAWER_TRAVEL = 0.250
CUP_PROFILES = (
    (0.045, 0.090, 0.002, 0.028, 0.018),
    (0.048, 0.096, 0.002, 0.029, 0.020),
    (0.041, 0.082, 0.0018, 0.025, 0.017),
)
MUG_PROFILES = ((0.040, 0.090, 0.003), (0.036, 0.080, 0.0028), (0.044, 0.100, 0.0032))
PLATE_PROFILES = ((0.100, 0.008), (0.090, 0.007), (0.110, 0.009))
BIN_PROFILES = ((0.350, 0.250, 0.150), (0.310, 0.280, 0.170), (0.370, 0.230, 0.130))

CLASS_IDS = {
    "jenga_block": 1,
    "letter_block": 2,
    "plate": 3,
    "cup": 4,
    "mug": 5,
    "bottle": 6,
    "rack": 7,
    "mug_tree": 8,
    "bin": 9,
    "domino": 10,
    "cabinet": 11,
    "drawer": 12,
}
# Legacy appearance/fallback contact ranges. Physical surface labels are separate.
# Wood mass uses 650 kg/m^3; nominal plate mass retains the reference disk estimate.
# Empty-vessel masses are assigned to collision proxies; visuals contribute no mass.
_MATERIALS = {
    "jenga_block": ("wood", (0.40, 0.60)),
    "letter_block": ("wood", (0.40, 0.60)),
    "domino": ("wood", (0.40, 0.60)),
    "mug": ("ceramic", (0.25, 0.40)),
    "plate": ("ceramic", (0.25, 0.40)),
    "cup": ("plastic", (0.20, 0.35)),
    "bottle": ("plastic", (0.30, 0.55)),
    "rack": ("coated_metal", (0.35, 0.55)),
    "mug_tree": ("wood", (0.40, 0.60)),
    "bin": ("plastic", (0.30, 0.55)),
    "cabinet": ("wood", (0.40, 0.60)),
    "drawer": ("wood", (0.35, 0.50)),
}

BASE_MASSES = {
    "jenga_block": 650.0 * math.prod(JENGA_BLOCK_SIZE),
    "letter_block": 650.0 * math.prod(LETTER_BLOCK_SIZE),
    "plate": 2400.0 * math.pi * PLATE_RADIUS ** 2 * PLATE_THICKNESS,
    "cup": 0.030,
    "mug": 0.220,
    "bottle": 0.060,
    "rack": 0.500,
    "mug_tree": 0.400,
    "bin": 0.800,
    "domino": 650.0 * math.prod(DOMINO_SIZE),
    "cabinet": 3.0,
    "drawer": 0.45,
}

# Cohesive, seeded appearance choices instead of arbitrary RGB hues.
_PALETTES = {
    "wood": ((0.76, 0.60, 0.40), (0.66, 0.48, 0.29), (0.84, 0.70, 0.49)),
    "ceramic": ((0.91, 0.89, 0.82), (0.27, 0.46, 0.53), (0.70, 0.39, 0.28)),
    "plastic": ((0.25, 0.43, 0.48), (0.74, 0.40, 0.28), (0.78, 0.69, 0.42)),
    "coated_metal": ((0.24, 0.27, 0.29), (0.56, 0.58, 0.58), (0.83, 0.81, 0.74)),
}
SPELLING_WORDS = ("CAFE", "IMAGE", "ROBOTICS", "ROBOT", "TABLE", "CUP", "BLOCKS", "VISION")


def _letters_for(objects: tuple[ObjectSpec, ...], seed: int, task: str, target_word: str) -> dict[str, str]:
    blocks = [obj for obj in objects if obj.class_name == "letter_block"]
    if not blocks:
        return {}
    rng = np.random.default_rng(seed)
    if task == "spelling":
        letters = rng.permutation(list(target_word))
    elif task == "vowel_consonant_sort":
        letters = np.concatenate((
            rng.choice(list("AEIOU"), 4, replace=False),
            rng.choice(list("BCDFGHJKLMNPQRSTVWXYZ"), len(blocks) - 4, replace=False),
        ))
        rng.shuffle(letters)
    else:
        letters = rng.choice(list("ABCDEFGHIJKLMNOPQRSTUVWXYZ"), len(blocks), replace=False)
    return {obj.name: str(letter) for obj, letter in zip(blocks, letters, strict=True)}


def _geom_attributes(geom: dict[str, Any]) -> dict[str, str]:
    return {
        key: " ".join(f"{value:.12g}" for value in item)
        if isinstance(item, (tuple, list, np.ndarray)) else str(item)
        for key, item in geom.items() if key != "volume"
    }


class SceneResetError(RuntimeError):
    """Raised when deterministic placement cannot pass the contact gate."""


@dataclass(frozen=True)
class ObjectSpec:
    instance_id: int
    class_id: int
    class_name: str
    name: str
    position: tuple[float, float, float]
    yaw_rad: float
    size: tuple[float, ...]
    mass_kg: float
    friction: float
    contact_group: str
    assembled: bool
    color_rgb: tuple[float, float, float]
    geoms: tuple[dict[str, Any], ...]
    asset_id: str
    appearance_variant: int
    visual_geoms: tuple[dict[str, Any], ...] = ()
    asset_definitions: tuple[dict[str, Any], ...] = ()
    asset_provenance: dict[str, Any] = field(default_factory=dict)
    joint: dict[str, Any] = field(default_factory=dict)

    def manifest(self) -> dict[str, Any]:
        values = asdict(self)
        values["collision_debug_rgb"] = values.pop("color_rgb")
        if self.class_name in _MATERIALS:
            _, friction_range = _MATERIALS[self.class_name]
            surfaces = object_surface_materials(self)
            values.update(material=surfaces["default"], surface_materials=surfaces,
                          friction_range=list(friction_range),
                          reference_mass_kg=BASE_MASSES[self.class_name],
                          material_parameter_source="user-specified surface labels; legacy geom coefficients uncalibrated")
        return values


@dataclass(frozen=True)
class SceneBuildResult:
    scene: str
    task: str
    seed: int
    candidate: int
    objects: tuple[ObjectSpec, ...]
    sampled_values: dict[str, Any]
    worldbody: ET.Element
    assets: ET.Element = field(default_factory=lambda: ET.Element("asset"))
    table_near_edge_m: float = 0.10
    material_coefficients: dict[tuple[str, str], float] | None = None
    material_profile_path: Path | None = None

    def __post_init__(self):
        # Own a validated snapshot and path; later file/env changes cannot alter it.
        path = (task_material_config_path(self.scene, self.task)
                if self.material_profile_path is None else Path(self.material_profile_path))
        coefficients = (load_task_material_coefficients(self.scene, self.task)
                        if self.material_coefficients is None else self.material_coefficients)
        object.__setattr__(self, "material_profile_path", path)
        object.__setattr__(self, "material_coefficients",
                           validate_material_coefficients(coefficients))

    def with_table_near_edge(self, distance: float) -> SceneBuildResult:
        """Translate the table and task together along +X without resampling.

        Distance is measured from the robot base origin to the near table edge.
        The returned result owns independent XML, geometry and sampled metadata.
        """
        distance = float(distance)
        if not math.isfinite(distance) or distance < 0:
            raise ValueError("table near-edge distance must be finite and nonnegative")
        result = deepcopy(self)
        delta = distance - self.table_near_edge_m
        if delta == 0:
            return result
        objects = tuple(
            replace(obj, position=(obj.position[0] + delta, *obj.position[1:]))
            for obj in result.objects
        )
        positions = {obj.name: obj.position for obj in objects}
        for child in result.worldbody:
            if child.tag == "body":
                # Fixtures and painted labels move with their parent bodies.
                position = positions[child.attrib["name"]]
            elif child.tag in {"geom", "light"} and "pos" in child.attrib:
                x, y, z = map(float, child.attrib["pos"].split())
                position = (x + delta, y, z)
            else:
                continue
            child.set("pos", " ".join(map(str, position)))
        table_appearance = result.sampled_values.get("appearance", {}).get("table")
        if table_appearance is not None:
            table_appearance["center_xyz_m"][0] += delta
            table_appearance["near_edge_m"] = distance
        return replace(result, objects=objects, table_near_edge_m=distance)

    def manifest(self) -> dict[str, Any]:
        translation = self.table_near_edge_m - 0.10
        table = self.worldbody.find("geom[@name='scene_table']")
        center = [float(value) for value in table.attrib["pos"].split()]
        size = [2 * float(value) for value in table.attrib["size"].split()]
        return {
            "scene": self.scene,
            "task": self.task,
            "seed": self.seed,
            "geometry_revision": GEOMETRY_REVISION,
            "candidate": self.candidate,
            "table": {
                "top_z_m": center[2] + size[2] / 2,
                "center_xyz_m": center,
                "size_xyz_m": size,
                "near_edge_x_m": self.table_near_edge_m,
                "translation_x_m": translation,
                "workspace_center": [WORKSPACE_CENTER[0] + translation, WORKSPACE_CENTER[1], center[2] + size[2] / 2],
                "x_range_m": [value + translation for value in WORKSPACE_X],
                "y_range_m": list(WORKSPACE_Y),
            },
            "sampled_values": self.sampled_values,
            "objects": [item.manifest() for item in self.objects],
            "physical_materials": {
                **material_manifest(self.material_coefficients),
                "task_profile": {"scene": self.scene, "task": self.task,
                                 "path": str(self.material_profile_path)},
            },
        }

    def xml_string(self) -> str:
        """Return standalone MJCF including the local visual and collision assets."""
        root = ET.Element("mujoco", model=f"{self.scene}_{self.task}")
        ET.SubElement(root, "option", timestep=str(1 / 480), integrator="implicitfast",
                      cone="elliptic", noslip_iterations="1")
        ET.SubElement(root, "size", nuser_geom="2")
        root.extend((deepcopy(self.assets), deepcopy(self.worldbody)))
        apply_material_policy(root, self.objects, self.material_coefficients)
        return ET.tostring(root, encoding="unicode")


def _quat_z(yaw: float) -> tuple[float, float, float, float]:
    return (math.cos(yaw * 0.5), 0.0, 0.0, math.sin(yaw * 0.5))


def _box_geom(name: str, size: tuple[float, float, float], *, pos=(0.0, 0.0, 0.0), rgba=(0.5, 0.5, 0.5, 1.0)) -> dict[str, Any]:
    return {"name": name, "type": "box", "size": tuple(value * 0.5 for value in size), "pos": tuple(pos), "rgba": tuple(rgba)}


def _cylinder_geom(name: str, radius: float, height: float, *, pos=(0.0, 0.0, 0.0), rgba=(0.5, 0.5, 0.5, 1.0)) -> dict[str, Any]:
    return {"name": name, "type": "cylinder", "size": (radius, height * 0.5), "pos": tuple(pos), "rgba": tuple(rgba)}


def _capsule_geom(name: str, radius: float, fromto: tuple[float, ...], *, rgba=(0.5, 0.5, 0.5, 1.0)) -> dict[str, Any]:
    return {"name": name, "type": "capsule", "size": (radius,), "fromto": tuple(fromto), "rgba": tuple(rgba)}


def _vessel_geoms(prefix: str, radius: float, bottom_radius: float, height: float, wall: float, rgba: tuple[float, ...]) -> list[dict[str, Any]]:
    """Overlapping tangent panels form a closed hollow polygonal frustum.

    The panels slope outwards; unlike a convex hull they leave the entire inner
    cavity available for contact. Panel corners remain within z=[0, height].
    """
    geoms = [_cylinder_geom(f"{prefix}_bottom", bottom_radius, wall, pos=(0.0, 0.0, wall * 0.5), rgba=rgba)]
    slope = math.atan2(radius - bottom_radius, height)
    length = (height - wall * math.sin(slope)) / math.cos(slope)
    tangent_width = 2.0 * radius * math.tan(math.pi / VESSEL_SIDES)
    middle_radius = (radius + bottom_radius) * 0.5 - wall * 0.5
    for index in range(VESSEL_SIDES):
        angle = 2.0 * math.pi * index / VESSEL_SIDES
        c, s = math.cos(angle * 0.5), math.sin(angle * 0.5)
        cy, sy = math.cos(slope * 0.5), math.sin(slope * 0.5)
        panel = _box_geom(
            f"{prefix}_wall_{index:02d}", (wall, tangent_width, length),
            pos=(middle_radius * math.cos(angle), middle_radius * math.sin(angle), height * 0.5), rgba=rgba,
        )
        panel["quat"] = (c * cy, -s * sy, c * sy, s * cy)
        geoms.append(panel)
    return geoms


def _geoms_for(class_name: str, size: tuple[float, ...], color: tuple[float, float, float], instance_id: int) -> tuple[dict[str, Any], ...]:
    rgba = (*color, 1.0)
    prefix = f"obj{instance_id}"
    if class_name in {"jenga_block", "letter_block", "domino"}:
        return (_box_geom(f"{prefix}_geom", size, rgba=rgba),)
    if class_name == "plate":
        radius, thickness = size
        base_thickness = 0.003
        inner_radius = radius * 0.72
        geoms = [_cylinder_geom(
            f"{prefix}_well", inner_radius, base_thickness,
            pos=(0, 0, -thickness / 2 + base_thickness / 2), rgba=rgba,
        )]
        rise = thickness - base_thickness
        slope = math.atan2(rise, radius - inner_radius)
        middle_radius = (radius + inner_radius) / 2
        # A shallow sloped ring has an open upper well, not a solid disk hull.
        for index in range(VESSEL_SIDES):
            angle = index * math.tau / VESSEL_SIDES
            half = angle / 2
            cy, sy = math.cos(-slope / 2), math.sin(-slope / 2)
            panel = _box_geom(
                f"{prefix}_rim_{index:02d}",
                (math.hypot(radius - inner_radius, rise),
                 2 * radius * math.tan(math.pi / VESSEL_SIDES),
                 base_thickness * math.cos(slope)),
                pos=(middle_radius * math.cos(angle), middle_radius * math.sin(angle), 0),
                rgba=rgba,
            )
            panel["quat"] = (math.cos(half) * cy, -math.sin(half) * sy,
                             math.cos(half) * sy, math.sin(half) * cy)
            geoms.append(panel)
        return tuple(geoms)
    if class_name in {"cup", "mug"}:
        radius, height, wall = size[:3]
        bottom_radius = size[3] if class_name == "cup" else radius
        geoms = _vessel_geoms(prefix, radius, bottom_radius, height, wall, rgba)
        if class_name == "cup":
            # Three internal feet carry the next cup; tapered walls remain clear.
            nest_step = size[4]
            for index in range(3):
                angle = index * 2.0 * math.pi / 3.0
                geoms.append(_cylinder_geom(
                    f"{prefix}_nest_stop_{index}", wall, nest_step - wall,
                    pos=(bottom_radius * 0.82 * math.cos(angle), bottom_radius * 0.82 * math.sin(angle), (nest_step + wall) * 0.5), rgba=rgba,
                ))
        else:
            # Closed oval handle in the x-z plane: about 26 x 42 mm clear.
            # Its inner edge joins the mug wall; no bar crosses the opening.
            center_x, center_z = radius + 0.015, height * 0.55
            for index in range(32):
                a, b = index * math.tau / 32, (index + 1) * math.tau / 32
                geoms.append(_capsule_geom(
                    f"{prefix}_handle_{index:02d}", 0.004,
                    (center_x + 0.019 * math.cos(a), 0.0, center_z + 0.025 * math.sin(a),
                     center_x + 0.019 * math.cos(b), 0.0, center_z + 0.025 * math.sin(b)), rgba=rgba,
                ))
        return tuple(geoms)
    if class_name == "rack":
        width, depth, height = size
        geoms = []
        for side in (-1, 1):
            geoms.append(_box_geom(f"{prefix}_rail_{side}", (width, 0.018, 0.016), pos=(0.0, side * 0.065, 0.008), rgba=rgba))
        # Four pairs of uprights yield three unobstructed 50 mm plate slots.
        for index, x in enumerate((-0.090, -0.030, 0.030, 0.090)):
            for side in (-1, 1):
                geoms.append(_capsule_geom(f"{prefix}_peg_{index}_{side}", 0.005,
                    (x, side * 0.065, 0.016, x, side * 0.065, height - 0.005), rgba=rgba))
        for side in (-1, 1):
            geoms.append(_box_geom(f"{prefix}_crossbar_{side}", (0.014, depth, 0.012), pos=(side * (width * 0.5 - 0.007), 0.0, 0.006), rgba=rgba))
        return tuple(geoms)
    if class_name == "mug_tree":
        geoms = [
            _cylinder_geom(f"{prefix}_base", 0.070, 0.018, pos=(0.0, 0.0, 0.009), rgba=rgba),
            _capsule_geom(f"{prefix}_trunk", 0.010, (0.0, 0.0, 0.018, 0.0, 0.0, 0.290), rgba=rgba),
        ]
        # Tangential pegs cross the handle opening perpendicular to its plane.
        # The radial branches stay behind the ring instead of piercing its rim.
        for index, angle in enumerate((0.0, math.pi, math.pi * 0.5, -math.pi * 0.5)):
            z = 0.185 if index < 2 else 0.245
            radial = np.array((math.cos(angle), math.sin(angle)))
            tangent = np.array((-math.sin(angle), math.cos(angle)))
            start, end = 0.085 * radial - 0.028 * tangent, 0.085 * radial + 0.028 * tangent
            geoms.append(_capsule_geom(f"{prefix}_branch_{index}", 0.005,
                                      (0.0, 0.0, z - 0.035, *start, z), rgba=rgba))
            geoms.append(_capsule_geom(f"{prefix}_hook_{index}", 0.004,
                                      (*start, z, *end, z), rgba=rgba))
            geoms.append(_capsule_geom(f"{prefix}_tip_{index}", 0.004,
                                      (*end, z, *end, z + 0.014), rgba=rgba))
        return tuple(geoms)
    if class_name == "bin":
        width, depth, height = size[:3]
        wall = 0.008
        return (
            _box_geom(f"{prefix}_bottom", (width + 2 * wall, depth + 2 * wall, wall), pos=(0.0, 0.0, wall * 0.5), rgba=rgba),
            _box_geom(f"{prefix}_x1", (wall, depth + 2 * wall, height), pos=((width + wall) * 0.5, 0.0, wall + height * 0.5), rgba=rgba),
            _box_geom(f"{prefix}_x2", (wall, depth + 2 * wall, height), pos=(-(width + wall) * 0.5, 0.0, wall + height * 0.5), rgba=rgba),
            _box_geom(f"{prefix}_y1", (width, wall, height), pos=(0.0, (depth + wall) * 0.5, wall + height * 0.5), rgba=rgba),
            _box_geom(f"{prefix}_y2", (width, wall, height), pos=(0.0, -(depth + wall) * 0.5, wall + height * 0.5), rgba=rgba),
        )
    if class_name == "cabinet":
        depth, width, height = size
        wall = 0.010
        geoms = [
            _box_geom(f"{prefix}_back", (wall, width, height), pos=((depth - wall) / 2, 0, height / 2), rgba=rgba),
            _box_geom(f"{prefix}_top", (depth, width, wall), pos=(0, 0, height - wall / 2), rgba=rgba),
        ]
        for side in (-1, 1):
            geoms.append(_box_geom(f"{prefix}_side_{side}", (depth, wall, height),
                                  pos=(0, side * (width - wall) / 2, height / 2), rgba=rgba))
        # The slide guides carry drawer weight. A 1 mm running gap prevents
        # redundant, immovable normal contacts between shelf and guided tray.
        for level, floor in enumerate(DRAWER_FLOORS):
            geoms.append(_box_geom(f"{prefix}_shelf_{level}", (depth, width - 2 * wall, wall),
                                  pos=(0, 0, floor - 0.001 - wall / 2), rgba=rgba))
        # Close the underside without filling the open-front drawer bays.
        geoms.append(_box_geom(f"{prefix}_bottom", (depth, width, 0.008), pos=(0, 0, 0.004), rgba=rgba))
        return tuple(geoms)
    if class_name == "drawer":
        depth, width, height = size
        wall = 0.006
        geoms = [_box_geom(f"{prefix}_floor", (depth, width, wall), pos=(0, 0, wall / 2), rgba=rgba)]
        for side in (-1, 1):
            geoms.append(_box_geom(f"{prefix}_side_{side}", (depth, wall, height - wall),
                                  pos=(0, side * (width - wall) / 2, (height + wall) / 2), rgba=rgba))
            geoms.append(_box_geom(f"{prefix}_end_{side}", (wall, width - 2 * wall, height - wall),
                                  pos=(side * (depth - wall) / 2, 0, (height + wall) / 2), rgba=rgba))
        handle_x, handle_z = -depth / 2 - 0.032, height * 0.62
        geoms.append(_capsule_geom(f"{prefix}_handle_grip", 0.004,
                                  (handle_x, -0.054, handle_z, handle_x, 0.054, handle_z), rgba=rgba))
        for side in (-1, 1):
            geoms.append(_capsule_geom(f"{prefix}_handle_mount_{side}", 0.004,
                                      (-depth / 2, side * 0.054, handle_z, handle_x, side * 0.054, handle_z), rgba=rgba))
        return tuple(geoms)
    raise ValueError(f"unknown procedural class: {class_name}")


def _geom_volume(geom: dict[str, Any]) -> float:
    if geom["type"] == "mesh":
        volume = float(geom["volume"])
        if not math.isfinite(volume) or volume <= 0:
            raise SceneResetError("collision mesh must have a finite positive volume")
        return volume
    size = geom["size"]
    if geom["type"] == "box":
        return 8.0 * math.prod(size)
    if geom["type"] == "cylinder":
        return math.pi * size[0] ** 2 * 2.0 * size[1]
    start, end = geom["fromto"][:3], geom["fromto"][3:]
    return math.pi * size[0] ** 2 * math.dist(start, end) + 4.0 / 3.0 * math.pi * size[0] ** 3


class ProceduralSceneBuilder:
    """Build one scene/task with all random values in its manifest."""

    def __init__(self, scene: str, task: str, seed: int) -> None:
        self.scene = scene
        self.task = task
        self.seed = int(seed)
        self.target_word = str(np.random.default_rng(self.seed).choice(SPELLING_WORDS))
        self.layout_values: dict[str, Any] = {}

    def _layout(self) -> list[tuple[str, tuple[float, float, float], bool]]:
        scene, task = self.scene, self.task
        if scene == "jenga":
            if task == "playing":
                # 18 alternating three-block layers, all blocks individually free.
                return [("jenga_block", (
                    0.45 + ((i % 3 - 1) * 0.0252 if (i // 3) % 2 else 0.0),
                    (i % 3 - 1) * 0.0252 if not (i // 3) % 2 else 0.0,
                    TABLE_Z + (i // 3 + 0.5) * JENGA_BLOCK_SIZE[2]), True) for i in range(54)]
            if task in {"hollow_tower", "tower", "criss_cross"}:
                # Construction tasks start with supported loose blocks, not
                # floating layers that collapse before the operator can act.
                return [("jenga_block", (0.24 + (i % 3) * 0.18, -0.15 + (i // 3) * 0.15, TABLE_Z + JENGA_BLOCK_SIZE[2] * 0.5), False) for i in range(9)]
            if task == "dominos":
                return [("domino", (0.21 + (i % 3) * 0.18, -0.16 + (i // 3) * 0.15, TABLE_Z + DOMINO_SIZE[2] / 2), False) for i in range(9)]
            if task in {"handover_lr", "handover_rl"}:
                return [("jenga_block", (0.45, 0.0, TABLE_Z + JENGA_BLOCK_SIZE[2] * 0.5), False)]
        if scene == "spelling_blocks":
            count = {"spelling": len(self.target_word), "sort_and_unload": 8,
                     "pyramid": 6, "vowel_consonant_sort": 10}[task]
            openings = (0.245, 0.165, 0.085) if task == "sort_and_unload" else (0.0, 0.0, 0.0)
            layout = [("cabinet", (0.66, 0.0, TABLE_Z), True)]
            layout.extend(("drawer", (0.66 - opening, 0.0, TABLE_Z + floor), True)
                          for opening, floor in zip(openings, DRAWER_FLOORS, strict=True))
            if task == "sort_and_unload":
                layout.extend(("letter_block",
                               (0.66 - openings[i // 3] - 0.075, (i % 3 - 1) * 0.080,
                                TABLE_Z + DRAWER_FLOORS[i // 3] + 0.006 + LETTER_BLOCK_SIZE[2] / 2),
                               True) for i in range(count))
            else:
                layout.extend(("letter_block", (0.23 + (i % 3) * 0.105, -0.23 + (i // 3) * 0.130,
                                                TABLE_Z + LETTER_BLOCK_SIZE[2] / 2), False) for i in range(count))
            return layout
        if scene == "mugs" and task == "hang_mug":
            return [("mug", (0.30, -0.13, TABLE_Z), False), ("mug_tree", (0.52, 0.12, TABLE_Z), True)]
        if scene == "dishes":
            if task == "rack_dishes":
                return [("plate", (0.27, -0.13 + i * 0.26, TABLE_Z + PLATE_THICKNESS * 0.5), False) for i in range(2)] + [("rack", (0.60, 0.02, TABLE_Z), True)]
            if task == "plate_dishes":
                return [("plate", (0.25 + (i % 2) * 0.30, -0.25 + (i // 2) * 0.30, TABLE_Z + PLATE_THICKNESS * 0.5), False) for i in range(4)]
        if scene == "cups":
            if task == "pyramid":
                return [("cup", (0.38, -0.08, TABLE_Z + i * CUP_NEST_STEP), True) for i in range(6)]
            if task == "stack_two_threes":
                return [("cup", (0.32 + (i // 3) * 0.22, 0.0, TABLE_Z + (i % 3) * CUP_NEST_STEP), True) for i in range(6)]
            if task == "unstack":
                return [("cup", (0.45, 0.0, TABLE_Z + i * CUP_NEST_STEP), True) for i in range(3)]
        if scene == "bottles" and task == "toss_in_bin":
            return [
                ("bottle", (0.22, -0.20, TABLE_Z), False),
                ("bottle", (0.22, 0.20, TABLE_Z), False),
                ("bin", (0.59, 0.0, TABLE_Z), True),
            ]
        raise KeyError(f"unknown SPD task: {scene}/{task}")

    @staticmethod
    def _size(class_name: str) -> tuple[float, ...]:
        return {
            "jenga_block": JENGA_BLOCK_SIZE,
            "domino": DOMINO_SIZE,
            "letter_block": LETTER_BLOCK_SIZE,
            "plate": (PLATE_RADIUS, PLATE_THICKNESS),
            "cup": CUP_PROFILES[0],
            "mug": (0.040, 0.090, 0.003),
            "bottle": (BOTTLE_RADIUS, BOTTLE_HEIGHT),
            "rack": (0.24, 0.17, 0.14),
            "mug_tree": (0.18, 0.18, 0.30),
            "bin": BIN_INNER_SIZE,
            "cabinet": CABINET_SIZE,
            "drawer": DRAWER_SIZE,
        }[class_name]

    def _sample_candidate(self, rng: np.random.Generator, candidate: int, table_top_z: float,
                          table_distance: float) -> tuple[ObjectSpec, ...]:
        objects: list[ObjectSpec] = []
        # Whole-layout transforms preserve every mechanical assembly. Independent
        # loose-object scatter stays bounded and passes the actual contact gate.
        layout_variant = int(rng.integers(3))
        assembly_yaw = float(rng.uniform(-0.16, 0.16))
        translation = rng.uniform(-0.015, 0.015, size=2)
        mirror_y = bool(rng.integers(2)) and self.scene != "spelling_blocks"
        scatter = (0.010, 0.020, 0.028)[layout_variant]
        cup_profile = CUP_PROFILES[int(rng.integers(len(CUP_PROFILES)))]
        rotation = np.array(((math.cos(assembly_yaw), -math.sin(assembly_yaw)),
                             (math.sin(assembly_yaw), math.cos(assembly_yaw))))
        self.layout_values = {
            "arrangement": ("grid", "staggered", "scattered")[layout_variant],
            "assembly_yaw_rad": assembly_yaw, "assembly_translation_xy_m": translation.tolist(),
            "mirrored_y": mirror_y, "loose_scatter_limit_m": scatter,
        }
        drawer_index = 0
        reference_volumes: dict[str, float] = {}
        for instance_id, (class_name, base_position, assembled) in enumerate(self._layout(), start=1):
            size = self._size(class_name)
            if class_name == "cup":
                size = cup_profile
            elif class_name in {"mug", "plate", "bin"}:
                profiles = {"mug": MUG_PROFILES, "plate": PLATE_PROFILES, "bin": BIN_PROFILES}[class_name]
                size = profiles[int(rng.integers(len(profiles)))]
            asset_id = f"procedural/{class_name}/{GEOMETRY_REVISION}"
            asset_definitions, visual_geoms, provenance = (), (), {}
            if class_name == "bottle":
                asset_id = str(rng.choice(BOTTLE_VARIANTS))
                size = (size[0] * float(rng.uniform(0.92, 1.08)),
                        size[1] * float(rng.uniform(0.90, 1.12)))
            elif class_name in {"cup", "mug", "plate", "bin"}:
                asset_id += "/" + "x".join(f"{value:g}" for value in size)
            xy = np.array(base_position[:2], dtype=float)
            if mirror_y:
                xy[1] *= -1
            if not assembled:
                xy += rng.uniform(-scatter, scatter, size=2)
                if layout_variant == 1:
                    xy[1] += 0.020 if instance_id % 2 else -0.020
            xy = rotation @ (xy - (0.45, 0.0)) + (0.45, 0.0) + translation
            if self.scene == "bottles" and self.task == "toss_in_bin" and class_name == "bin":
                # The table shifts after collision validation; sample the bin in world X.
                xy[0] = rng.uniform(*BIN_X_RANGE) - (table_distance - TABLE_DISTANCE_RANGE[0])
            yaw = assembly_yaw
            if not assembled:
                yaw += float(rng.uniform(-math.pi, math.pi))
            if self.scene == "jenga" and self.task == "playing":
                yaw += ((instance_id - 1) // 3 % 2) * math.pi * 0.5
            z = base_position[2] + table_top_z - TABLE_Z
            if class_name == "plate":
                z += (size[1] - PLATE_THICKNESS) / 2
            elif class_name == "cup":
                layer = round((base_position[2] - TABLE_Z) / CUP_NEST_STEP)
                z = table_top_z + layer * size[4]
            position = (float(xy[0]), float(xy[1]), float(z))
            if not (WORKSPACE_X[0] <= position[0] <= WORKSPACE_X[1] and WORKSPACE_Y[0] <= position[1] <= WORKSPACE_Y[1]):
                raise SceneResetError(f"object {instance_id} leaves workspace")
            friction = float(rng.uniform(*_MATERIALS[class_name][1]))
            appearance_variant = int(rng.integers(appearance_count(class_name)))
            palette = _PALETTES[_MATERIALS[class_name][0]]
            color = tuple(float(value) for value in palette[appearance_variant % len(palette)])
            if class_name == "bottle":
                assets, geoms, visual_geoms, provenance = bottle_geometry(asset_id, size, instance_id)
                asset_definitions = tuple({"tag": element.tag, "attributes": dict(element.attrib)} for element in assets)
                nominal_mass = BASE_MASSES[class_name] * (size[0] / BOTTLE_RADIUS) ** 2 * size[1] / BOTTLE_HEIGHT
            else:
                geoms = _geoms_for(class_name, size, color, instance_id)
                nominal_mass = BASE_MASSES[class_name]
                if class_name in {"cup", "mug", "plate", "bin", "cabinet", "drawer"}:
                    volume = sum(_geom_volume(geom) for geom in geoms)
                    if class_name in {"cabinet", "drawer"}:
                        nominal_mass = 650.0 * volume
                    else:
                        if class_name not in reference_volumes:
                            reference_volumes[class_name] = sum(
                                _geom_volume(geom) for geom in _geoms_for(class_name, self._size(class_name), color, 0))
                        nominal_mass *= volume / reference_volumes[class_name]
            joint: dict[str, Any] = {}
            if class_name == "drawer":
                opening = (0.245, 0.165, 0.085)[drawer_index] if self.task == "sort_and_unload" else 0.0
                joint = {"type": "slide", "axis": (-1.0, 0.0, 0.0),
                         "range": (0.0, DRAWER_TRAVEL), "ref": opening,
                         "limited": "true", "damping": 2.0, "frictionloss": 0.3}
                drawer_index += 1
            objects.append(ObjectSpec(
                instance_id=instance_id, class_id=CLASS_IDS[class_name], class_name=class_name,
                name=f"{self.scene}_{self.task}_object_{instance_id:03d}", position=position,
                yaw_rad=yaw, size=tuple(float(value) for value in size),
                mass_kg=nominal_mass * float(rng.uniform(0.8, 1.2)),
                friction=friction, contact_group="hand_object", assembled=assembled,
                color_rgb=color, geoms=geoms, asset_id=asset_id,
                appearance_variant=appearance_variant, visual_geoms=visual_geoms,
                asset_definitions=asset_definitions, asset_provenance=provenance, joint=joint,
            ))
        return tuple(objects)

    @staticmethod
    def _worldbody(objects: Iterable[ObjectSpec], table_top_z: float) -> ET.Element:
        worldbody = ET.Element("worldbody")
        ET.SubElement(worldbody, "light", name="scene_key_light", directional="true",
                      pos="0.20 -0.40 1.90", dir="0.20 0.25 -1",
                      ambient="0.03 0.03 0.03", diffuse="0.38 0.37 0.35",
                      specular="0.10 0.10 0.10")
        ET.SubElement(worldbody, "light", name="scene_fill_light", directional="true",
                      pos="0.80 0.40 1.50", dir="-0.20 -0.30 -1",
                      diffuse="0.12 0.14 0.16", specular="0.03 0.03 0.03", castshadow="false")
        ET.SubElement(worldbody, "geom", name="scene_visual_floor", type="plane",
                      pos="0 0 -0.002", size="3 3 .01", group="2", mass="0",
                      contype="0", conaffinity="0", rgba="0.26 0.29 0.30 1", user="0 0")
        # Near edge x=0.10 clears the base column (x=0.0825 at tabletop height).
        ET.SubElement(worldbody, "geom", name="scene_table", type="box", pos=f"0.50 0 {table_top_z - 0.025:.12g}", size="0.40 0.55 0.025", contype="1", conaffinity="1", group="3", rgba="0.30 0.26 0.22 1")
        for obj in objects:
            body = ET.SubElement(worldbody, "body", name=obj.name,
                pos=" ".join(f"{value:.12g}" for value in obj.position),
                quat=" ".join(f"{value:.12g}" for value in _quat_z(obj.yaw_rad)))
            if obj.joint:
                ET.SubElement(body, "joint", name=f"{obj.name}_slide", **_geom_attributes(obj.joint))
            elif obj.class_name not in FIXTURE_CLASSES:
                ET.SubElement(body, "joint", name=f"{obj.name}_free", type="free", damping="0.002")
            # MuJoCo integrates the actual compound geometry to obtain the COM
            # and full inertia tensor, including the offset mug handle.
            volumes = tuple(_geom_volume(geom) for geom in obj.geoms)
            total_volume = sum(volumes)
            for geom, volume in zip(obj.geoms, volumes):
                attributes = _geom_attributes(geom)
                attributes.update(
                    contype="1", conaffinity="1", group="3",
                    user=f"{obj.instance_id} {obj.class_id}",
                    mass=f"{obj.mass_kg * volume / total_volume:.12g}",
                    friction=f"{obj.friction:.12g} 0.005 0.0001",
                    solref="0.004 1", solimp="0.95 0.99 0.001", priority="1",
                )
                ET.SubElement(body, "geom", **attributes)
        return worldbody

    def build(self) -> SceneBuildResult:
        import mujoco

        material_profile_path = task_material_config_path(self.scene, self.task)
        material_coefficients = load_task_material_coefficients(self.scene, self.task)
        rng = np.random.default_rng(self.seed)
        table_top_z = float(rng.uniform(*TABLE_HEIGHT_RANGE))
        table_distance = float(rng.uniform(*TABLE_DISTANCE_RANGE))
        last_error: Exception | None = None
        for candidate in range(MAX_RESET_CANDIDATES):
            try:
                objects = self._sample_candidate(rng, candidate, table_top_z, table_distance)
                worldbody = self._worldbody(objects, table_top_z)
                labels = _letters_for(objects, self.seed, self.task, self.target_word)
                assets, visuals, table_visuals, appearance = build_visual_details(
                    objects, self.seed, labels, table_top_z=table_top_z,
                )
                ET.SubElement(assets, "texture", name="scene_sky", type="skybox", builtin="gradient",
                              rgb1="0.64 0.71 0.76", rgb2="0.93 0.94 0.92", width="256", height="1536")
                for obj in objects:
                    for definition in obj.asset_definitions:
                        ET.SubElement(assets, definition["tag"], **definition["attributes"])
                    body = worldbody.find(f"body[@name='{obj.name}']")
                    for geom in (*visuals.get(obj.name, ()), *obj.visual_geoms):
                        attributes = _geom_attributes(geom)
                        attributes.update(contype="0", conaffinity="0", mass="0", group="2",
                                          user=f"{obj.instance_id} {obj.class_id}")
                        ET.SubElement(body, "geom", **attributes)
                for geom in table_visuals:
                    attributes = _geom_attributes(geom)
                    attributes.update(contype="0", conaffinity="0", mass="0", group="2", user="0 0")
                    ET.SubElement(worldbody, "geom", **attributes)
                # Test the actual contact geometry, including hollow interiors.
                # Bounding boxes cannot distinguish valid nesting from overlap.
                root = ET.Element("mujoco")
                ET.SubElement(root, "option", timestep=str(1 / 480), integrator="implicitfast",
                              cone="elliptic", noslip_iterations="1")
                ET.SubElement(root, "size", nuser_geom="2")
                root.append(assets)
                root.append(worldbody)
                apply_material_policy(root, objects, material_coefficients)
                model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
                data = mujoco.MjData(model)
                contact_gate(model, data, {item.name for item in objects})
                table_clearances = _table_clearances(model, data, objects)
                values = {
                    "geometry_revision": GEOMETRY_REVISION,
                    "appearance": appearance,
                    "table_top_z_m": table_top_z,
                    "table_height_range_m": list(TABLE_HEIGHT_RANGE),
                    "table_distance_range_m": list(TABLE_DISTANCE_RANGE),
                    "object_asset_ids": {str(item.instance_id): item.asset_id for item in objects},
                    "object_appearance_variants": {str(item.instance_id): item.appearance_variant for item in objects},
                    "bottle_scale_ranges": {"radius": [0.92, 1.08], "height": [0.90, 1.12]},
                    "mass_multiplier_range": [0.8, 1.2],
                    "friction_range_by_class": {
                        item.class_name: list(_MATERIALS.get(item.class_name, ("", (0.6, 1.2)))[1])
                        for item in objects
                    },
                    "layout": self.layout_values,
                    "loose_yaw_range_rad": [-math.pi, math.pi],
                    "table_edge_clearance_m": table_clearances,
                    "affordances": _affordances(objects),
                    "candidate": candidate,
                    "dimension_source": "engineering dimensions; Figure 4 / A.4 give object counts and actions, not CAD dimensions",
                    "cup_bottom_radius_m": next((item.size[3] for item in objects if item.class_name == "cup"), None),
                    "cup_nest_step_m": next((item.size[4] for item in objects if item.class_name == "cup"), None),
                    "fixed_fixture_classes": sorted(FIXTURE_CLASSES),
                    "object_sizes_m": {str(item.instance_id): list(item.size) for item in objects},
                    "object_masses_kg": {str(item.instance_id): item.mass_kg for item in objects},
                    "object_friction": {str(item.instance_id): item.friction for item in objects},
                    "collision_debug_colors": {str(item.instance_id): list(item.color_rgb) for item in objects},
                }
                if labels:
                    values["object_letters"] = labels
                if self.scene == "spelling_blocks" and self.task == "spelling":
                    values["target_word"] = self.target_word
                    values["prompt"] = f"Spell {self.target_word} with the letter blocks."
                    values["task_goal_zh"] = f"用字母积木拼出 {self.target_word}。"
                if self.scene == "spelling_blocks" and self.task == "sort_and_unload":
                    values["prompt"] = "Open the drawers, sort the letter blocks, and unload them onto the table."
                    values["task_goal_zh"] = "拉开抽屉，将其中的字母积木分类并取出放到桌上。"
                if self.scene == "jenga" and self.task == "playing":
                    values["extraction_target_instance_id"] = 26  # Centre block of layer 9 (one-based).
                result = SceneBuildResult(self.scene, self.task, self.seed, candidate, objects, values,
                                          worldbody, assets=assets,
                                          material_coefficients=material_coefficients,
                                          material_profile_path=material_profile_path)
                return result.with_table_near_edge(table_distance)
            except SceneResetError as exc:
                last_error = exc
        raise SceneResetError(
            f"scene reset failed after {MAX_RESET_CANDIDATES} candidates for {self.scene}/{self.task} seed={self.seed}: {last_error}"
        )


def _table_clearances(model: Any, data: Any, objects: tuple[ObjectSpec, ...]) -> dict[str, float]:
    """Conservative compiled contact bounds, including both drawer endpoints."""
    import mujoco

    clearances: dict[str, float] = {}
    for obj in objects:
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, obj.name)
        minimum = math.inf
        offsets = (0.0,)
        direction = np.array((-math.cos(obj.yaw_rad), -math.sin(obj.yaw_rad), 0.0))
        if obj.joint:
            offsets = (-obj.joint["ref"], DRAWER_TRAVEL - obj.joint["ref"])
        for geom_id in range(model.ngeom):
            if model.geom_bodyid[geom_id] != body_id or not model.geom_contype[geom_id]:
                continue
            rotation = data.geom_xmat[geom_id].reshape(3, 3)
            local_center, local_half = model.geom_aabb[geom_id, :3], model.geom_aabb[geom_id, 3:]
            center = data.geom_xpos[geom_id] + rotation @ local_center
            half = np.abs(rotation) @ local_half
            for offset in offsets:
                shifted = center + offset * direction
                lower, upper = shifted - half, shifted + half
                minimum = min(minimum, float(lower[0] - 0.10), float(0.90 - upper[0]),
                              float(lower[1] + 0.55), float(0.55 - upper[1]))
        if minimum < 0.002:
            raise SceneResetError(f"{obj.name} contact envelope leaves tabletop ({minimum:.6g} m)")
        clearances[obj.name] = minimum
    return clearances


def _affordances(objects: tuple[ObjectSpec, ...]) -> dict[str, Any]:
    """Dimensions and probe poses in each object's local frame, not CAD claims."""
    result: dict[str, Any] = {}
    drawers = [obj for obj in objects if obj.class_name == "drawer"]
    for obj in objects:
        values: dict[str, Any] = {"coordinate_frame": "object-local; transform by object pose"}
        if obj.class_name == "rack":
            maximum_thickness = max(item.size[1] for item in objects if item.class_name == "plate")
            values.update(slot_centers_xyz_m=[[x, 0.0, 0.016] for x in (-0.060, 0.0, 0.060)],
                          slot_axis_xyz=[1, 0, 0], slot_clear_width_m=0.050,
                          plate_axial_clearance_m=0.050 - maximum_thickness,
                          rail_y_m=[-0.065, 0.065], rail_top_z_m=0.016,
                          plate_insertion_quat_wxyz=[math.sqrt(0.5), 0, math.sqrt(0.5), 0])
        elif obj.class_name == "mug_tree":
            values.update(hook_radius_m=0.004, hook_length_m=0.056, tip_rise_m=0.014,
                          hook_center_xyz_m=[[0.085 * math.cos(a), 0.085 * math.sin(a), 0.185 if i < 2 else 0.245]
                                             for i, a in enumerate((0, math.pi, math.pi / 2, -math.pi / 2))],
                          hook_radial_angles_rad=[0, math.pi, math.pi / 2, -math.pi / 2],
                          hanging_mug_yaw_rule="tree yaw + hook radial angle + pi",
                          hanging_mug_center_radius_rule_m="0.085 + mug.radius + 0.015",
                          hanging_mug_bottom_z_rule_m="hook_center_z + 0.004 - 0.021 - 0.55*mug.height",
                          hook_tip_xyz_m=[list(geom["fromto"][3:]) for geom in obj.geoms if "_tip_" in geom["name"]])
        elif obj.class_name == "mug":
            values.update(handle_center_xyz_m=[obj.size[0] + 0.015, 0, obj.size[1] * 0.55],
                          handle_inner_width_m=0.030, handle_inner_height_m=0.042,
                          handle_tube_radius_m=0.004, hook_diameter_clearance_m=0.022)
        elif obj.class_name == "bin":
            values.update(inner_size_xyz_m=list(obj.size[:3]), mouth_size_xy_m=list(obj.size[:2]),
                          interior_floor_z_m=0.008, rim_top_z_m=0.008 + obj.size[2],
                          inner_volume_m3=math.prod(obj.size[:3]), wall_thickness_m=0.008,
                          bottle_diameter_clearance_m=min(obj.size[:2]) -
                          max(2 * item.size[0] for item in objects if item.class_name == "bottle"))
        elif obj.class_name == "cabinet":
            values.update(front_x_m=-obj.size[0] / 2, drawer_floor_z_m=list(DRAWER_FLOORS),
                          drawer_side_clearance_m=0.004, drawer_back_clearance_m=0.001,
                          shelf_top_z_m=[floor - 0.001 for floor in DRAWER_FLOORS],
                          drawer_running_clearance_m=0.001,
                          drawer_bay_vertical_clearance_m=[0.040, 0.040, 0.041], open_front=True)
        elif obj.class_name == "drawer":
            level = drawers.index(obj)
            contained = [item.instance_id for item in objects if item.class_name == "letter_block"
                         and item.assembled and abs(item.position[2] - obj.position[2] - 0.026) < 1e-7]
            values.update(joint_name=f"{obj.name}_slide", qpos_units="metres outward from closed",
                          joint_axis_local_xyz=[-1, 0, 0], travel_range_m=[0.0, DRAWER_TRAVEL],
                          initial_opening_m=obj.joint["ref"], qpos0_m=obj.joint["ref"],
                          interior_size_xyz_m=[obj.size[0] - 0.012, obj.size[1] - 0.012, obj.size[2] - 0.006],
                          interior_floor_z_m=0.006, cabinet_floor_z_m=DRAWER_FLOORS[level],
                          handle_center_xyz_m=[-obj.size[0] / 2 - 0.032, 0, obj.size[2] * 0.62],
                          handle_finger_gap_m=0.028, handle_clear_span_m=0.100,
                          contained_instance_ids=contained)
        elif obj.class_name == "cup":
            radius, height, wall, bottom, step = obj.size
            values.update(mouth_inner_radius_m=radius - wall, bottom_outer_radius_m=bottom,
                          nest_step_m=step, nest_stop_top_z_m=step,
                          nested_wall_normal_clearance_m=(radius - bottom) * step / math.hypot(height, radius - bottom) - wall)
        else:
            continue
        result[str(obj.instance_id)] = values
    return result


def contact_gate(model: Any, data: Any, object_body_names: set[str]) -> None:
    """Reject object/object and object/table penetration after ``mj_forward``.

    There are no assembled-object exemptions: nested cups and towers must pass
    the same real collision check as loose objects.
    """
    import mujoco

    mujoco.mj_forward(model, data)
    for index in range(int(data.ncon)):
        contact = data.contact[index]
        if contact.dist >= -1e-7:
            continue
        first = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[contact.geom1]))
        second = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[contact.geom2]))
        first_geom = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(contact.geom1))
        second_geom = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(contact.geom2))
        if ((first in object_body_names and (second in object_body_names or second_geom == "scene_table"))
                or (second in object_body_names and first_geom == "scene_table")):
            raise SceneResetError(f"initial interpenetration: {first_geom} vs {second_geom} ({contact.dist:.6g} m)")


__all__ = [
    "BASE_MASSES", "BIN_INNER_SIZE", "CLASS_IDS", "CUP_BOTTOM_RADIUS", "CUP_HEIGHT",
    "CUP_NEST_STEP", "CUP_OUTER_RADIUS", "CUP_WALL", "JENGA_BLOCK_SIZE",
    "LETTER_BLOCK_SIZE", "MAX_RESET_CANDIDATES", "ObjectSpec", "PLATE_RADIUS",
    "PLATE_THICKNESS", "ProceduralSceneBuilder", "SceneBuildResult", "SceneResetError",
    "TABLE_Z", "WORKSPACE_CENTER", "WORKSPACE_X", "WORKSPACE_Y", "contact_gate",
]
