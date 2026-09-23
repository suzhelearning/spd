"""Original, portable visual surfaces, independent of SPD contact geometry.

All coordinates are metres in the owning body's frame, except table geometry
which is world-relative at the default table placement. Meshes are deliberately
render-only: concave vessel and handle surfaces must never become contact hulls.
"""
from __future__ import annotations

from functools import lru_cache
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Sequence
import xml.etree.ElementTree as ET

import numpy as np

GEOMETRY_REVISION = "surfaces-v2"
_TEXTURES = Path(__file__).resolve().parent / "assets" / "detail_textures"
_PALETTES = {
    "cup": ((.96, .06, .045), (.045, .76, .14), (.035, .24, .97), (1.0, .83, .015)),
    "mug": ((0.87, 0.88, 0.80), (0.40, 0.57, 0.53), (0.43, 0.51, 0.62), (0.74, 0.47, 0.35)),
    "plate": ((0.94, 0.92, 0.84), (0.82, 0.88, 0.83), (0.84, 0.87, 0.91)),
    "bin": ((0.27, 0.37, 0.39), (0.42, 0.48, 0.40), (0.55, 0.43, 0.33)),
}
_WOOD_CLASSES = frozenset({"jenga_block", "letter_block", "domino", "rack", "mug_tree", "cabinet", "drawer"})
_WOOD_VARIANTS = 3


def appearance_count(class_name: str) -> int:
    """Number of selectable appearances; palette definitions are authoritative."""
    if class_name in _PALETTES:
        return len(_PALETTES[class_name])
    if class_name in _WOOD_CLASSES:
        return _WOOD_VARIANTS
    if class_name == "bottle":
        return 1  # The imported bottle supplies its own appearance.
    raise ValueError(f"no appearance palette for {class_name!r}")


def _numbers(values: Any) -> str:
    return " ".join(format(float(value), ".9g") for value in np.asarray(values).flat)


def _geom(name: str, instance: int, class_id: int, kind: str, **attrs: Any) -> dict[str, Any]:
    return {"name": name, "type": kind, "mass": "0", "contype": "0",
            "conaffinity": "0", "group": "2", "user": f"{instance} {class_id}", **attrs}


def _material(asset: ET.Element, name: str, rgb: Sequence[float], *, texture: str | None = None,
              specular: float = .22, shininess: float = .22) -> str:
    attrs = {"name": name, "rgba": _numbers((*rgb, 1)), "specular": str(specular),
             "shininess": str(shininess)}
    if texture is not None:
        attrs.update(texture=texture, texuniform="false")
    ET.SubElement(asset, "material", attrs)
    return name


def _mesh(asset: ET.Element, name: str, vertices: Any, faces: Any,
          uv: Any | None = None, normals: Any | None = None) -> str:
    attrs = {"name": name, "vertex": _numbers(vertices),
             "face": " ".join(str(int(index)) for index in np.asarray(faces).flat)}
    if uv is not None:
        attrs["texcoord"] = _numbers(uv)
    if normals is not None:
        attrs["normal"] = _numbers(normals)
    ET.SubElement(asset, "mesh", attrs)
    return name


def _rounded_box(asset: ET.Element, name: str, size: Sequence[float], bevel: float) -> str:
    """Six stitched rounded patches with analytic normals and per-face UVs."""
    half = np.asarray(size) * .5
    bevel = min(bevel, float(half.min()) * .35)
    vertices, normals, uv, faces = [], [], [], []
    for axis in range(3):
        u_axis, v_axis = (axis + 1) % 3, (axis + 2) % 3
        for sign in (-1, 1):
            def coordinates(h: float) -> tuple[float, ...]:
                return (-h, -h + bevel * .3, -h + bevel, h - bevel, h - bevel * .3, h)
            u_values, v_values = coordinates(half[u_axis]), coordinates(half[v_axis])
            start = len(vertices)
            for v in v_values:
                for u in u_values:
                    point = np.zeros(3)
                    point[axis], point[u_axis], point[v_axis] = sign * half[axis], u, v
                    core = np.clip(point, -half + bevel, half - bevel)
                    normal = point - core
                    normal /= np.linalg.norm(normal)
                    vertices.append(core + bevel * normal)
                    normals.append(normal)
                    # PNG rows run downward; MuJoCo's texture V runs upward.
                    uv.append((u / size[u_axis] + .5, .5 - v / size[v_axis]))
            for j in range(5):
                for i in range(5):
                    a = start + j * 6 + i
                    triangles = ((a, a + 1, a + 7), (a, a + 7, a + 6))
                    faces.extend(triangles if sign > 0 else tuple(tuple(reversed(t)) for t in triangles))
    return _mesh(asset, name, vertices, faces, uv, normals)


def _lathe(asset: ET.Element, name: str, profile: Sequence[tuple[float, float]],
           segments: int = 80) -> str:
    """Revolve a CCW radial/height boundary, retaining true open cavities.

    Outer walls run upwards and inner walls downwards. Axis vertices become
    cap fans, rather than degenerate triangles, and all faces point out of the
    material (therefore inward into a vessel's air cavity).
    """
    vertices, uv, faces, rings = [], [], [], []
    distances = [0.0]
    for a, b in zip(profile, profile[1:]):
        distances.append(distances[-1] + math.dist(a, b))
    total = distances[-1] + math.dist(profile[-1], profile[0])
    for index, (radius, z) in enumerate(profile):
        ring = []
        for j in range(1 if radius == 0 else segments + 1):
            angle = math.tau * j / segments
            ring.append(len(vertices))
            vertices.append((radius * math.cos(angle), radius * math.sin(angle), z))
            uv.append((j / segments, distances[index] / total))
        rings.append(ring)
    for index, first in enumerate(rings):
        second = rings[(index + 1) % len(rings)]
        if len(first) == len(second) == 1:
            continue
        for j in range(segments):
            if len(first) == 1:
                faces.append((first[0], second[j + 1], second[j]))
            elif len(second) == 1:
                faces.append((first[j], first[j + 1], second[0]))
            else:
                faces.extend(((first[j], first[j + 1], second[j + 1]),
                              (first[j], second[j + 1], second[j])))
    return _mesh(asset, name, vertices, faces, uv)


def _oval_tube(asset: ET.Element, name: str, center_x: float, center_z: float,
               rx: float, rz: float, radius: float) -> str:
    vertices, normals, uv, faces = [], [], [], []
    for i in range(81):
        angle = math.tau * i / 80
        center = np.asarray((center_x + rx * math.cos(angle), 0, center_z + rz * math.sin(angle)))
        outward = np.asarray((rz * math.cos(angle), 0, rx * math.sin(angle)))
        outward /= np.linalg.norm(outward)
        for j in range(13):
            phi = math.tau * j / 12
            normal = math.cos(phi) * outward + np.asarray((0, math.sin(phi), 0))
            vertices.append(center + radius * normal)
            normals.append(normal)
            uv.append((i / 80, j / 12))
    for i in range(80):
        for j in range(12):
            a = i * 13 + j
            faces.extend(((a, a + 1, a + 14), (a, a + 14, a + 13)))
    return _mesh(asset, name, vertices, faces, uv, normals)


def _ring(asset: ET.Element, name: str, radius: float, z: float, tube: float) -> str:
    return _lathe(asset, name, tuple((radius + tube * math.cos(a), z + tube * math.sin(a))
                                   for a in np.linspace(0, math.tau, 12, endpoint=False)))


def _texture(asset: ET.Element, filename: str) -> str:
    name = "scene_detail_texture_" + Path(filename).stem
    if asset.find(f"texture[@name='{name}']") is None:
        ET.SubElement(asset, "texture", name=name, type="2d", file=str(_TEXTURES / filename))
    return name


@lru_cache(maxsize=1)
def _provenance() -> dict[str, Any]:
    return {"geometry_revision": GEOMETRY_REVISION,
            "geometry_source_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            "textures": json.loads((_TEXTURES / "manifest.json").read_text())}


def build_visual_details(objects: Sequence[Any], seed: int, letters: dict[str, str], *, table_top_z: float
                         ) -> tuple[ET.Element, dict[str, tuple[dict, ...]], tuple[dict, ...], dict]:
    """Build render-only meshes/materials and record every appearance selection.

    ``letters`` maps object names to uppercase A-Z labels. Object positions,
    collision proxies and mass are never modified; fixture surfaces follow proxies.
    """
    asset = ET.Element("asset")
    result: dict[str, tuple[dict, ...]] = {}
    variants: dict[str, Any] = {}
    for obj in objects:
        if obj.class_name == "bottle":
            continue
        prefix = f"scene_detail_{obj.instance_id}"
        variant = int(obj.appearance_variant)
        # Seeded choices are independent of iteration order and other classes.
        choice = int.from_bytes(hashlib.sha256(f"{seed}:{obj.instance_id}:{variant}".encode()).digest()[:4], "big")
        wood_index = variant % _WOOD_VARIANTS if obj.class_name in _WOOD_CLASSES else choice % _WOOD_VARIANTS
        wood_texture = _texture(asset, f"wood_{wood_index}.png")
        wood = _material(asset, prefix + "_wood", (1, 1, 1), texture=wood_texture, specular=.12, shininess=.1)
        metal = _material(asset, prefix + "_metal", (.38, .41, .40), specular=.75, shininess=.65)
        dark = _material(asset, prefix + "_dark", (.075, .10, .10), specular=.1, shininess=.08)
        palette = _PALETTES.get(obj.class_name, ((.91, .85, .71),))
        palette_index = variant % len(palette)
        color = palette[palette_index]
        glaze_index = choice % 2
        glaze = _material(asset, prefix + "_surface", color,
                          texture=_texture(asset, f"glaze_{glaze_index}.png"),
                          specular=.45 if obj.class_name in {"mug", "plate"} else .23,
                          shininess=.4 if obj.class_name in {"mug", "plate"} else .18)
        accent = _material(asset, prefix + "_accent", tuple(c * .64 for c in color), specular=.3, shininess=.3)
        geoms: list[dict] = []

        def add(part: str, kind: str, material: str, **attrs: Any) -> None:
            geoms.append(_geom(prefix + "_" + part, obj.instance_id, obj.class_id, kind, material=material, **attrs))

        def box(part: str, dimensions: Sequence[float], material: str, *, pos=(0, 0, 0), bevel=.0007, **attrs: Any) -> None:
            mesh = _rounded_box(asset, prefix + "_" + part + "_mesh", dimensions, bevel)
            add(part, "mesh", material, mesh=mesh, pos=tuple(pos), **attrs)

        def lathe(part: str, profile: Sequence[tuple[float, float]], material: str) -> None:
            mesh = _lathe(asset, prefix + "_" + part + "_mesh", profile)
            add(part, "mesh", material, mesh=mesh)

        def ring(part: str, radius: float, z: float, tube: float, material: str) -> None:
            add(part, "mesh", material, mesh=_ring(asset, prefix + "_" + part + "_mesh", radius, z, tube))

        def copy_shape(index: int, source: dict, material: str) -> None:
            attrs = {key: value for key, value in source.items() if key in {"size", "pos", "quat", "fromto"}}
            if source["type"] == "box":
                box(f"support_{index}", tuple(2 * value for value in attrs.pop("size")), material, **attrs)
            else:
                add(f"support_{index}", source["type"], material, **attrs)

        kind = obj.class_name
        if kind in {"jenga_block", "letter_block", "domino"}:
            box("body", obj.size, wood)
            if kind == "letter_block":
                label = letters[obj.name]
                if len(label) != 1 or label not in "ABCDEFGHIJKLMNOPQRSTUVWXYZ":
                    raise ValueError(f"letter block {obj.name} needs one uppercase A-Z label")
                label_mat = _material(asset, prefix + "_letter", (1, 1, 1),
                                      texture=_texture(asset, f"letter_{label}.png"), specular=.12, shininess=.1)
                # A thin, explicitly UV-mapped veneer, not a grid of pixel geoms.
                for side, pos, quat, dimensions in (
                    ("top", (0, 0, obj.size[2] / 2 + .000025), (1, 0, 0, 0), (obj.size[0] * .82, obj.size[1] * .82, .00005)),
                    ("front", (0, -obj.size[1] / 2 - .000025, 0), (math.sqrt(.5), math.sqrt(.5), 0, 0), (obj.size[0] * .82, obj.size[2] * .82, .00005)),
                    ("back", (0, obj.size[1] / 2 + .000025, 0), (0, 0, math.sqrt(.5), math.sqrt(.5)), (obj.size[0] * .82, obj.size[2] * .82, .00005)),
                    ("right", (obj.size[0] / 2 + .000025, 0, 0), (.5, .5, .5, .5), (obj.size[1] * .82, obj.size[2] * .82, .00005)),
                    ("left", (-obj.size[0] / 2 - .000025, 0, 0), (.5, .5, -.5, -.5), (obj.size[1] * .82, obj.size[2] * .82, .00005)),
                    ("bottom", (0, 0, -obj.size[2] / 2 - .000025), (0, 1, 0, 0), (obj.size[0] * .82, obj.size[1] * .82, .00005)),
                ):
                    box("letter_" + side, dimensions, label_mat, pos=pos, quat=quat, bevel=.000008)
                variants[obj.name] = {
                    "letter": label,
                    "letter_art": _provenance()["textures"]["letter_art"]["letters"][label],
                }
        elif kind in {"cup", "mug"}:
            if kind == "cup":
                radius, height, wall, bottom, nest_step = obj.size
                variants[obj.name] = {"bottom_radius_m": bottom, "nest_step_m": nest_step}
            else:
                radius, height, wall = obj.size
                bottom = radius
            # Revolve the contact panels' radial cross-section: their thickness
            # is measured normal to the taper, not along the radius. The floor
            # meets the inner wall at z=wall, without flattening the outer taper.
            slope = math.atan2(radius - bottom, height)
            radial_half_wall = wall / (2 * math.cos(slope))
            outer_bottom = bottom - wall / 2 + radial_half_wall
            outer_top_z = height - wall * math.sin(slope)
            outer_top = outer_bottom + (radius - bottom) * outer_top_z / height
            inner_top = radius - wall / 2 - radial_half_wall
            inner_floor = bottom - wall / 2 - radial_half_wall + (radius - bottom) * wall / height
            lathe("shell", ((0, 0), (outer_bottom, 0), (outer_top, outer_top_z),
                            (inner_top, height), (inner_floor, wall), (0, wall)), glaze)
            ring("lip_accent", radius - wall * .5, height - wall * .24, wall * .22, accent)
            ring("foot", bottom - wall * .5, wall * .4, wall * .36, accent)
            if kind == "cup":
                for index in range(3):
                    angle = index * math.tau / 3
                    add(f"nest_stop_{index}", "cylinder", glaze, size=(wall, (nest_step - wall) / 2),
                        pos=(bottom * .82 * math.cos(angle), bottom * .82 * math.sin(angle), (nest_step + wall) / 2))
                # Molded lower strengthening bead remains inside the wall envelope.
                bead_z = height * .15
                bead_radius = bottom + (radius - bottom) * bead_z / height - wall * .2
                ring("mold_line", bead_radius, bead_z, wall * .2, accent)
            else:
                handle = _oval_tube(asset, prefix + "_handle_mesh", radius + .015, height * .55, .019, .025, .004)
                add("handle", "mesh", glaze, mesh=handle)
                ring("glaze_line", radius - .00018, height * .23, .00018, accent)
        elif kind == "plate":
            radius, thickness = obj.size
            low, high = -thickness / 2, thickness / 2
            well = low + .003
            lathe("dish", ((0, low), (radius * .70, low), (radius * .74, low + .0002),
                           (radius * .98, high - .0032), (radius, high - .0025),
                           (radius, high - .0008), (radius * .993, high),
                           (radius * .976, high), (radius * .74, well + .0002),
                           (radius * .70, well), (0, well)), glaze)
            ring("rim_glaze", radius * .97, high - .00012, .00012, accent)
            ring("well_glaze", radius * .715, well + .00006, .00009, accent)
        elif kind in {"rack", "mug_tree", "cabinet", "drawer"}:
            # Follow every load-bearing component, including open trays and
            # separate drawer roots. Never span slots or openings with a shell.
            for index, source in enumerate(obj.geoms):
                if kind in {"cabinet", "drawer"}:
                    material = metal if "handle" in source["name"] else wood
                elif kind == "rack":
                    material = metal if source["type"] == "capsule" else wood
                else:
                    material = wood if "base" in source["name"] or "trunk" in source["name"] else metal
                copy_shape(index, source, material)
        elif kind == "bin":
            width, depth, height = obj.size[:3]
            wall = .008
            for index, source in enumerate(obj.geoms):
                copy_shape(index, source, glaze)
            # Rim strips sit entirely over the walls, never across the cavity.
            for side in (-1, 1):
                box(f"rim_x_{side}", (wall, depth + 2 * wall, .005), accent,
                    pos=(side * (width + wall) / 2, 0, wall + height - .0025), bevel=.001)
                box(f"rim_y_{side}", (width, wall, .005), accent,
                    pos=(0, side * (depth + wall) / 2, wall + height - .0025), bevel=.001)
                for j, fraction in enumerate((-.35, -.175, 0, .175, .35)):
                    box(f"panel_rib_y_{side}_{j}", (.004, .0005, height * .70), accent,
                        pos=(width * fraction, side * (depth / 2 + wall - .0001), wall + height * .44), bevel=.00012)
                for j, fraction in enumerate((-.3, 0, .3)):
                    box(f"panel_rib_x_{side}_{j}", (.0005, .004, height * .70), accent,
                        pos=(side * (width / 2 + wall - .0001), depth * fraction, wall + height * .44), bevel=.00012)
                box(f"grip_y_{side}", (width * .26, .0006, .014), dark,
                    pos=(0, side * (depth / 2 + wall), height * .83), bevel=.00015)
        else:
            raise ValueError(f"no detailed visual geometry for {kind!r}")
        result[obj.name] = tuple(geoms)
        variants.setdefault(obj.name, {}).update(appearance_variant=variant, palette_index=palette_index,
                                               wood_texture=wood_index, glaze_texture=glaze_index,
                                               color_rgb=list(color))

    table_prefix = "scene_detail_table"
    table_wood_index = int(seed) % _WOOD_VARIANTS
    table_wood = _material(asset, table_prefix + "_wood", (1, 1, 1),
                           texture=_texture(asset, f"wood_{table_wood_index}.png"), specular=.16, shininess=.12)
    table_edge = _material(asset, table_prefix + "_edge", (.36, .25, .16),
                           texture=_texture(asset, "wood_1.png"), specular=.13, shininess=.1)
    table_metal = _material(asset, table_prefix + "_metal", (.16, .19, .19), specular=.45, shininess=.35)
    table: list[dict] = []

    def table_box(part: str, dimensions: tuple[float, ...], pos: tuple[float, ...], material: str, bevel=.002) -> None:
        mesh = _rounded_box(asset, table_prefix + "_" + part + "_mesh", dimensions, bevel)
        table.append(_geom(table_prefix + "_" + part, 0, 0, "mesh", mesh=mesh, pos=pos, material=material))

    table_box("top", (.8, 1.10, .05), (.50, 0, table_top_z - .025), table_wood, .0018)
    for side in (-1, 1):
        table_box(f"edge_x_{side}", (.001, 1.096, .037), (.50 + side * .3995, 0, table_top_z - .029), table_edge, .0002)
        table_box(f"edge_y_{side}", (.796, .001, .037), (.50, side * .5495, table_top_z - .029), table_edge, .0002)
        table_box(f"apron_y_{side}", (.71, .026, .065), (.50, side * .478, table_top_z - .0825), table_edge)
        table_box(f"apron_x_{side}", (.026, .95, .065), (.50 + side * .325, 0, table_top_z - .0825), table_edge)
        for end in (-1, 1):
            x, y = .50 + side * .325, end * .478
            leg_height = table_top_z - .05 - .026
            table_box(f"leg_{side}_{end}", (.038, .038, leg_height),
                      (x, y, .026 + leg_height / 2), table_metal, .003)
            table_box(f"foot_{side}_{end}", (.042, .042, .026), (x, y, .013), table_metal, .003)
    manifest = {**_provenance(), "seed": int(seed), "objects": variants,
                "table": {"wood_texture": table_wood_index, "center_xyz_m": [.50, 0, table_top_z - .025],
                          "near_edge_m": .10, "visual_legs_only": True}}
    return asset, result, tuple(table), manifest
