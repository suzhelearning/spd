"""Portable, textured ABC bottles with independent convex contact geometry.

Packaged OBJ vertices share a canonical radius of one and height of one, with
bottom z=0. Applying the same ``(radius, radius, height)`` mesh scale to every
part preserves visual/contact alignment. Source attribution and the complete
baking transform are recorded in ``assets/abc_bottles/manifest.json``.
"""

from __future__ import annotations

import json
import math
from functools import lru_cache
from pathlib import Path
from typing import Any
import xml.etree.ElementTree as ET


BOTTLE_VARIANTS = (
    "bottle_0",
    "bottle_1",
    "bottle_2",
    "bottle_3",
    "bottle_4",
    "bottle_7",
)

_ASSET_ROOT = Path(__file__).resolve().parent / "assets" / "abc_bottles"
_BOTTLE_CLASS_ID = 6


@lru_cache(maxsize=1)
def _asset_manifest() -> dict[str, Any]:
    with (_ASSET_ROOT / "manifest.json").open(encoding="utf-8") as stream:
        return json.load(stream)


def bottle_geometry(
    asset_id: str,
    size: tuple[float, ...],
    instance_id: int,
) -> tuple[ET.Element, tuple[dict, ...], tuple[dict, ...], dict]:
    """Return instance-local assets, contact parts, visual parts and provenance.

    ``size[:2]`` is the requested enclosing radius and total height in metres.
    Contact volumes are in cubic metres, for the caller's mass allocation;
    contact masks, friction and mass remain the scene builder's responsibility.
    Textures are the source image bytes and OBJ UV indices are unchanged.
    """
    if asset_id not in BOTTLE_VARIANTS:
        raise ValueError(f"Unknown ABC bottle asset: {asset_id!r}")
    radius, height = (float(value) for value in size[:2])
    if not all(math.isfinite(value) and value > 0 for value in (radius, height)):
        raise ValueError("Bottle radius and height must be finite and positive")

    manifest = _asset_manifest()
    variant = manifest["variants"][asset_id]
    prefix = f"scene_bottle_{instance_id:03d}"
    assets = ET.Element("asset")
    texture_names: dict[str, str] = {}
    material_names: dict[str, str] = {}
    for index, texture in enumerate(variant["textures"]):
        attributes = dict(texture["attributes"])
        source_name = attributes["name"]
        attributes["name"] = f"{prefix}_texture_{index}"
        attributes["file"] = str(_ASSET_ROOT / texture["file"])
        texture_names[source_name] = attributes["name"]
        ET.SubElement(assets, "texture", attributes)
    for index, material in enumerate(variant["materials"]):
        attributes = dict(material)
        source_name = attributes["name"]
        attributes["name"] = f"{prefix}_material_{index}"
        if "texture" in attributes:
            attributes["texture"] = texture_names[attributes["texture"]]
        material_names[source_name] = attributes["name"]
        ET.SubElement(assets, "material", attributes)

    collision_geoms: list[dict] = []
    visual_geoms: list[dict] = []
    scale = f"{radius:.12g} {radius:.12g} {height:.12g}"
    volume_scale = radius * radius * height
    for index, mesh in enumerate(variant["meshes"]):
        role = mesh["role"]
        mesh_name = f"{prefix}_{role}_mesh_{index}"
        ET.SubElement(assets, "mesh", {
            "name": mesh_name,
            "file": str(_ASSET_ROOT / mesh["file"]),
            "scale": scale,
        })
        geom = {
            "name": f"{prefix}_{role}_geom_{index}",
            "type": "mesh",
            "mesh": mesh_name,
            "pos": (0.0, 0.0, 0.0),
        }
        if role == "collision":
            geom["volume"] = mesh["volume"] * volume_scale
            collision_geoms.append(geom)
        else:
            geom.update({
                "material": material_names[mesh["source_geom_attributes"]["material"]],
                "rgba": (1.0, 1.0, 1.0, 1.0),
                "mass": "0",
                "contype": "0",
                "conaffinity": "0",
                "group": "2",
                "user": f"{instance_id} {_BOTTLE_CLASS_ID}",
            })
            visual_geoms.append(geom)

    provenance = {
        "asset_id": asset_id,
        "geometry_revision": manifest["geometry_revision"],
        "source_collection": manifest["source_collection"],
        "source_model": variant["source_model"],
        "source_model_sha256": variant["source_model_sha256"],
        "source_sha256": variant["source_sha256"],
        "source_files": [dict(source) for source in variant["source_files"]],
        "asset_manifest": "assets/abc_bottles/manifest.json",
        "mesh_scale": (radius, radius, height),
        "canonical_normalization": {
            key: list(value) if isinstance(value, list) else value
            for key, value in variant["normalization"].items()
        },
        "visual_mesh_count": len(visual_geoms),
        "collision_mesh_count": len(collision_geoms),
        "texture_sha256": {
            texture["file"]: texture["sha256"] for texture in variant["textures"]
        },
        "mesh_sha256": {mesh["file"]: mesh["sha256"] for mesh in variant["meshes"]},
        "rights_notice": manifest["rights_notice"],
    }
    return assets, tuple(collision_geoms), tuple(visual_geoms), provenance
