"""Merge one procedural scene into the verified unified plant."""

from __future__ import annotations

from copy import deepcopy
from pathlib import Path
import xml.etree.ElementTree as ET

from .scene_builder import SceneBuildResult, SceneResetError, contact_gate
from .physical_materials import apply_material_policy


def write_scene_model(base_model: str | Path, result: SceneBuildResult, output_model: str | Path) -> Path:
    base_model, output_model = Path(base_model).resolve(), Path(output_model).resolve()
    if base_model == output_model:
        raise SceneResetError("scene output must not overwrite the verified base model")
    root = ET.parse(base_model).getroot()
    size = root.find("size")
    if size is None:
        size = ET.Element("size")
        root.insert(0, size)
    size.set("nuser_geom", str(max(2, int(size.get("nuser_geom", "0")))))
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise SceneResetError("base model has no worldbody")

    # Compiled assets are relative to the original MJCF, not the scene output.
    # Absolute resource directories also survive MjSpec camera composition.
    compiler = root.find("compiler")
    if compiler is not None:
        # Robot inertials remain explicit; infer only the new compound objects.
        compiler.set("inertiafromgeom", "auto")
        for attribute in ("assetdir", "meshdir", "texturedir"):
            if attribute in compiler.attrib:
                compiler.set(attribute, str((base_model.parent / compiler.get(attribute)).resolve()))
    for element in root.iter():
        file_name = element.get("file")
        if not file_name:
            continue
        directory = base_model.parent
        if compiler is not None and element.tag in {"mesh", "texture", "hfield"}:
            attribute = {"mesh": "meshdir", "texture": "texturedir", "hfield": "assetdir"}[element.tag]
            directory = Path(compiler.get(attribute, compiler.get("assetdir", str(directory))))
        source = (directory / file_name).resolve()
        if not source.is_file():
            raise SceneResetError(f"base model resource not found: {source}")
        element.set("file", str(source))
    assets = root.find("asset")
    if assets is None:
        assets = ET.SubElement(root, "asset")
    asset_names = {(child.tag, child.get("name")) for child in assets}
    for child in result.assets:
        identity = (child.tag, child.get("name"))
        if identity in asset_names:
            raise SceneResetError(f"duplicate scene asset: {identity}")
        asset_names.add(identity)
        assets.append(deepcopy(child))
    # Scene lighting is shared with standalone previews; keep one lighting rig.
    for light in list(worldbody.findall("light")):
        worldbody.remove(light)
    for child in result.worldbody:
        if child.tag == "body" and any(existing.attrib.get("name") == child.attrib.get("name") for existing in worldbody.findall("body")):
            raise SceneResetError(f"duplicate scene body: {child.attrib.get('name')}")
        if child.tag == "geom" and any(existing.attrib.get("name") == child.attrib.get("name") for existing in worldbody.findall("geom")):
            raise SceneResetError(f"duplicate scene geom: {child.attrib.get('name')}")
        worldbody.append(deepcopy(child))
    apply_material_policy(root, result.objects, result.material_coefficients)
    output_model.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(output_model, encoding="utf-8", xml_declaration=True)
    try:
        import mujoco
        model = mujoco.MjModel.from_xml_path(str(output_model))
        data = mujoco.MjData(model)
        object_names = {item.name for item in result.objects}
        contact_gate(model, data, object_names)
    except Exception:
        output_model.unlink(missing_ok=True)
        raise
    return output_model


__all__ = ["write_scene_model"]
