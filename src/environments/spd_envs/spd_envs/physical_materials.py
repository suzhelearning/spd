"""Engineering contact friction; labels never change geometry or inertial data."""
import re
import xml.etree.ElementTree as ET


MATERIAL_IDS = {
    "wood": 1, "polyethylene": 2, "ceramic_glaze": 3,
    "unglazed_ceramic": 4, "bare_iron": 5,
    "polyester_woven_fabric": 6, "silicone": 7,
}
_OBJECT_MATERIALS = {
    "jenga_block": "wood", "domino": "wood", "letter_block": "wood",
    "cabinet": "wood", "drawer": "wood",
    "cup": "polyethylene", "bottle": "polyethylene", "bin": "polyethylene",
    "rack": "bare_iron",
}
# Single effective sliding coefficients: dry, unlubricated engineering choices.
# Only the two previously approved same-material values have reference provenance.
_PAIR_COEFFICIENTS = {
    ("wood", "wood"): .40,
    ("polyethylene", "polyethylene"): .20,
    ("wood", "polyethylene"): .30,
    ("wood", "ceramic_glaze"): .30,
    ("wood", "unglazed_ceramic"): .40,
    ("wood", "bare_iron"): .40,
    ("polyethylene", "ceramic_glaze"): .20,
    ("polyethylene", "unglazed_ceramic"): .25,
    ("polyethylene", "bare_iron"): .20,
    ("ceramic_glaze", "ceramic_glaze"): .25,
    ("ceramic_glaze", "unglazed_ceramic"): .35,
    ("unglazed_ceramic", "unglazed_ceramic"): .50,
    ("ceramic_glaze", "bare_iron"): .25,
    ("unglazed_ceramic", "bare_iron"): .35,
    ("bare_iron", "bare_iron"): .30,
    ("polyester_woven_fabric", "wood"): .50,
    ("polyester_woven_fabric", "polyethylene"): .30,
    ("polyester_woven_fabric", "ceramic_glaze"): .35,
    ("polyester_woven_fabric", "unglazed_ceramic"): .45,
    ("polyester_woven_fabric", "bare_iron"): .40,
    ("silicone", "wood"): .80,
    ("silicone", "polyethylene"): .80,
    ("silicone", "ceramic_glaze"): .70,
    ("silicone", "unglazed_ceramic"): .80,
    ("silicone", "bare_iron"): .70,
    ("silicone", "polyester_woven_fabric"): .80,
}
_FINGER_BODY = re.compile(r"[lr]_(?:thumb|index_finger|middle_finger|ring_finger|pinky)_(?:proximal|proximal_abd|middle|distal)")


def object_surface_materials(obj):
    """Contact face labels in object-body coordinates, not new collision geoms."""
    if obj.class_name in {"mug", "plate"}:
        return {
            "default": "ceramic_glaze",
            "regions": [{"geom": obj.geoms[0]["name"], "surface": "underside",
                         "material": "unglazed_ceramic"}],
            "mapping": "outward contact normal in body frame: z < -0.5 selects underside; other faces glazed",
        }
    if obj.class_name == "mug_tree":
        return {"default": "bare_iron", "regions": [
            {"geom": obj.geoms[0]["name"], "surface": "all", "material": "wood"},
        ]}
    return {"default": _OBJECT_MATERIALS[obj.class_name], "regions": []}


def material_manifest():
    """Keep chosen coefficients, reference limits and runtime requirements explicit."""
    pairs = []
    for materials, mu in _PAIR_COEFFICIENTS.items():
        entry = {"materials": list(materials), "effective_sliding_mu": mu,
                 "basis": "engineering_choice_not_measured; dry unlubricated single effective sliding coefficient"}
        if materials == ("wood", "wood"):
            entry.update(basis="dry bare wood, isotropic engineering approximation; not calibrated",
                         reference_static_mu=[.54, .62], reference_sliding_mu=[.32, .48],
                         source="https://ntrs.nasa.gov/api/citations/19900009424/downloads/19900009424.pdf",
                         source_location="Table IV, printed page 16; oak on oak, perpendicular/parallel grain")
        elif materials == ("polyethylene", "polyethylene"):
            entry.update(basis="single effective coefficient approximated from dry static friction; not measured kinetic friction",
                         reference_static_mu=.20, reference_sliding_mu=None,
                         source="https://www.engineeringtoolbox.com/friction-coefficients-d_778.html")
        pairs.append(entry)
    return {
        "revision": "engineering-contact-friction-v2",
        "surface_label_source": "user specification; not manufacturer-certified",
        "approved_pairs": pairs,
        "environment_surfaces": {"table": "polyester_woven_fabric", "ground": "polyester_woven_fabric"},
        "robot_surfaces": {
            "hand_skin": {"material": "silicone", "hardware": "Wuji Hand 2.1 Beta",
                          "scope": ["whole_distal_fingertips", "whole_finger_pads", "palm"],
                          "assignment": "existing geoms; whole distal tips and palmar contact faces only",
                          "silicone_assumption": "ordinary commercial silicone rubber; engineering approximation, no measured formulation",
                          "palmar_normal_body_y": {"left": "< 0", "right": "> 0"}},
            "hand_bare_shell": {"material": None, "friction": "unchanged"},
            "arm_shell": {"material": None, "friction": "unchanged"},
        },
        "runtime": {"numeric": "spd_material_friction", "version": 2,
                    "material_ids": MATERIAL_IDS, "geom_user_fields": {"material_id": 2, "surface_region": 3},
                    "surface_regions": {"whole": 0, "ceramic_underside": 1, "left_palmar": 2, "right_palmar": 3},
                    "required": "_spd_native material_step/material_forward or Physics; bare mj_step does not apply this policy"},
        "unapproved_pairs": "retain legacy geom friction and contact mixing; no hand self-contact override",
        "fixed_fixture_table": "existing collision filters preserved; no new contacts",
        "preserved": ["visuals", "mass", "inertia", "geometry", "random_draw_order",
                      "condim", "normal_contact_parameters", "joint_and_actuator_parameters"],
    }


def _label_geom(geom, material, region=0):
    if geom.get("contype", "1") == "0" and geom.get("conaffinity", "1") == "0":
        return
    values = geom.get("user", "").split()
    values.extend(["0"] * max(0, 4 - len(values)))
    values[2:4] = [str(MATERIAL_IDS[material]), str(region)]
    geom.set("user", " ".join(values))


def apply_material_policy(root, objects):
    """Embed portable coefficients and face rules, preserving original collision filters."""
    size = root.find("size")
    if size is None:
        size = ET.SubElement(root, "size")
    size.set("nuser_geom", str(max(4, int(size.get("nuser_geom", "0")))))
    custom = root.find("custom")
    if custom is None:
        custom = ET.SubElement(root, "custom")
    matrix = [-1.] * 64
    for (first, second), mu in _PAIR_COEFFICIENTS.items():
        i, j = MATERIAL_IDS[first], MATERIAL_IDS[second]
        matrix[8 * i + j] = matrix[8 * j + i] = mu
    ET.SubElement(custom, "numeric", name="spd_material_friction",
                  data=" ".join(f"{value:g}" for value in [2, *matrix]))
    worldbody = root.find("worldbody")
    bodies = {body.get("name"): body for body in worldbody.iter("body")}
    for obj in objects:
        labels = object_surface_materials(obj)
        overrides = {region["geom"]: region for region in labels["regions"]}
        for geom in bodies[obj.name].iter("geom"):
            region = overrides.get(geom.get("name"))
            material, kind = labels["default"], 0
            if region is not None:
                if region["surface"] == "all":
                    material = region["material"]
                else:
                    kind = 1
            _label_geom(geom, material, kind)
    for geom in worldbody.findall("geom"):
        if geom.get("name") in {"ground", "scene_table"}:
            _label_geom(geom, "polyester_woven_fabric")
    for name, body in bodies.items():
        if name in {"l_wrist", "r_wrist"} or _FINGER_BODY.fullmatch(name):
            kind = 0 if name.endswith("_distal") else (2 if name.startswith("l_") else 3)
            for geom in body.findall("geom"):
                _label_geom(geom, "silicone", kind)
