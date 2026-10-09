"""Engineering contact friction; labels never change geometry or inertial data."""
import math
import os
from collections.abc import Mapping
from numbers import Real
from pathlib import Path
import re
import tempfile
import xml.etree.ElementTree as ET

import yaml


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
# Fixed editor/manifest order; every pair is canonical by MATERIAL_IDS.
# Coefficients live only in the editable YAML configuration.
PAIR_KEYS = (
    ("wood", "wood"),
    ("polyethylene", "polyethylene"),
    ("wood", "polyethylene"),
    ("wood", "ceramic_glaze"),
    ("wood", "unglazed_ceramic"),
    ("wood", "bare_iron"),
    ("polyethylene", "ceramic_glaze"),
    ("polyethylene", "unglazed_ceramic"),
    ("polyethylene", "bare_iron"),
    ("ceramic_glaze", "ceramic_glaze"),
    ("ceramic_glaze", "unglazed_ceramic"),
    ("unglazed_ceramic", "unglazed_ceramic"),
    ("ceramic_glaze", "bare_iron"),
    ("unglazed_ceramic", "bare_iron"),
    ("bare_iron", "bare_iron"),
    ("wood", "polyester_woven_fabric"),
    ("polyethylene", "polyester_woven_fabric"),
    ("ceramic_glaze", "polyester_woven_fabric"),
    ("unglazed_ceramic", "polyester_woven_fabric"),
    ("bare_iron", "polyester_woven_fabric"),
    ("wood", "silicone"),
    ("polyethylene", "silicone"),
    ("ceramic_glaze", "silicone"),
    ("unglazed_ceramic", "silicone"),
    ("bare_iron", "silicone"),
    ("polyester_woven_fabric", "silicone"),
)


def material_config_path() -> Path:
    """Locate the editable repository config or the installed package default."""
    override = os.environ.get("SPD_MATERIAL_FRICTION_CONFIG")
    if override is not None:
        if not override.strip():
            raise ValueError("SPD_MATERIAL_FRICTION_CONFIG must not be empty")
        return Path(override).expanduser()
    module = Path(__file__).resolve()
    for parent in module.parents:
        source = parent / "src/environments/spd_envs/spd_envs/physical_materials.py"
        if source == module:
            return parent / "config/material_friction.yaml"
    return module.parent / "defaults/material_friction.yaml"


def _task_identity(scene, task):
    for name, value in (("scene", scene), ("task", task)):
        if not isinstance(value, str) or not re.fullmatch(r"[A-Za-z0-9_][A-Za-z0-9_.-]*", value):
            raise ValueError(f"{name} must be a nonempty task path component: {value!r}")
    return scene, task


def task_material_config_path(scene: str, task: str) -> Path:
    """Locate a seed-independent profile, writable also outside a source checkout."""
    _task_identity(scene, task)
    override = os.environ.get("SPD_TASK_MATERIAL_FRICTION_DIR")
    if override is not None:
        if not override.strip():
            raise ValueError("SPD_TASK_MATERIAL_FRICTION_DIR must not be empty")
        root = Path(override).expanduser()
    elif "SPD_MATERIAL_FRICTION_CONFIG" in os.environ:
        root = material_config_path().parent / "task_material_friction"
    else:
        template = material_config_path()
        if template.parent == Path(__file__).resolve().parent / "defaults":
            user_config = os.environ.get("XDG_CONFIG_HOME")
            root = (Path(user_config).expanduser() if user_config
                    else Path.home() / ".config") / "spd/task_material_friction"
        else:
            root = template.parent / "task_material_friction"
    return (root / scene / f"{task}.yaml").absolute()


def _canonical_pair(pair):
    if not isinstance(pair, (tuple, list)) or len(pair) != 2:
        raise ValueError(f"material pair must contain two material names: {pair!r}")
    if any(not isinstance(name, str) or name not in MATERIAL_IDS for name in pair):
        raise ValueError(f"unknown material in pair: {pair!r}")
    canonical = tuple(sorted(pair, key=MATERIAL_IDS.__getitem__))
    if canonical not in PAIR_KEYS:
        raise ValueError(f"unapproved material pair: {pair!r}")
    return canonical


def validate_material_coefficients(coefficients) -> dict[tuple[str, str], float]:
    """Require exactly the approved pairs, without ambiguous symmetric entries."""
    if not isinstance(coefficients, Mapping):
        raise ValueError("material coefficients must be a mapping of pairs to numbers")
    values = {}
    for pair, coefficient in coefficients.items():
        canonical = _canonical_pair(pair)
        if canonical in values:
            raise ValueError(f"duplicate material pair: {canonical!r}")
        if isinstance(coefficient, bool) or not isinstance(coefficient, Real):
            raise ValueError(f"coefficient for {canonical!r} must be a number")
        coefficient = float(coefficient)
        if not math.isfinite(coefficient) or coefficient < 0:
            raise ValueError(f"coefficient for {canonical!r} must be finite and nonnegative")
        values[canonical] = coefficient
    missing = set(PAIR_KEYS) - values.keys()
    if missing:
        raise ValueError(f"missing material pairs: {sorted(missing)!r}")
    return {pair: values[pair] for pair in PAIR_KEYS}


class _UniqueKeyLoader(yaml.SafeLoader):
    """Reject duplicate YAML fields rather than silently taking the last one."""


def _unique_mapping(loader, node, deep=False):
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if not isinstance(key, str):
            raise ValueError("material configuration field names must be strings")
        if key in result:
            raise ValueError(f"duplicate material configuration field: {key}")
        result[key] = loader.construct_object(value_node, deep=deep)
    return result


_UniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _unique_mapping,
)


def _read_material_document(path):
    try:
        return yaml.load(path.read_text(encoding="utf-8"), Loader=_UniqueKeyLoader)
    except (yaml.YAMLError, UnicodeError) as exc:
        raise ValueError(f"invalid material configuration {path}: {exc}") from exc


def _document_coefficients(document):
    if type(document["version"]) is not int or document["version"] != 1:
        raise ValueError("unsupported material configuration version (expected 1)")
    if not isinstance(document["pairs"], list):
        raise ValueError("material configuration pairs must be a list")
    coefficients = {}
    for entry in document["pairs"]:
        if not isinstance(entry, dict) or set(entry) != {"materials", "sliding_mu"}:
            raise ValueError("each material pair must contain only materials and sliding_mu")
        pair = _canonical_pair(entry["materials"])
        if pair in coefficients:
            raise ValueError(f"duplicate material pair: {pair!r}")
        coefficients[pair] = entry["sliding_mu"]
    return validate_material_coefficients(coefficients)


def load_material_coefficients(path=None) -> dict[tuple[str, str], float]:
    """Read the complete global template; partial or asymmetric policies are errors."""
    path = material_config_path() if path is None else Path(path)
    document = _read_material_document(path)
    if not isinstance(document, dict) or set(document) != {"version", "pairs"}:
        raise ValueError("material configuration must contain only version and pairs")
    return _document_coefficients(document)


def _material_document(coefficients, *, scene=None, task=None):
    document = {"version": 1}
    if scene is not None:
        document.update(scene=scene, task=task)
    document["pairs"] = [
        {"materials": list(pair), "sliding_mu": mu}
        for pair, mu in coefficients.items()
    ]
    return document


def _write_material_document(path, document, *, create=False):
    """Publish only a complete file; hard-link creation never replaces a winner."""
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = yaml.safe_dump(document, sort_keys=False, allow_unicode=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=path.parent,
                                         prefix=f".{path.name}.", suffix=".tmp",
                                         delete=False) as stream:
            temporary = Path(stream.name)
            stream.write("# Dry, unlubricated effective sliding coefficients; no upper bound.\n")
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        if create:
            try:
                os.link(temporary, path)
            except FileExistsError:
                pass
        else:
            os.replace(temporary, path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def save_material_coefficients(coefficients, path=None) -> None:
    """Validate and atomically replace the complete global initialization template."""
    coefficients = validate_material_coefficients(coefficients)
    path = material_config_path() if path is None else Path(path)
    _write_material_document(path, _material_document(coefficients))


def _load_task_material_profile(scene, task, path):
    document = _read_material_document(path)
    if not isinstance(document, dict) or set(document) != {"version", "scene", "task", "pairs"}:
        raise ValueError(f"task material configuration {path} must contain version, scene, task and pairs")
    if document["scene"] != scene or document["task"] != task:
        raise ValueError(f"task material configuration {path} does not belong to {scene}/{task}")
    return _document_coefficients(document)


def load_task_material_coefficients(scene: str, task: str) -> dict[tuple[str, str], float]:
    """Load an independent complete task profile, initializing it exactly once."""
    path = task_material_config_path(scene, task)
    try:
        return _load_task_material_profile(scene, task, path)
    except FileNotFoundError:
        if path.is_symlink():
            raise
    coefficients = load_material_coefficients()
    _write_material_document(path, _material_document(coefficients, scene=scene, task=task),
                             create=True)
    # A concurrent initializer may have won. Always use the published profile.
    return _load_task_material_profile(scene, task, path)


def save_task_material_coefficients(scene: str, task: str, coefficients, *, path=None) -> None:
    """Replace only this task's complete policy, optionally at its captured path."""
    _task_identity(scene, task)
    coefficients = validate_material_coefficients(coefficients)
    path = task_material_config_path(scene, task) if path is None else Path(path)
    try:
        _load_task_material_profile(scene, task, path)
    except FileNotFoundError:
        if path.is_symlink():
            raise
    _write_material_document(path, _material_document(coefficients, scene=scene, task=task))


def model_material_pairs(model) -> tuple[tuple[str, str], ...]:
    """Filter the approved order by all collidable surfaces, not active contacts."""
    if model.nuser_geom < 3:
        return ()
    material_ids = set()
    for geom_id in range(model.ngeom):
        if not (model.geom_contype[geom_id] or model.geom_conaffinity[geom_id]):
            continue
        material = model.geom_user[geom_id, 2]
        material_ids.add(material)
        if (material == MATERIAL_IDS["ceramic_glaze"] and model.nuser_geom >= 4
                and model.geom_user[geom_id, 3] == 1):
            material_ids.add(MATERIAL_IDS["unglazed_ceramic"])
    return tuple(pair for pair in PAIR_KEYS
                 if all(MATERIAL_IDS[name] in material_ids for name in pair))


def material_numeric_values(coefficients) -> list[float]:
    """Return version 2 and the symmetric 8x8 matrix, with legacy -1 sentinels."""
    coefficients = validate_material_coefficients(coefficients)
    matrix = [-1.] * 64
    for (first, second), mu in coefficients.items():
        i, j = MATERIAL_IDS[first], MATERIAL_IDS[second]
        matrix[8 * i + j] = matrix[8 * j + i] = mu
    return [2., *matrix]


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


def material_manifest(coefficients=None):
    """Keep chosen coefficients, reference limits and runtime requirements explicit."""
    coefficients = (load_material_coefficients() if coefficients is None
                    else validate_material_coefficients(coefficients))
    pairs = []
    for materials, mu in coefficients.items():
        entry = {"materials": list(materials), "effective_sliding_mu": mu,
                 "basis": "engineering_choice_not_measured; dry unlubricated single effective sliding coefficient"}
        if materials == ("wood", "wood"):
            entry.update(reference_static_mu=[.54, .62], reference_sliding_mu=[.32, .48],
                         source="https://ntrs.nasa.gov/api/citations/19900009424/downloads/19900009424.pdf",
                         source_location="Table IV, printed page 16; oak on oak, perpendicular/parallel grain")
        elif materials == ("polyethylene", "polyethylene"):
            entry.update(reference_static_mu=.20, reference_sliding_mu=None,
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


def apply_material_policy(root, objects, coefficients=None):
    """Embed portable coefficients and face rules, preserving original collision filters."""
    coefficients = load_material_coefficients() if coefficients is None else coefficients
    numeric_values = material_numeric_values(coefficients)
    size = root.find("size")
    if size is None:
        size = ET.SubElement(root, "size")
    size.set("nuser_geom", str(max(4, int(size.get("nuser_geom", "0")))))
    custom = root.find("custom")
    if custom is None:
        custom = ET.SubElement(root, "custom")
    for existing in custom.findall("numeric[@name='spd_material_friction']"):
        custom.remove(existing)
    ET.SubElement(custom, "numeric", name="spd_material_friction",
                  data=" ".join(f"{value:.17g}" for value in numeric_values))
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
