"""Self-contained MuJoCo geometry export for browser replay.

The exporter deliberately uses only compiled MJB arrays.  In particular, mesh
vertices are already in MuJoCo's cooked mesh frame, so each mesh is placed by
its geom frame exactly once.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

import mujoco
import numpy as np
import trimesh
from PIL import Image, ImageOps
from trimesh.exchange.gltf import export_glb
from trimesh.visual.material import PBRMaterial
from trimesh.visual.texture import TextureVisuals


_VISIBLE_GROUPS = frozenset((1, 2))
_RADIAL_SEGMENTS = 32
_SPHERE_SEGMENTS = 16
_CAP_SEGMENTS = 8


class SceneError(ValueError):
    """The compiled model contains visual data that cannot become a GLB."""


@dataclass(frozen=True)
class _MaterialBinding:
    material: PBRMaterial
    key: tuple[Any, ...]
    needs_uv: bool
    uv_scale: tuple[float, float]


def _finite_vector(value: Any, length: int, label: str) -> np.ndarray:
    array = np.asarray(value, dtype=np.float64)
    if array.shape != (length,) or not np.all(np.isfinite(array)):
        raise SceneError(f"{label} must be a finite vector of length {length}")
    return array


def _positive_sizes(value: Any, count: int, label: str) -> np.ndarray:
    sizes = _finite_vector(value, 3, label)[:count]
    if np.any(sizes <= 0):
        raise SceneError(f"{label} must be positive")
    return sizes


def _transform(position: Any, quaternion: Any, label: str) -> np.ndarray:
    """Return a homogeneous matrix from MuJoCo's wxyz quaternion convention."""
    xyz = _finite_vector(position, 3, f"{label} position")
    wxyz = _finite_vector(quaternion, 4, f"{label} quaternion")
    norm = float(np.linalg.norm(wxyz))
    if not np.isfinite(norm) or norm <= np.finfo(np.float64).eps:
        raise SceneError(f"{label} quaternion has zero length")
    w, x, y, z = wxyz / norm
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = (
        (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)),
        (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)),
        (2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)),
    )
    matrix[:3, 3] = xyz
    return matrix


def _mesh_from_arrays(
    vertices: np.ndarray,
    faces: np.ndarray,
    normals: np.ndarray | None,
    uv: np.ndarray | None,
    material: PBRMaterial,
    label: str,
) -> trimesh.Trimesh:
    vertices = np.asarray(vertices, dtype=np.float64)
    faces = np.asarray(faces, dtype=np.int64)
    if vertices.ndim != 2 or vertices.shape[1] != 3 or not len(vertices):
        raise SceneError(f"{label} has no usable vertices")
    if faces.ndim != 2 or faces.shape[1] != 3 or not len(faces):
        raise SceneError(f"{label} has no triangular faces")
    if not np.all(np.isfinite(vertices)) or np.any(faces < 0) or np.any(faces >= len(vertices)):
        raise SceneError(f"{label} has invalid vertices or faces")
    if normals is not None:
        normals = np.asarray(normals, dtype=np.float64)
        if normals.shape != vertices.shape or not np.all(np.isfinite(normals)):
            raise SceneError(f"{label} has invalid vertex normals")
    if uv is not None:
        uv = np.asarray(uv, dtype=np.float64)
        if uv.shape != (len(vertices), 2) or not np.all(np.isfinite(uv)):
            raise SceneError(f"{label} has invalid UV coordinates")
    visual = TextureVisuals(uv=uv, material=material)
    return trimesh.Trimesh(
        vertices=vertices,
        faces=faces,
        vertex_normals=normals,
        visual=visual,
        process=False,
        validate=False,
    )


def _quad_mesh(
    quads: list[tuple[np.ndarray, np.ndarray]],
    material: PBRMaterial,
    uv_scale: tuple[float, float],
    label: str,
) -> trimesh.Trimesh:
    """Build face-separated quads so planar texture coordinates stay correct."""
    vertices: list[np.ndarray] = []
    normals: list[np.ndarray] = []
    uvs: list[np.ndarray] = []
    faces: list[tuple[int, int, int]] = []
    base_uv = np.asarray(((0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)), dtype=np.float64)
    for corners, normal in quads:
        start = len(vertices)
        vertices.extend(np.asarray(corners, dtype=np.float64))
        normals.extend(np.repeat(np.asarray(normal, dtype=np.float64)[None, :], 4, axis=0))
        uvs.extend(base_uv * np.asarray(uv_scale, dtype=np.float64))
        faces.extend(((start, start + 1, start + 2), (start, start + 2, start + 3)))
    return _mesh_from_arrays(
        np.asarray(vertices), np.asarray(faces), np.asarray(normals), np.asarray(uvs), material, label
    )


def _plane_mesh(size: np.ndarray, material: PBRMaterial, uv_scale: tuple[float, float], label: str) -> trimesh.Trimesh:
    half_x, half_y = _positive_sizes(size, 2, label)
    corners = np.asarray(((-half_x, -half_y, 0.0), (half_x, -half_y, 0.0),
                          (half_x, half_y, 0.0), (-half_x, half_y, 0.0)))
    return _quad_mesh([(corners, np.asarray((0.0, 0.0, 1.0)))], material, uv_scale, label)


def _box_mesh(size: np.ndarray, material: PBRMaterial, uv_scale: tuple[float, float], label: str) -> trimesh.Trimesh:
    x, y, z = _positive_sizes(size, 3, label)
    quads = [
        (np.asarray(((x, -y, -z), (x, y, -z), (x, y, z), (x, -y, z))), np.asarray((1.0, 0.0, 0.0))),
        (np.asarray(((-x, -y, -z), (-x, -y, z), (-x, y, z), (-x, y, -z))), np.asarray((-1.0, 0.0, 0.0))),
        (np.asarray(((-x, y, -z), (-x, y, z), (x, y, z), (x, y, -z))), np.asarray((0.0, 1.0, 0.0))),
        (np.asarray(((-x, -y, -z), (x, -y, -z), (x, -y, z), (-x, -y, z))), np.asarray((0.0, -1.0, 0.0))),
        (np.asarray(((-x, -y, z), (x, -y, z), (x, y, z), (-x, y, z))), np.asarray((0.0, 0.0, 1.0))),
        (np.asarray(((-x, -y, -z), (-x, y, -z), (x, y, -z), (x, -y, -z))), np.asarray((0.0, 0.0, -1.0))),
    ]
    return _quad_mesh(quads, material, uv_scale, label)


def _latlong_mesh(
    radii: np.ndarray,
    material: PBRMaterial,
    uv_scale: tuple[float, float],
    label: str,
) -> trimesh.Trimesh:
    radii = _positive_sizes(radii, 3, label)
    longitudes = _RADIAL_SEGMENTS
    latitudes = _SPHERE_SEGMENTS
    vertices: list[tuple[float, float, float]] = []
    normals: list[tuple[float, float, float]] = []
    uv: list[tuple[float, float]] = []
    for latitude in range(latitudes + 1):
        fraction = latitude / latitudes
        phi = -0.5 * np.pi + np.pi * fraction
        cosine, sine = float(np.cos(phi)), float(np.sin(phi))
        for longitude in range(longitudes + 1):
            horizontal = longitude / longitudes
            theta = 2.0 * np.pi * horizontal
            direction = np.asarray((cosine * np.cos(theta), cosine * np.sin(theta), sine), dtype=np.float64)
            point = direction * radii
            normal = direction / radii
            normal /= np.linalg.norm(normal)
            vertices.append(tuple(point))
            normals.append(tuple(normal))
            uv.append((horizontal * uv_scale[0], fraction * uv_scale[1]))
    faces: list[tuple[int, int, int]] = []
    width = longitudes + 1
    for latitude in range(latitudes):
        for longitude in range(longitudes):
            lower_left = latitude * width + longitude
            lower_right = lower_left + 1
            upper_left = lower_left + width
            upper_right = upper_left + 1
            faces.extend(((lower_left, lower_right, upper_right), (lower_left, upper_right, upper_left)))
    return _mesh_from_arrays(
        np.asarray(vertices), np.asarray(faces), np.asarray(normals), np.asarray(uv), material, label
    )


def _cylinder_mesh(size: np.ndarray, material: PBRMaterial, uv_scale: tuple[float, float], label: str) -> trimesh.Trimesh:
    radius, half_length = _positive_sizes(size, 2, label)
    segments = _RADIAL_SEGMENTS
    vertices: list[tuple[float, float, float]] = []
    normals: list[tuple[float, float, float]] = []
    uv: list[tuple[float, float]] = []
    for z, v in ((-half_length, 0.0), (half_length, 1.0)):
        for index in range(segments + 1):
            fraction = index / segments
            theta = 2.0 * np.pi * fraction
            cosine, sine = float(np.cos(theta)), float(np.sin(theta))
            vertices.append((radius * cosine, radius * sine, z))
            normals.append((cosine, sine, 0.0))
            uv.append((fraction * uv_scale[0], v * uv_scale[1]))
    faces: list[tuple[int, int, int]] = []
    top_offset = segments + 1
    for index in range(segments):
        lower_left, lower_right = index, index + 1
        upper_left, upper_right = top_offset + index, top_offset + index + 1
        faces.extend(((lower_left, lower_right, upper_right), (lower_left, upper_right, upper_left)))

    def cap(z: float, normal_z: float, reverse: bool) -> None:
        center = len(vertices)
        vertices.append((0.0, 0.0, z))
        normals.append((0.0, 0.0, normal_z))
        uv.append((0.5 * uv_scale[0], 0.5 * uv_scale[1]))
        rim = len(vertices)
        for index in range(segments + 1):
            fraction = index / segments
            theta = 2.0 * np.pi * fraction
            cosine, sine = float(np.cos(theta)), float(np.sin(theta))
            vertices.append((radius * cosine, radius * sine, z))
            normals.append((0.0, 0.0, normal_z))
            uv.append(((0.5 + 0.5 * cosine) * uv_scale[0], (0.5 + 0.5 * sine) * uv_scale[1]))
        for index in range(segments):
            if reverse:
                faces.append((center, rim + index + 1, rim + index))
            else:
                faces.append((center, rim + index, rim + index + 1))

    cap(-half_length, -1.0, True)
    cap(half_length, 1.0, False)
    return _mesh_from_arrays(
        np.asarray(vertices), np.asarray(faces), np.asarray(normals), np.asarray(uv), material, label
    )


def _capsule_mesh(size: np.ndarray, material: PBRMaterial, uv_scale: tuple[float, float], label: str) -> trimesh.Trimesh:
    radius, half_length = _positive_sizes(size, 2, label)
    if half_length <= np.finfo(np.float64).eps:
        return _latlong_mesh(np.asarray((radius, radius, radius)), material, uv_scale, label)
    rings: list[tuple[float, float, float]] = []
    # (z, radial distance, normal z) from south pole to north pole.
    for index in range(_CAP_SEGMENTS + 1):
        phi = -0.5 * np.pi + 0.5 * np.pi * index / _CAP_SEGMENTS
        rings.append((float(-half_length + radius * np.sin(phi)), float(radius * np.cos(phi)), float(np.sin(phi))))
    rings.append((float(half_length), float(radius), 0.0))
    for index in range(1, _CAP_SEGMENTS + 1):
        phi = 0.5 * np.pi * index / _CAP_SEGMENTS
        rings.append((float(half_length + radius * np.sin(phi)), float(radius * np.cos(phi)), float(np.sin(phi))))

    vertices: list[tuple[float, float, float]] = []
    normals: list[tuple[float, float, float]] = []
    uv: list[tuple[float, float]] = []
    total_height = 2.0 * (half_length + radius)
    for z, radial, normal_z in rings:
        radial_normal = float(np.sqrt(max(0.0, 1.0 - normal_z * normal_z)))
        vertical = (z + half_length + radius) / total_height
        for index in range(_RADIAL_SEGMENTS + 1):
            horizontal = index / _RADIAL_SEGMENTS
            theta = 2.0 * np.pi * horizontal
            cosine, sine = float(np.cos(theta)), float(np.sin(theta))
            vertices.append((radial * cosine, radial * sine, z))
            normals.append((radial_normal * cosine, radial_normal * sine, normal_z))
            uv.append((horizontal * uv_scale[0], vertical * uv_scale[1]))
    faces: list[tuple[int, int, int]] = []
    width = _RADIAL_SEGMENTS + 1
    for ring in range(len(rings) - 1):
        for index in range(_RADIAL_SEGMENTS):
            lower_left = ring * width + index
            lower_right = lower_left + 1
            upper_left = lower_left + width
            upper_right = upper_left + 1
            faces.extend(((lower_left, lower_right, upper_right), (lower_left, upper_right, upper_left)))
    return _mesh_from_arrays(
        np.asarray(vertices), np.asarray(faces), np.asarray(normals), np.asarray(uv), material, label
    )


def _hfield_mesh(
    model: Any,
    hfield_id: int,
    material: PBRMaterial,
    uv_scale: tuple[float, float],
    label: str,
) -> trimesh.Trimesh:
    if hfield_id < 0 or hfield_id >= int(model.nhfield):
        raise SceneError(f"{label} references an invalid height field")
    rows, columns = int(model.hfield_nrow[hfield_id]), int(model.hfield_ncol[hfield_id])
    if rows < 2 or columns < 2:
        raise SceneError(f"{label} height field needs at least two rows and columns")
    size = _finite_vector(model.hfield_size[hfield_id], 4, f"{label} height field size")
    if np.any(size[:3] <= 0) or size[3] < 0:
        raise SceneError(f"{label} height field has invalid size")
    start = int(model.hfield_adr[hfield_id])
    count = rows * columns
    heights = np.asarray(model.hfield_data[start:start + count], dtype=np.float64)
    if heights.shape != (count,) or not np.all(np.isfinite(heights)):
        raise SceneError(f"{label} height field data is invalid")
    x = np.linspace(-size[0], size[0], columns)
    y = np.linspace(-size[1], size[1], rows)
    xx, yy = np.meshgrid(x, y)
    vertices = np.column_stack((xx.reshape(-1), yy.reshape(-1), heights * size[2]))
    uv = np.column_stack((
        np.tile(np.linspace(0.0, uv_scale[0], columns), rows),
        np.repeat(np.linspace(0.0, uv_scale[1], rows), columns),
    ))
    faces: list[tuple[int, int, int]] = []
    for row in range(rows - 1):
        for column in range(columns - 1):
            lower_left = row * columns + column
            lower_right = lower_left + 1
            upper_left = lower_left + columns
            upper_right = upper_left + 1
            faces.extend(((lower_left, lower_right, upper_right), (lower_left, upper_right, upper_left)))
    return _mesh_from_arrays(vertices, np.asarray(faces), None, uv, material, label)


def _attribute_mapping(position_indices: np.ndarray, attribute_indices: np.ndarray, count: int) -> np.ndarray | None:
    """Return position→attribute mapping only if it has no per-face seams."""
    mapping = np.full(count, -1, dtype=np.int64)
    mapping[position_indices] = attribute_indices
    if np.array_equal(mapping[position_indices], attribute_indices):
        return mapping
    return None


def _mesh_geometry(
    model: Any,
    geom_id: int,
    mesh_id: int,
    binding: _MaterialBinding,
) -> trimesh.Trimesh:
    label = f"geom {geom_id} mesh {mesh_id}"
    if mesh_id < 0 or mesh_id >= int(model.nmesh):
        raise SceneError(f"{label} references an invalid mesh")
    vertex_start = int(model.mesh_vertadr[mesh_id])
    vertex_count = int(model.mesh_vertnum[mesh_id])
    face_start = int(model.mesh_faceadr[mesh_id])
    face_count = int(model.mesh_facenum[mesh_id])
    vertices = np.asarray(model.mesh_vert[vertex_start:vertex_start + vertex_count], dtype=np.float64)
    faces = np.asarray(model.mesh_face[face_start:face_start + face_count], dtype=np.int64)
    if vertices.shape != (vertex_count, 3) or faces.shape != (face_count, 3):
        raise SceneError(f"{label} has inconsistent compiled mesh arrays")
    if (not np.all(np.isfinite(vertices)) or np.any(faces < 0) or np.any(faces >= vertex_count)):
        raise SceneError(f"{label} has invalid compiled vertices or faces")

    face_positions = faces.reshape(-1)
    attributes: list[np.ndarray] = [face_positions]
    normal_values: np.ndarray | None = None
    normal_indices: np.ndarray | None = None
    normal_count = int(model.mesh_normalnum[mesh_id])
    if normal_count:
        normal_start = int(model.mesh_normaladr[mesh_id])
        normal_values = np.asarray(model.mesh_normal[normal_start:normal_start + normal_count], dtype=np.float64)
        normal_indices = np.asarray(
            model.mesh_facenormal[face_start:face_start + face_count], dtype=np.int64
        ).reshape(-1)
        if (normal_values.shape != (normal_count, 3) or normal_indices.shape != face_positions.shape
                or not np.all(np.isfinite(normal_values)) or np.any(normal_indices < 0)
                or np.any(normal_indices >= normal_count)):
            raise SceneError(f"{label} has invalid compiled normal indices")
        attributes.append(normal_indices)

    texcoord_values: np.ndarray | None = None
    texcoord_indices: np.ndarray | None = None
    if binding.needs_uv:
        texcoord_count = int(model.mesh_texcoordnum[mesh_id])
        if texcoord_count <= 0:
            raise SceneError(f"{label} needs UV coordinates for its material texture")
        texcoord_start = int(model.mesh_texcoordadr[mesh_id])
        texcoord_values = np.asarray(
            model.mesh_texcoord[texcoord_start:texcoord_start + texcoord_count], dtype=np.float64
        )
        texcoord_indices = np.asarray(
            model.mesh_facetexcoord[face_start:face_start + face_count], dtype=np.int64
        ).reshape(-1)
        if texcoord_values.shape != (texcoord_count, 2) or texcoord_indices.shape != face_positions.shape:
            raise SceneError(f"{label} has inconsistent compiled UV arrays")
        # MuJoCo uses texcoord_count as the no-coordinate sentinel.  Its own USD
        # exporter maps that sentinel to coordinate zero as well.
        if np.any(texcoord_indices == texcoord_count):
            texcoord_indices = texcoord_indices.copy()
            texcoord_indices[texcoord_indices == texcoord_count] = 0
        if (not np.all(np.isfinite(texcoord_values)) or np.any(texcoord_indices < 0)
                or np.any(texcoord_indices >= texcoord_count)):
            raise SceneError(f"{label} has invalid compiled UV indices")
        attributes.append(texcoord_indices)

    mappings = [_attribute_mapping(face_positions, values, vertex_count) for values in attributes[1:]]
    if all(mapping is not None for mapping in mappings):
        output_vertices = vertices
        output_faces = faces
        next_mapping = 0
        if normal_values is not None:
            output_normals = normal_values[mappings[next_mapping]]
            next_mapping += 1
        else:
            output_normals = None
        if texcoord_values is not None:
            output_uv = texcoord_values[mappings[next_mapping]] * np.asarray(binding.uv_scale)
        else:
            output_uv = None
    else:
        corner_attributes = np.column_stack(attributes)
        unique, inverse = np.unique(corner_attributes, axis=0, return_inverse=True)
        output_vertices = vertices[unique[:, 0]]
        output_faces = inverse.reshape((-1, 3))
        column = 1
        if normal_values is not None:
            output_normals = normal_values[unique[:, column]]
            column += 1
        else:
            output_normals = None
        if texcoord_values is not None:
            output_uv = texcoord_values[unique[:, column]] * np.asarray(binding.uv_scale)
        else:
            output_uv = None
    return _mesh_from_arrays(
        output_vertices, output_faces, output_normals, output_uv, binding.material, label
    )


class _MaterialFactory:
    """Decode only textures actually referenced by visible GLB materials."""

    def __init__(self, model: Any) -> None:
        self.model = model
        self._images: dict[int, Image.Image] = {}
        self._bindings: dict[tuple[Any, ...], _MaterialBinding] = {}
        self._role = {
            name: int(getattr(mujoco.mjtTextureRole, name).value)
            for name in (
                "mjTEXROLE_RGB", "mjTEXROLE_RGBA", "mjTEXROLE_NORMAL", "mjTEXROLE_OCCLUSION",
                "mjTEXROLE_EMISSIVE", "mjTEXROLE_ORM", "mjTEXROLE_OPACITY",
                "mjTEXROLE_METALLIC", "mjTEXROLE_ROUGHNESS",
            )
            if hasattr(mujoco.mjtTextureRole, name)
        }

    def _texture_id(self, material_id: int, role: str) -> int:
        column = self._role.get(role)
        if column is None:
            return -1
        texids = np.asarray(self.model.mat_texid[material_id])
        if column >= len(texids):
            raise SceneError(f"material {material_id} lacks texture role column {role}")
        texture_id = int(texids[column])
        if texture_id < -1 or texture_id >= int(self.model.ntex):
            raise SceneError(f"material {material_id} has invalid texture id for {role}")
        return texture_id

    def _image(self, texture_id: int) -> Image.Image:
        if texture_id in self._images:
            return self._images[texture_id]
        if texture_id < 0 or texture_id >= int(self.model.ntex):
            raise SceneError(f"invalid texture id {texture_id}")
        texture_type = int(self.model.tex_type[texture_id])
        if texture_type != int(mujoco.mjtTexture.mjTEXTURE_2D):
            try:
                name = mujoco.mjtTexture(texture_type).name
            except ValueError:
                name = str(texture_type)
            raise SceneError(f"texture {texture_id} uses unsupported texture type {name}")
        width, height, channels = (
            int(self.model.tex_width[texture_id]),
            int(self.model.tex_height[texture_id]),
            int(self.model.tex_nchannel[texture_id]),
        )
        modes = {1: "L", 2: "LA", 3: "RGB", 4: "RGBA"}
        if width <= 0 or height <= 0 or channels not in modes:
            raise SceneError(f"texture {texture_id} has unsupported dimensions or channel count")
        start = int(self.model.tex_adr[texture_id])
        count = width * height * channels
        pixels = np.asarray(self.model.tex_data[start:start + count], dtype=np.uint8)
        if pixels.shape != (count,):
            raise SceneError(f"texture {texture_id} data is truncated")
        # MJB texture rows use OpenGL's lower-left origin.  Flip into the PNG
        # convention; trimesh performs the corresponding GLTF UV conversion.
        image = ImageOps.flip(Image.frombytes(modes[channels], (width, height), pixels.tobytes()))
        self._images[texture_id] = image
        return image

    @staticmethod
    def _factor(value: Any, fallback: float) -> float:
        number = float(value)
        if not np.isfinite(number):
            raise SceneError("material factor is not finite")
        return float(np.clip(number, 0.0, 1.0)) if np.isfinite(number) else fallback

    def for_geom(self, geom_id: int) -> _MaterialBinding:
        material_id = int(self.model.geom_matid[geom_id])
        if material_id < -1 or material_id >= int(self.model.nmat):
            raise SceneError(f"geom {geom_id} has invalid material id")
        if material_id < 0:
            rgba = _finite_vector(self.model.geom_rgba[geom_id], 4, f"geom {geom_id} rgba")
            rgba = np.clip(rgba, 0.0, 1.0)
            key: tuple[Any, ...] = ("geom", tuple(float(value) for value in rgba))
            repeat = (1.0, 1.0)
            source = None
        else:
            rgba = _finite_vector(self.model.mat_rgba[material_id], 4, f"material {material_id} rgba")
            rgba = np.clip(rgba, 0.0, 1.0)
            repeat_values = _finite_vector(
                (self.model.mat_texrepeat[material_id, 0], self.model.mat_texrepeat[material_id, 1], 1.0),
                3,
                f"material {material_id} texture repeat",
            )[:2]
            if np.any(repeat_values <= 0):
                raise SceneError(f"material {material_id} texture repeat must be positive")
            repeat = (float(repeat_values[0]), float(repeat_values[1]))
            source = material_id
            key = ("material", material_id)
        existing = self._bindings.get(key)
        if existing is not None:
            return existing

        base_texture = normal_texture = occlusion_texture = emissive_texture = orm_texture = None
        if source is not None:
            rgb = self._texture_id(source, "mjTEXROLE_RGB")
            rgba_texture = self._texture_id(source, "mjTEXROLE_RGBA")
            base_id = rgb if rgb >= 0 else rgba_texture
            normal_id = self._texture_id(source, "mjTEXROLE_NORMAL")
            occlusion_id = self._texture_id(source, "mjTEXROLE_OCCLUSION")
            emissive_id = self._texture_id(source, "mjTEXROLE_EMISSIVE")
            orm_id = self._texture_id(source, "mjTEXROLE_ORM")
            unsupported = {
                "opacity": self._texture_id(source, "mjTEXROLE_OPACITY"),
                "metallic": self._texture_id(source, "mjTEXROLE_METALLIC"),
                "roughness": self._texture_id(source, "mjTEXROLE_ROUGHNESS"),
            }
            active_unsupported = [name for name, texture_id in unsupported.items() if texture_id >= 0]
            if active_unsupported:
                joined = ", ".join(active_unsupported)
                raise SceneError(f"material {source} uses unsupported independent texture roles: {joined}")
            if base_id >= 0:
                base_texture = self._image(base_id)
            if normal_id >= 0:
                normal_texture = self._image(normal_id)
            if occlusion_id >= 0:
                occlusion_texture = self._image(occlusion_id)
            if emissive_id >= 0:
                emissive_texture = self._image(emissive_id)
            if orm_id >= 0:
                orm_texture = self._image(orm_id)
            emission = self._factor(self.model.mat_emission[source], 0.0)
            metallic = self._factor(self.model.mat_metallic[source], 0.0)
            roughness = self._factor(self.model.mat_roughness[source], 1.0)
        else:
            emission, metallic, roughness = 0.0, 0.0, 1.0

        alpha_mode = "BLEND" if rgba[3] < 1.0 else "OPAQUE"
        material = PBRMaterial(
            name=f"material_{source}" if source is not None else f"geom_{geom_id}",
            baseColorFactor=rgba,
            baseColorTexture=base_texture,
            normalTexture=normal_texture,
            occlusionTexture=occlusion_texture,
            emissiveTexture=emissive_texture,
            metallicRoughnessTexture=orm_texture,
            emissiveFactor=np.clip(rgba[:3] * emission, 0.0, 1.0),
            metallicFactor=metallic,
            roughnessFactor=roughness,
            alphaMode=alpha_mode,
            doubleSided=True,
        )
        binding = _MaterialBinding(
            material=material,
            key=key,
            needs_uv=any(texture is not None for texture in (
                base_texture, normal_texture, occlusion_texture, emissive_texture, orm_texture,
            )),
            uv_scale=repeat,
        )
        self._bindings[key] = binding
        return binding


def _geometry_for_geom(model: Any, geom_id: int, binding: _MaterialBinding) -> trimesh.Trimesh:
    geom_type = int(model.geom_type[geom_id])
    size = _finite_vector(model.geom_size[geom_id], 3, f"geom {geom_id} size")
    label = f"geom {geom_id}"
    if geom_type == int(mujoco.mjtGeom.mjGEOM_PLANE):
        return _plane_mesh(size, binding.material, binding.uv_scale, label)
    if geom_type == int(mujoco.mjtGeom.mjGEOM_HFIELD):
        return _hfield_mesh(model, int(model.geom_dataid[geom_id]), binding.material, binding.uv_scale, label)
    if geom_type == int(mujoco.mjtGeom.mjGEOM_SPHERE):
        radius = _positive_sizes(size, 1, label)[0]
        return _latlong_mesh(np.asarray((radius, radius, radius)), binding.material, binding.uv_scale, label)
    if geom_type == int(mujoco.mjtGeom.mjGEOM_CAPSULE):
        return _capsule_mesh(size, binding.material, binding.uv_scale, label)
    if geom_type == int(mujoco.mjtGeom.mjGEOM_ELLIPSOID):
        return _latlong_mesh(size, binding.material, binding.uv_scale, label)
    if geom_type == int(mujoco.mjtGeom.mjGEOM_CYLINDER):
        return _cylinder_mesh(size, binding.material, binding.uv_scale, label)
    if geom_type == int(mujoco.mjtGeom.mjGEOM_BOX):
        return _box_mesh(size, binding.material, binding.uv_scale, label)
    if geom_type == int(mujoco.mjtGeom.mjGEOM_MESH):
        return _mesh_geometry(model, geom_id, int(model.geom_dataid[geom_id]), binding)
    try:
        name = mujoco.mjtGeom(geom_type).name
    except ValueError:
        name = str(geom_type)
    raise SceneError(f"{label} uses unsupported MuJoCo geom type {name}")


def _geometry_key(model: Any, geom_id: int, binding: _MaterialBinding) -> tuple[Any, ...]:
    geom_type = int(model.geom_type[geom_id])
    if geom_type in (int(mujoco.mjtGeom.mjGEOM_MESH), int(mujoco.mjtGeom.mjGEOM_HFIELD)):
        shape_key: tuple[Any, ...] = (int(model.geom_dataid[geom_id]),)
    else:
        shape_key = tuple(float(value) for value in np.asarray(model.geom_size[geom_id]))
    return (geom_type, shape_key, binding.key, binding.uv_scale, binding.needs_uv)


def _extend_bounds(lower: np.ndarray | None, upper: np.ndarray | None, mesh: trimesh.Trimesh, transform: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    bounds = np.asarray(mesh.bounds, dtype=np.float64)
    if bounds.shape != (2, 3) or not np.all(np.isfinite(bounds)):
        raise SceneError("exported geometry has invalid bounds")
    corners = np.asarray([
        (x, y, z)
        for x in (bounds[0, 0], bounds[1, 0])
        for y in (bounds[0, 1], bounds[1, 1])
        for z in (bounds[0, 2], bounds[1, 2])
    ], dtype=np.float64)
    world = (transform[:3, :3] @ corners.T).T + transform[:3, 3]
    if not np.all(np.isfinite(world)):
        raise SceneError("exported geometry has non-finite world bounds")
    world_lower, world_upper = np.min(world, axis=0), np.max(world, axis=0)
    return (
        world_lower if lower is None else np.minimum(lower, world_lower),
        world_upper if upper is None else np.maximum(upper, world_upper),
    )


def export_scene(model: Any, data: Any, metadata: Mapping[str, Any]) -> tuple[bytes, list[int], list[float]]:
    """Export visible MuJoCo geometry with direct world-body animation nodes.

    ``data`` must already contain frame-zero kinematics.  No forward dynamics or
    rendering is performed here; only compiled visual geometry is converted.
    """
    if not isinstance(metadata, Mapping):
        raise SceneError("model metadata must be a mapping")
    if int(model.nbody) <= 0 or int(model.ngeom) < 0:
        raise SceneError("compiled model has invalid body or geom counts")
    visible_geoms = [
        geom_id for geom_id in range(int(model.ngeom))
        if int(model.geom_group[geom_id]) in _VISIBLE_GROUPS
    ]
    if not visible_geoms:
        raise SceneError("model has no visible geoms in MuJoCo groups 1 or 2")
    body_ids = sorted({int(model.geom_bodyid[geom_id]) for geom_id in visible_geoms})
    if any(body_id < 0 or body_id >= int(model.nbody) for body_id in body_ids):
        raise SceneError("visible geom references an invalid body")

    scene = trimesh.Scene(base_frame="world")
    body_transforms: dict[int, np.ndarray] = {}
    for body_id in body_ids:
        world_transform = _transform(data.xpos[body_id], data.xquat[body_id], f"body {body_id}")
        body_transforms[body_id] = world_transform
        scene.graph.update(frame_to=f"body_{body_id}", frame_from="world", matrix=world_transform)

    material_factory = _MaterialFactory(model)
    geometry_cache: dict[tuple[Any, ...], tuple[str, trimesh.Trimesh]] = {}
    lower: np.ndarray | None = None
    upper: np.ndarray | None = None
    for geom_id in visible_geoms:
        body_id = int(model.geom_bodyid[geom_id])
        binding = material_factory.for_geom(geom_id)
        key = _geometry_key(model, geom_id, binding)
        cached = geometry_cache.get(key)
        if cached is None:
            mesh = _geometry_for_geom(model, geom_id, binding)
            geometry_name = f"geometry_{len(geometry_cache)}"
            geometry_cache[key] = (geometry_name, mesh)
        else:
            geometry_name, mesh = cached
        local_transform = _transform(model.geom_pos[geom_id], model.geom_quat[geom_id], f"geom {geom_id}")
        node_name = f"geom_{geom_id}"
        if cached is None:
            scene.add_geometry(
                mesh,
                node_name=node_name,
                geom_name=geometry_name,
                parent_node_name=f"body_{body_id}",
                transform=local_transform,
            )
        else:
            scene.graph.update(
                frame_to=node_name,
                frame_from=f"body_{body_id}",
                matrix=local_transform,
                geometry=geometry_name,
            )
        lower, upper = _extend_bounds(lower, upper, mesh, body_transforms[body_id] @ local_transform)

    if lower is None or upper is None:
        raise SceneError("model has no exportable visible geometry")
    center = (lower + upper) * 0.5
    if not np.all(np.isfinite(center)):
        raise SceneError("scene center is not finite")
    try:
        glb = export_glb(scene, include_normals=True)
    except (ValueError, TypeError, OSError) as exc:
        raise SceneError(f"could not export GLB: {exc}") from exc
    if not isinstance(glb, bytes) or len(glb) < 20 or glb[:4] != b"glTF":
        raise SceneError("trimesh did not produce a valid GLB payload")
    return glb, body_ids, [float(value) for value in center]


__all__ = ["SceneError", "export_scene"]
