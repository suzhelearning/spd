"""Immutable real MuJoCo geometry; called only by the simulation owner."""
from __future__ import annotations

import base64
from io import BytesIO
import math

import mujoco
import numpy as np
from PIL import Image


_GEOM_TYPES = {
    int(mujoco.mjtGeom.mjGEOM_PLANE): "plane",
    int(mujoco.mjtGeom.mjGEOM_SPHERE): "sphere",
    int(mujoco.mjtGeom.mjGEOM_CAPSULE): "capsule",
    int(mujoco.mjtGeom.mjGEOM_ELLIPSOID): "ellipsoid",
    int(mujoco.mjtGeom.mjGEOM_CYLINDER): "cylinder",
    int(mujoco.mjtGeom.mjGEOM_BOX): "box",
    int(mujoco.mjtGeom.mjGEOM_MESH): "mesh",
}
MAX_SCENE_BYTES = 256 * 1024 * 1024
MAX_TEXTURE_PIXELS = 16 * 1024 * 1024
_ARM_BODIES = frozenset(
    f"{link}_{side}" for side in ("L", "R")
    for link in ("Base", *(f"Link{i}" for i in range(1, 8)), "TCP_Link")
)


def copy_body_poses(model, data):
    """Absolute transforms, not body-local transforms or mesh asset transforms."""
    poses = np.empty((model.nbody, 7), dtype="<f4")
    poses[:, :3] = data.xpos
    poses[:, 3:6] = data.xquat[:, 1:4]
    poses[:, 6] = data.xquat[:, 0]
    poses[0] = (0., 0., 0., 0., 0., 0., 1.)
    if not np.isfinite(poses).all() or not math.isfinite(data.time):
        raise ValueError("nonfinite MuJoCo body state cannot be streamed")
    return poses


def _mesh(model, mesh_id):
    va, vn = int(model.mesh_vertadr[mesh_id]), int(model.mesh_vertnum[mesh_id])
    fa, fn = int(model.mesh_faceadr[mesh_id]), int(model.mesh_facenum[mesh_id])
    na, nn = int(model.mesh_normaladr[mesh_id]), int(model.mesh_normalnum[mesh_id])
    ta, tn = int(model.mesh_texcoordadr[mesh_id]), int(model.mesh_texcoordnum[mesh_id])
    faces = model.mesh_face[fa:fa + fn].reshape(-1)
    normal_faces = model.mesh_facenormal[fa:fa + fn].reshape(-1)
    if vn <= 0 or fn <= 0 or np.any(faces < 0) or np.any(faces >= vn):
        raise ValueError(f"mesh {mesh_id} has invalid triangle indices")
    has_normals = nn > 0 and np.all(normal_faces >= 0)
    if has_normals and np.any(normal_faces >= nn):
        raise ValueError(f"mesh {mesh_id} has invalid normal indices")
    columns = [faces]
    if has_normals:
        columns.append(normal_faces)
    if ta >= 0 and tn > 0:
        tex_faces = model.mesh_facetexcoord[fa:fa + fn].reshape(-1)
        if np.any(tex_faces < 0) or np.any(tex_faces >= tn):
            raise ValueError(f"mesh {mesh_id} has incomplete texture coordinates")
        columns.append(tex_faces)
    # MuJoCo/OBJ allow separate normal and UV indices, WebGL allows one index.
    # Split only vertices on actual seams, retaining indexed/shared geometry.
    corners, inverse = np.unique(np.column_stack(columns), axis=0, return_inverse=True)
    mesh = {
        "id": mesh_id,
        "positions": model.mesh_vert[va:va + vn][corners[:, 0]].reshape(-1).tolist(),
        "indices": inverse.tolist(),
    }
    if has_normals:
        mesh["normals"] = model.mesh_normal[na:na + nn][corners[:, 1]].reshape(-1).tolist()
    if ta >= 0 and tn > 0:
        mesh["uvs"] = model.mesh_texcoord[ta:ta + tn][corners[:, -1]].reshape(-1).tolist()
    return mesh


def _texture(model, texture_id):
    if int(model.tex_type[texture_id]) != int(mujoco.mjtTexture.mjTEXTURE_2D):
        raise ValueError(f"material texture {texture_id} is not 2D; cube material textures are unsupported")
    width, height = int(model.tex_width[texture_id]), int(model.tex_height[texture_id])
    channels = int(model.tex_nchannel[texture_id])
    if width <= 0 or height <= 0 or width * height > MAX_TEXTURE_PIXELS or channels not in (1, 2, 3, 4):
        raise ValueError(f"texture {texture_id} exceeds supported image dimensions/channels")
    start = int(model.tex_adr[texture_id])
    pixels = model.tex_data[start:start + width * height * channels].reshape(height, width, channels)
    if channels == 1:
        pixels = pixels[:, :, 0]
    stream = BytesIO()
    Image.fromarray(pixels).save(stream, format="PNG")
    texture = {"id": texture_id,
               "data_url": "data:image/png;base64," + base64.b64encode(stream.getvalue()).decode("ascii")}
    colorspace = int(model.tex_colorspace[texture_id])
    if colorspace == int(mujoco.mjtColorSpace.mjCOLORSPACE_LINEAR):
        texture["color_space"] = "linear"
    elif colorspace == int(mujoco.mjtColorSpace.mjCOLORSPACE_SRGB):
        texture["color_space"] = "srgb"
    elif colorspace != int(mujoco.mjtColorSpace.mjCOLORSPACE_AUTO):
        raise ValueError(f"texture {texture_id} has an unknown color space")
    return texture


def _material(model, material_id):
    roles = np.asarray(model.mat_texid[material_id]).reshape(-1)
    supported_roles = {int(mujoco.mjtTextureRole.mjTEXROLE_RGB), int(mujoco.mjtTextureRole.mjTEXROLE_RGBA)}
    textures = {int(texture) for role, texture in enumerate(roles) if texture >= 0 and role in supported_roles}
    if any(texture >= 0 and role not in supported_roles for role, texture in enumerate(roles)):
        raise ValueError(f"material {material_id} uses unsupported non-color texture maps")
    if len(textures) > 1:
        raise ValueError(f"material {material_id} has competing RGB/RGBA maps")
    texture_id = next(iter(textures), None)
    return {"id": material_id, "rgba": model.mat_rgba[material_id].tolist(),
            "texture_id": texture_id, "texrepeat": model.mat_texrepeat[material_id].tolist()}


def _view(model, data, height_m):
    origin = np.zeros(3)
    shoulders = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name) for name in ("Base_L", "Base_R")]
    if all(body_id >= 0 for body_id in shoulders):
        origin[:] = np.mean(data.xpos[shoulders], axis=0)
        # Place the operator's eyes above the real robot shoulder center, not at
        # an orbit camera. XR local-floor already contributes the user's height.
        origin[2] += .45 - height_m
    # XR right/up/back -> MuJoCo (-Z,-X,+Y); quaternion is xyzw.
    return {"position": origin.tolist(), "quaternion": [.5, -.5, -.5, .5]}


def export_scene(model, data, task, *, generation, height_m=1.7):
    """Export once per model generation; never retain live physics arrays.

    Compiled mesh vertices already include the compiler's scale/recentering.
    geom_pos/geom_quat are the corresponding compensated body-local placement;
    applying mesh_pos/mesh_quat again would double-transform the robot.
    """
    if model.nflex or model.nskin:
        raise ValueError("WebXR rigid scene export does not support flex or skinned geometry")
    if not math.isfinite(height_m) or not .8 <= height_m <= 2.5:
        raise ValueError("operator height must be between 0.8 and 2.5 meters")
    visual_bodies = set(int(model.geom_bodyid[i]) for i in range(model.ngeom) if model.geom_group[i] == 1)
    visible = []
    for geom_id in range(model.ngeom):
        group, body_id = int(model.geom_group[geom_id]), int(model.geom_bodyid[geom_id])
        if group == 3 or (group == 0 and body_id in visual_bodies):
            continue
        if int(model.geom_type[geom_id]) not in _GEOM_TYPES:
            raise ValueError(f"unsupported visible MuJoCo geom {geom_id}: type {int(model.geom_type[geom_id])}")
        visible.append(geom_id)

    # Use MuJoCo's own material/geom color precedence, without requiring GL.
    options = mujoco.MjvOption()
    options.geomgroup[:] = 1
    options.sitegroup[:] = 0
    camera = mujoco.MjvCamera()
    mujoco.mjv_defaultFreeCamera(model, camera)
    visual = mujoco.MjvScene(model, maxgeom=max(1000, model.ngeom + model.nbody + 100))
    mujoco.mjv_updateScene(model, data, options, None, camera, mujoco.mjtCatBit.mjCAT_ALL, visual)
    colors = {int(geom.objid): geom.rgba.copy() for geom in visual.geoms[:visual.ngeom]
              if geom.objtype == mujoco.mjtObj.mjOBJ_GEOM}
    mesh_ids, material_ids = set(), set()
    geoms = []
    for geom_id in visible:
        # MuJoCo excludes fully transparent geoms from its visual scene.
        if geom_id not in colors:
            continue
        mesh_id = int(model.geom_dataid[geom_id]) if model.geom_type[geom_id] == mujoco.mjtGeom.mjGEOM_MESH else None
        material_id = int(model.geom_matid[geom_id])
        if mesh_id is not None:
            mesh_ids.add(mesh_id)
        if material_id >= 0:
            material_ids.add(material_id)
        quaternion = model.geom_quat[geom_id]
        body_id = int(model.geom_bodyid[geom_id])
        if model.geom_group[geom_id] == 1 and model.body(body_id).name in _ARM_BODIES:
            # Operator-only transparency must never leak into the recorded MJB;
            # colors are private copies, and the hands stay naturally opaque.
            colors[geom_id][3] = .45
        geoms.append({
            "body_id": int(model.geom_bodyid[geom_id]), "type": _GEOM_TYPES[int(model.geom_type[geom_id])],
            "mesh_id": mesh_id, "position": model.geom_pos[geom_id].tolist(),
            "quaternion": [float(quaternion[1]), float(quaternion[2]), float(quaternion[3]), float(quaternion[0])],
            "size": model.geom_size[geom_id].tolist(), "rgba": colors[geom_id].tolist(),
            "material_id": material_id if material_id >= 0 else None,
        })
    materials = [_material(model, material_id) for material_id in sorted(material_ids)]
    texture_ids = {material["texture_id"] for material in materials if material["texture_id"] is not None}
    meshes = [_mesh(model, mesh_id) for mesh_id in sorted(mesh_ids)]
    textured_materials = {material["id"] for material in materials if material["texture_id"] is not None}
    mesh_has_uvs = {mesh["id"] for mesh in meshes if "uvs" in mesh}
    if any(geom["mesh_id"] is not None and geom["material_id"] in textured_materials
           and geom["mesh_id"] not in mesh_has_uvs for geom in geoms):
        raise ValueError("textured meshes require explicit UV coordinates for WebXR")
    return {
        "version": 1, "generation": generation,
        "task": {"title": str(task.get("title", ""))[:512], "goal": str(task.get("goal", ""))[:4096]},
        "bodies": [{"id": body_id, "name": model.body(body_id).name} for body_id in range(model.nbody)],
        "meshes": meshes, "textures": [_texture(model, texture_id) for texture_id in sorted(texture_ids)],
        "materials": materials, "geoms": geoms, "view": _view(model, data, height_m),
    }
