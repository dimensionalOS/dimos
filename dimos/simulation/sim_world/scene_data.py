# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


"""The sim's world outside the robot as dimos.scene_data.v2 files: a JSON index that names its meshes (STL) and textures (PNG)."""

from __future__ import annotations

import io
import json
import re
import struct
from typing import Any

import mujoco
import numpy as np
from numpy.typing import NDArray
from PIL import Image

from dimos.simulation.sim_world.body import COLLISION_GROUP

SCENE_DATA_FORMAT = "dimos.scene_data.v2"
SCENE_FILE = "sim_scene.json"
_GEOM_TYPES = {
    int(mujoco.mjtGeom.mjGEOM_PLANE): "plane",
    int(mujoco.mjtGeom.mjGEOM_SPHERE): "sphere",
    int(mujoco.mjtGeom.mjGEOM_CAPSULE): "capsule",
    int(mujoco.mjtGeom.mjGEOM_ELLIPSOID): "ellipsoid",  # type: ignore[attr-defined]
    int(mujoco.mjtGeom.mjGEOM_CYLINDER): "cylinder",
    int(mujoco.mjtGeom.mjGEOM_BOX): "box",
    int(mujoco.mjtGeom.mjGEOM_MESH): "mesh",
}


def _xyzw(wxyz: NDArray[np.float64]) -> list[float]:
    w, x, y, z = (float(value) for value in wxyz)
    return [x, y, z, w]


def _file_name(kind: str, name: str, suffix: str) -> str:
    return f"sim_scene/{kind}/{re.sub(r'[^A-Za-z0-9_.-]', '_', name)}{suffix}"


def _stl(model: mujoco.MjModel, mesh: int) -> bytes:
    """The mesh as binary STL, in its own frame."""
    vertices = model.mesh_vert[
        model.mesh_vertadr[mesh] : model.mesh_vertadr[mesh] + model.mesh_vertnum[mesh]
    ].astype(np.float32)
    faces = model.mesh_face[
        model.mesh_faceadr[mesh] : model.mesh_faceadr[mesh] + model.mesh_facenum[mesh]
    ]
    triangles = vertices[faces]
    normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    lengths = np.linalg.norm(normals, axis=1, keepdims=True)
    normals = (normals / np.where(lengths > 0, lengths, 1)).astype(np.float32)
    record = np.dtype([("normal", "<f4", 3), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")])
    records = np.zeros(len(faces), dtype=record)
    records["normal"], records["vertices"] = normals, triangles
    return b"\0" * 80 + struct.pack("<I", len(faces)) + records.tobytes()


def _png(model: mujoco.MjModel, texture: int) -> bytes:
    """The texture as PNG, its rows in MuJoCo's order."""
    width, height, channels = (
        int(model.tex_width[texture]),
        int(model.tex_height[texture]),
        int(model.tex_nchannel[texture]),
    )
    start = int(model.tex_adr[texture])
    pixels = np.asarray(model.tex_data[start : start + width * height * channels], dtype=np.uint8)
    mode = {1: "L", 3: "RGB", 4: "RGBA"}[channels]
    shape = (height, width) if channels == 1 else (height, width, channels)
    out = io.BytesIO()
    Image.fromarray(pixels.reshape(shape), mode).save(out, format="PNG")
    return out.getvalue()


def scene_files(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    scene_id: str,
    frame: str,
    robot_root_body: int,
) -> dict[str, bytes]:
    """Every body but the robot's (it is drawn from its URDF): SCENE_FILE and each mesh and texture file it names."""
    bodies: list[dict[str, Any]] = []
    meshes: dict[str, dict[str, str]] = {}
    textures: dict[str, dict[str, Any]] = {}
    files: dict[str, bytes] = {}
    for body in range(model.nbody):
        if body and model.body_rootid[body] == robot_root_body:
            continue
        geoms = []
        first = model.body_geomadr[body]
        for geom in range(first, first + model.body_geomnum[body]):
            kind = _GEOM_TYPES.get(int(model.geom_type[geom]))
            if kind is None:
                continue
            # Group 3 is contact geometry, which MuJoCo's own viewer hides and nothing should
            # draw. A generated scene has none, so this used to make no difference; a converted
            # room has it in bulk -- office_555 carries 3083 convex pieces behind its 464 visual
            # meshes, so sending them would be seven times the payload for the wrong picture.
            if int(model.geom_group[geom]) == COLLISION_GROUP:
                continue
            material = int(model.geom_matid[geom])
            rgba = model.mat_rgba[material] if material >= 0 else model.geom_rgba[geom]
            entry: dict[str, Any] = {
                "name": model.geom(geom).name or f"geom{geom}",
                "type": kind,
                "size": model.geom_size[geom].tolist(),
                "pos": model.geom_pos[geom].tolist(),
                "quat": _xyzw(model.geom_quat[geom]),
                "rgba": rgba.tolist(),
            }
            if kind == "mesh":
                mesh = int(model.geom_dataid[geom])
                entry["mesh"] = name = model.mesh(mesh).name or f"mesh{mesh}"
                if name not in meshes:
                    meshes[name] = {"file": _file_name("meshes", name, ".stl")}
                    files[meshes[name]["file"]] = _stl(model, mesh)
            if material >= 0:
                entry["material"] = model.material(material).name
                texture = int(model.mat_texid[material, mujoco.mjtTextureRole.mjTEXROLE_RGB])
                if texture >= 0:
                    entry["texture"] = name = model.texture(texture).name or f"texture{texture}"
                    if name not in textures:
                        file = _file_name("textures", name, ".png")
                        textures[name] = {
                            "file": file,
                            "width": int(model.tex_width[texture]),
                            "height": int(model.tex_height[texture]),
                            "channels": int(model.tex_nchannel[texture]),
                        }
                        files[file] = _png(model, texture)
                    # how many times the texture repeats: per meter when uniform, else per face
                    entry["texrepeat"] = model.mat_texrepeat[material].tolist()
                    entry["texuniform"] = bool(model.mat_texuniform[material])
            geoms.append(entry)
        if geoms:
            name = model.body(body).name or "world"
            bodies.append(
                {
                    "name": name,
                    "label": name,
                    "pos": data.xpos[body].tolist(),
                    "quat": _xyzw(data.xquat[body]),
                    "mocap": bool(model.body_mocapid[body] >= 0),
                    "geoms": geoms,
                    "lights": [],
                }
            )
    files[SCENE_FILE] = json.dumps(
        {
            "format": SCENE_DATA_FORMAT,
            "id": scene_id,
            "frame": frame,
            "bodies": bodies,
            "meshes": meshes,
            "hfields": {},
            "textures": textures,
        }
    ).encode()
    return files
