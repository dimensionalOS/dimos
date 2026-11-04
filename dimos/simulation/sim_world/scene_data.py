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


"""The sim's world outside the robot as dimos.scene_data.v1 JSON, the format the Controller's simulated-world layer draws."""

from __future__ import annotations

import base64
import json
from typing import Any

import mujoco
import numpy as np
from numpy.typing import NDArray

SCENE_DATA_FORMAT = "dimos.scene_data.v1"
_GEOM_TYPES = {
    int(mujoco.mjtGeom.mjGEOM_PLANE): "plane",
    int(mujoco.mjtGeom.mjGEOM_SPHERE): "sphere",
    int(mujoco.mjtGeom.mjGEOM_CAPSULE): "capsule",
    int(mujoco.mjtGeom.mjGEOM_ELLIPSOID): "ellipsoid",  # type: ignore[attr-defined]
    int(mujoco.mjtGeom.mjGEOM_CYLINDER): "cylinder",
    int(mujoco.mjtGeom.mjGEOM_BOX): "box",
    int(mujoco.mjtGeom.mjGEOM_MESH): "mesh",
}


def _b64(array: NDArray[Any]) -> str:
    return base64.b64encode(array.tobytes()).decode()


def _xyzw(wxyz: NDArray[np.float64]) -> list[float]:
    w, x, y, z = (float(value) for value in wxyz)
    return [x, y, z, w]


def _mesh(model: mujoco.MjModel, mesh: int) -> dict[str, str]:
    vertices = model.mesh_vert[
        model.mesh_vertadr[mesh] : model.mesh_vertadr[mesh] + model.mesh_vertnum[mesh]
    ]
    faces = model.mesh_face[
        model.mesh_faceadr[mesh] : model.mesh_faceadr[mesh] + model.mesh_facenum[mesh]
    ]
    return {"vertices": _b64(vertices.astype("<f4")), "faces": _b64(faces.astype("<u4"))}


def _texture(model: mujoco.MjModel, texture: int) -> dict[str, Any]:
    width, height, channels = (
        int(model.tex_width[texture]),
        int(model.tex_height[texture]),
        int(model.tex_nchannel[texture]),
    )
    start = int(model.tex_adr[texture])
    pixels = model.tex_data[start : start + width * height * channels]
    return {"width": width, "height": height, "channels": channels, "pixels": _b64(pixels)}


def scene_data(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    scene_id: str,
    frame: str,
    robot_root_body: int,
) -> str:
    """Every body but the robot's (it is drawn from its URDF) with its geoms, meshes and textures."""
    bodies: list[dict[str, Any]] = []
    meshes: dict[str, dict[str, str]] = {}
    textures: dict[str, dict[str, Any]] = {}
    for body in range(model.nbody):
        if body and model.body_rootid[body] == robot_root_body:
            continue
        geoms = []
        first = model.body_geomadr[body]
        for geom in range(first, first + model.body_geomnum[body]):
            kind = _GEOM_TYPES.get(int(model.geom_type[geom]))
            if kind is None:
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
                meshes.setdefault(name, _mesh(model, mesh))
            if material >= 0:
                entry["material"] = model.material(material).name
                texture = int(model.mat_texid[material, mujoco.mjtTextureRole.mjTEXROLE_RGB])
                if texture >= 0:
                    entry["texture"] = name = model.texture(texture).name or f"texture{texture}"
                    textures.setdefault(name, _texture(model, texture))
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
    return json.dumps(
        {
            "format": SCENE_DATA_FORMAT,
            "id": scene_id,
            "frame": frame,
            "bodies": bodies,
            "meshes": meshes,
            "hfields": {},
            "textures": textures,
        }
    )
