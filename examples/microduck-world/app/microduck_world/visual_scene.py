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

"""Export compiled MuJoCo visuals and body poses for the project web viewer."""

import base64
import gzip
import hashlib
import json
from io import BytesIO
from pathlib import Path
from typing import Any, cast

import mujoco
import numpy as np
from numpy.typing import NDArray
from PIL import Image as PixelImage

# Scene geoms use group 0; robot visuals use group 2. Group 3 is collision-only.
VISIBLE_GROUPS = (0, 1, 2)
SUPPORTED_GEOMS = {"plane", "sphere", "capsule", "ellipsoid", "cylinder", "box", "mesh"}


def packed(values: NDArray[Any], dtype: str) -> str:
    """Little-endian buffers retain mesh precision without JSON float expansion."""
    return base64.b64encode(np.asarray(values, dtype=dtype).tobytes()).decode("ascii")


def export_scene(
    model: mujoco.MjModel, appearance: dict[str, Any], camera_name: str | None = None
) -> dict[str, Any]:
    geoms: list[dict[str, Any]] = []
    meshes: dict[str, Any] = {}
    textures: dict[str, str] = {}
    bodies: set[int] = {0}
    mesh_ids: dict[int, str] = {}
    mesh_hashes: dict[str, str] = {}
    for i in range(model.ngeom):
        if int(model.geom_group[i]) not in VISIBLE_GROUPS:
            continue
        material = int(model.geom_matid[i])
        rgba = model.mat_rgba[material] if material >= 0 else model.geom_rgba[i]
        if float(rgba[3]) == 0:
            continue
        kind = mujoco.mjtGeom(int(model.geom_type[i])).name.removeprefix("mjGEOM_").lower()
        if kind not in SUPPORTED_GEOMS:
            raise ValueError(f"3D viewer does not support visible geom type {kind!r}")
        body = int(model.geom_bodyid[i])
        bodies.add(body)
        geom: dict[str, Any] = {
            "id": i,
            "name": mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i) or f"geom_{i}",
            "body": body,
            "kind": kind,
            "size": model.geom_size[i].tolist(),
            "position": model.geom_pos[i].tolist(),
            "quaternion": model.geom_quat[i].tolist(),
            "rgba": rgba.tolist(),
        }
        if material >= 0 and kind in ("box", "plane") and not model.mat_texuniform[material]:
            texture = int(model.mat_texid[material, mujoco.mjtTextureRole.mjTEXROLE_RGB])
            if texture >= 0 and model.tex_type[texture] == mujoco.mjtTexture.mjTEXTURE_2D:
                key = str(texture)
                if key not in textures:
                    width, height = int(model.tex_width[texture]), int(model.tex_height[texture])
                    channels, start = int(model.tex_nchannel[texture]), int(model.tex_adr[texture])
                    pixels = model.tex_data[start : start + width * height * channels].reshape(
                        height, width, channels
                    )
                    buffer = BytesIO()
                    PixelImage.fromarray(pixels).save(buffer, format="PNG")
                    textures[key] = base64.b64encode(buffer.getvalue()).decode("ascii")
                geom["texture"] = {"id": key, "repeat": model.mat_texrepeat[material].tolist()}
        if kind == "mesh":
            mesh = int(model.geom_dataid[i])
            geom["part"] = model.mesh(mesh).name
            if mesh not in mesh_ids:
                va, vn = int(model.mesh_vertadr[mesh]), int(model.mesh_vertnum[mesh])
                fa, fn = int(model.mesh_faceadr[mesh]), int(model.mesh_facenum[mesh])
                buffers = {
                    "positions": packed(model.mesh_vert[va : va + vn], "<f4"),
                    "indices": packed(model.mesh_face[fa : fa + fn], "<u4"),
                }
                digest = hashlib.sha256(
                    (buffers["positions"] + buffers["indices"]).encode()
                ).hexdigest()
                key = mesh_hashes.setdefault(digest, str(mesh))
                meshes.setdefault(key, buffers)
                mesh_ids[mesh] = key
            geom["mesh"] = mesh_ids[mesh]
        geoms.append(geom)
    focus = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "trunk_base")
    if focus >= 0:
        bodies.add(focus)
    camera = None
    if camera_name is not None:
        camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
        if camera_id < 0:
            raise ValueError(f"Camera {camera_name!r} is missing from the world")
        body = int(model.cam_bodyid[camera_id])
        bodies.add(body)
        camera = {
            "body": body,
            "position": model.cam_pos[camera_id].tolist(),
            "quaternion": model.cam_quat[camera_id].tolist(),
            "fovy": float(model.cam_fovy[camera_id]),
            "near": float(model.vis.map.znear * model.stat.extent),
            "far": float(model.vis.map.zfar * model.stat.extent),
        }
    return {
        "camera": camera,
        "version": 1,
        "up": "z",
        "bodyIds": sorted(bodies),
        "focusBody": max(0, focus),
        "geoms": geoms,
        "meshes": meshes,
        "textures": textures,
        "appearance": appearance,
    }


def write_scene(scene: dict[str, Any], directory: Path) -> str:
    """Publish immutable assets; a new model receives a different URL."""
    data = json.dumps(scene, separators=(",", ":"), allow_nan=False).encode()
    digest = hashlib.sha256(data).hexdigest()[:20]
    name = f"scene-{digest}.json"
    directory.mkdir(parents=True, exist_ok=True)
    for suffix, content in (("", data), (".gz", gzip.compress(data, compresslevel=6, mtime=0))):
        path = directory / (name + suffix)
        temporary = path.with_suffix(path.suffix + ".tmp")
        temporary.write_bytes(content)
        temporary.replace(path)
    return name


def body_snapshot(data: mujoco.MjData, body_ids: list[int]) -> list[list[float]]:
    """Read on the physics thread, after a step; quaternion order is MuJoCo wxyz."""
    poses = np.concatenate((data.xpos[body_ids], data.xquat[body_ids]), axis=1)
    return cast(list[list[float]], np.round(poses, 6).tolist())
