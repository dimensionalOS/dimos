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

"""Complete simulated lidar map sampled from the house's physical surfaces."""

from pathlib import Path

import mujoco
import numpy as np
from numpy.typing import NDArray

from dimos.robot.galaxea.r1pro.packing_sim import PACKING_BODIES
from dimos.simulation.perception.mujoco_surface import (
    _mesh_arrays,
    _sample_geom_local,
    voxel_downsample,
)


def environment_cloud(
    model: mujoco.MjModel, data: mujoco.MjData, spacing: float = 0.04
) -> NDArray[np.float32]:
    """Sample collision surfaces, including floors; exclude robot and carried cargo.

    This is a complete, static, world-frame map, without sensor occlusion or noise.
    The point cloud describes physical surfaces, never a precomputed route.
    """
    if not 0.01 <= spacing <= 0.1:
        raise ValueError("Map spacing must be between 1 and 10 cm")
    excluded = {model.body(name).id for name in ("base_link", "task_bin", *PACKING_BODIES)}
    for body in range(model.nbody):
        if int(model.body_parentid[body]) in excluded:
            excluded.add(body)
    rng = np.random.default_rng(0)
    chunks = []
    for geom in range(model.ngeom):
        if int(model.geom_bodyid[geom]) in excluded:
            continue
        if not (model.geom_contype[geom] or model.geom_conaffinity[geom]):
            continue
        kind = model.geom_type[geom]
        half = model.geom_size[geom]
        if kind == mujoco.mjtGeom.mjGEOM_BOX:
            faces = []
            for axis in range(3):
                others = [i for i in range(3) if i != axis]
                u, v = np.meshgrid(
                    *[
                        np.linspace(
                            -half[i], half[i], max(2, int(np.ceil(2 * half[i] / spacing)) + 1)
                        )
                        for i in others
                    ]
                )
                for sign in (-1, 1):
                    face = np.zeros((u.size, 3))
                    face[:, axis] = sign * half[axis]
                    face[:, others[0]], face[:, others[1]] = u.ravel(), v.ravel()
                    faces.append(face)
            local = np.vstack(faces)
        elif kind == mujoco.mjtGeom.mjGEOM_MESH:
            vertices, mesh_faces = _mesh_arrays(model, int(model.geom_dataid[geom]))
            triangles = vertices[mesh_faces]
            area = float(
                np.linalg.norm(
                    np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0]),
                    axis=1,
                ).sum()
                / 2
            )
            local = _sample_geom_local(
                model, geom, max(32, int(np.ceil(5 * area / spacing**2))), rng
            )
        elif kind == mujoco.mjtGeom.mjGEOM_PLANE:
            raise ValueError("This map requires bounded floor geometry, not an infinite plane")
        else:
            radius = float(np.max(half))
            local = _sample_geom_local(model, geom, max(32, int(20 * radius**2 / spacing**2)), rng)
        rotation = data.geom_xmat[geom].reshape(3, 3)
        chunks.append(local @ rotation.T + data.geom_xpos[geom])
    if not chunks:
        raise ValueError("Scene contains no environment collision surfaces")
    return voxel_downsample(np.vstack(chunks), spacing).astype(np.float32)


def save_environment_cloud(scene: Path, output: Path, spacing: float = 0.04) -> Path:
    model = mujoco.MjModel.from_xml_path(str(scene))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    points = environment_cloud(model, data, spacing)
    np.save(output, points)
    return output
