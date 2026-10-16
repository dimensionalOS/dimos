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

"""Ray casts against a large static MuJoCo scene, through Embree.

``mj_multiRay`` tests every ray against every geom, which is fine for a few hundred
boxes and hundreds of times too slow for a cooked scene package (tens of thousands of
geoms, millions of mesh triangles), even when a group mask leaves only the robot. The
static world never moves, so it is triangulated once into an Open3D ray casting scene;
moving bodies go through MuJoCo in a copy of the model without the scene, whose
joints mirror the full model's.
"""

from __future__ import annotations

import mujoco
import numpy as np
from numpy.typing import NDArray
import open3d as o3d

from dimos.simulation.sensors.mujoco_raycaster import MujocoRaycaster

WORLD = 0
# Facets around a round primitive; the lidar's own noise is larger than the faceting error.
ROUND_RESOLUTION = 12


def _primitive(geom_type: int, size: NDArray[np.float64]) -> o3d.geometry.TriangleMesh | None:
    """A centred triangle mesh for a primitive geom, in its own frame."""
    if geom_type == mujoco.mjtGeom.mjGEOM_BOX:
        mesh = o3d.geometry.TriangleMesh.create_box(*(2 * size[:3]))
        return mesh.translate(-size[:3])
    if geom_type == mujoco.mjtGeom.mjGEOM_SPHERE:
        return o3d.geometry.TriangleMesh.create_sphere(size[0], resolution=ROUND_RESOLUTION // 2)
    if geom_type == mujoco.mjtGeom.mjGEOM_CYLINDER:
        return o3d.geometry.TriangleMesh.create_cylinder(size[0], 2 * size[1], ROUND_RESOLUTION)
    if geom_type == mujoco.mjtGeom.mjGEOM_CAPSULE:
        # A cylinder covering the caps; the ends are a few centimetres too square.
        return o3d.geometry.TriangleMesh.create_cylinder(
            size[0], 2 * (size[1] + size[0]), ROUND_RESOLUTION
        )
    return None


def static_triangles(
    model: mujoco.MjModel, data: mujoco.MjData, geom_groups: NDArray[np.uint8]
) -> tuple[NDArray[np.float32], NDArray[np.uint32]]:
    """World-frame vertices and faces of every geom welded to the world, in *geom_groups*."""
    vertices: list[NDArray[np.float64]] = []
    faces: list[NDArray[np.int64]] = []
    count = 0
    for i in range(model.ngeom):
        static = model.body_weldid[model.geom_bodyid[i]] == WORLD
        if not static or not geom_groups[model.geom_group[i]]:
            continue
        if model.geom_type[i] == mujoco.mjtGeom.mjGEOM_MESH:
            mesh = model.geom_dataid[i]
            v0, nv = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
            f0, nf = model.mesh_faceadr[mesh], model.mesh_facenum[mesh]
            local, face = model.mesh_vert[v0 : v0 + nv], model.mesh_face[f0 : f0 + nf]
        else:
            primitive = _primitive(model.geom_type[i], model.geom_size[i])
            if primitive is None:
                continue  # planes and height fields: none in cooked packages
            local, face = np.asarray(primitive.vertices), np.asarray(primitive.triangles)
        rotation = data.geom_xmat[i].reshape(3, 3)
        vertices.append(local @ rotation.T + data.geom_xpos[i])
        faces.append(face + count)
        count += len(local)
    if not vertices:
        return np.zeros((0, 3), np.float32), np.zeros((0, 3), np.uint32)
    return (
        np.concatenate(vertices).astype(np.float32),
        np.concatenate(faces).astype(np.uint32),
    )


class StaticSceneRaycaster:
    """The static world through Embree, everything that moves through MuJoCo; nearest hit wins.

    *moving_model* is *model* without its static scene, with the same joints, so its
    state follows *data*'s qpos. None casts against the static scene alone.
    """

    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        geom_groups: NDArray[np.uint8],
        moving_model: mujoco.MjModel | None = None,
    ) -> None:
        mujoco.mj_forward(model, data)
        vertices, faces = static_triangles(model, data, geom_groups)
        self._scene = o3d.t.geometry.RaycastingScene()
        if len(faces):
            self._scene.add_triangles(o3d.core.Tensor(vertices), o3d.core.Tensor(faces))
        self._data = data
        self._moving: MujocoRaycaster | None = None
        if moving_model is not None:
            if moving_model.nq != model.nq:
                raise ValueError("moving_model must have the full model's joints")
            moving_data = mujoco.MjData(moving_model)
            self._moving = MujocoRaycaster(moving_model, moving_data, geom_groups)

    def cast(
        self, origin: NDArray[np.float64], directions: NDArray[np.float64], max_range: float
    ) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        """Distance and hit normal per world-frame unit direction. Distance is negative on a miss."""
        directions = np.asarray(directions, dtype=np.float64)
        rays = np.empty((len(directions), 6), np.float32)
        rays[:, :3] = origin
        rays[:, 3:] = directions
        hit = self._scene.cast_rays(o3d.core.Tensor(rays))
        dist = hit["t_hit"].numpy().astype(np.float64)
        normals = hit["primitive_normals"].numpy().astype(np.float64)
        dist[~np.isfinite(dist) | (dist > max_range)] = -1.0
        # Embree's normals follow the triangle winding; MuJoCo's face the ray.
        facing_away = np.einsum("ij,ij->i", normals, directions) > 0
        normals[facing_away] *= -1
        if self._moving is None:
            return dist, normals
        self._moving.data.qpos[:] = self._data.qpos
        mujoco.mj_forward(self._moving.model, self._moving.data)
        moving_dist, moving_normals = self._moving.cast(origin, directions, max_range)
        nearer = (moving_dist >= 0) & ((dist < 0) | (moving_dist < dist))
        dist[nearer] = moving_dist[nearer]
        normals[nearer] = moving_normals[nearer]
        return dist, normals
