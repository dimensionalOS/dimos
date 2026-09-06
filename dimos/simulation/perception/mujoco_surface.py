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

"""Surface sampling over a compiled MuJoCo model.

Ground-truth stand-in for a depth camera: instead of reconstructing surfaces
from pixels, sample them off the geometry the simulator already has. Pure
functions on ``MjModel``/``MjData`` so they can be exercised without a running
engine.
"""

from __future__ import annotations

import mujoco
import numpy as np
from numpy.typing import NDArray

_MJOBJ_BODY = int(mujoco.mjtObj.mjOBJ_BODY)
_MJGEOM_PLANE = int(mujoco.mjtGeom.mjGEOM_PLANE)
_MJGEOM_SPHERE = int(mujoco.mjtGeom.mjGEOM_SPHERE)
_MJGEOM_CAPSULE = int(mujoco.mjtGeom.mjGEOM_CAPSULE)
_MJGEOM_ELLIPSOID = int(mujoco.mjtGeom.mjGEOM_ELLIPSOID)
_MJGEOM_CYLINDER = int(mujoco.mjtGeom.mjGEOM_CYLINDER)
_MJGEOM_BOX = int(mujoco.mjtGeom.mjGEOM_BOX)
_MJGEOM_MESH = int(mujoco.mjtGeom.mjGEOM_MESH)

# Planes are the ground and the walls: unbounded in MuJoCo, so they would
# swamp any sample budget without describing an obstacle anyone can hit.
_UNSAMPLED_TYPES = frozenset({_MJGEOM_PLANE})


def sample_body_surface(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    body_name: str,
    count: int = 512,
    *,
    rng: np.random.Generator | None = None,
) -> NDArray[np.float64]:
    """World-frame surface points of one rigid body; empty when it is unknown.

    Bodies welded to *body_name* count as part of it, matching
    ``MujocoEngine.get_body_geoms``.
    """
    body_id = mujoco.mj_name2id(model, _MJOBJ_BODY, body_name)
    if body_id < 0:
        return np.zeros((0, 3), dtype=np.float64)
    geom_ids = [
        geom_id
        for member in rigid_subtree(model, body_id)
        for geom_id in body_geom_ids(model, member)
    ]
    return sample_geom_surfaces(model, data, geom_ids, count, rng=rng)


def sample_scene_surface(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    exclude_bodies: tuple[str, ...] = (),
    *,
    voxel_size: float = 0.01,
    count: int = 20000,
    rng: np.random.Generator | None = None,
) -> NDArray[np.float64]:
    """World-frame surface points of everything except the excluded bodies."""
    excluded: set[int] = set()
    for name in exclude_bodies:
        body_id = mujoco.mj_name2id(model, _MJOBJ_BODY, name)
        if body_id >= 0:
            excluded.update(rigid_subtree(model, body_id))
    geom_ids = [
        geom_id
        for body_id in range(int(model.nbody))
        if body_id not in excluded
        for geom_id in body_geom_ids(model, body_id)
    ]
    points = sample_geom_surfaces(model, data, geom_ids, count, rng=rng)
    return voxel_downsample(points, voxel_size)


def rigid_subtree(model: mujoco.MjModel, body_id: int) -> list[int]:
    """*body_id* plus every descendant welded to it (no joint of its own)."""
    members = [body_id]
    for child in range(body_id + 1, int(model.nbody)):
        if int(model.body_parentid[child]) in members and int(model.body_jntnum[child]) == 0:
            members.append(child)
    return members


def body_geom_ids(model: mujoco.MjModel, body_id: int) -> range:
    start = int(model.body_geomadr[body_id])
    return range(start, start + int(model.body_geomnum[body_id]))


def sample_geom_surfaces(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    geom_ids: list[int] | range,
    count: int,
    *,
    rng: np.random.Generator | None = None,
) -> NDArray[np.float64]:
    """Sample *count* points spread over the named geoms, weighted by area."""
    rng = rng or np.random.default_rng(0)
    sampled = [
        geom_id for geom_id in geom_ids if int(model.geom_type[geom_id]) not in _UNSAMPLED_TYPES
    ]
    if not sampled or count <= 0:
        return np.zeros((0, 3), dtype=np.float64)

    areas = np.array([_geom_area(model, geom_id) for geom_id in sampled], dtype=np.float64)
    total = float(areas.sum())
    if total <= 0.0:
        shares = np.full(len(sampled), count // len(sampled) + 1)
    else:
        shares = np.maximum(1, np.round(count * areas / total).astype(int))

    chunks = []
    for geom_id, share in zip(sampled, shares, strict=True):
        local = _sample_geom_local(model, geom_id, int(share), rng)
        if local.size == 0:
            continue
        rotation = data.geom_xmat[geom_id].reshape(3, 3)
        chunks.append(local @ rotation.T + data.geom_xpos[geom_id])
    if not chunks:
        return np.zeros((0, 3), dtype=np.float64)
    points = np.vstack(chunks)
    if len(points) > count:
        points = points[rng.choice(len(points), count, replace=False)]
    return points


def voxel_downsample(points: NDArray[np.float64], voxel_size: float) -> NDArray[np.float64]:
    """Keep one point per occupied voxel, in first-seen order."""
    if voxel_size <= 0.0 or len(points) == 0:
        return points
    keys = np.floor(points / voxel_size).astype(np.int64)
    _, first = np.unique(keys, axis=0, return_index=True)
    return points[np.sort(first)]


def _geom_area(model: mujoco.MjModel, geom_id: int) -> float:
    """Rough surface area, used only to spread the sample budget."""
    geom_type = int(model.geom_type[geom_id])
    sx, sy, sz = (float(v) for v in model.geom_size[geom_id])
    if geom_type == _MJGEOM_SPHERE:
        return 4.0 * np.pi * sx**2
    if geom_type == _MJGEOM_BOX:
        return 8.0 * (sx * sy + sy * sz + sz * sx)
    if geom_type == _MJGEOM_CYLINDER:
        return 2.0 * np.pi * sx * (sx + 2.0 * sy)
    if geom_type == _MJGEOM_CAPSULE:
        return 2.0 * np.pi * sx * (2.0 * sx + 2.0 * sy)
    if geom_type == _MJGEOM_ELLIPSOID:
        return 4.0 * np.pi * ((sx * sy) ** 1.6 + (sy * sz) ** 1.6 + (sz * sx) ** 1.6) ** (1 / 1.6)
    if geom_type == _MJGEOM_MESH:
        mesh_id = int(model.geom_dataid[geom_id])
        return float(np.prod(_mesh_extent(model, mesh_id))) if mesh_id >= 0 else 0.0
    return 0.0


def _sample_geom_local(
    model: mujoco.MjModel, geom_id: int, count: int, rng: np.random.Generator
) -> NDArray[np.float64]:
    """Points on one geom's surface, in the geom's own frame."""
    geom_type = int(model.geom_type[geom_id])
    sx, sy, sz = (float(v) for v in model.geom_size[geom_id])
    if geom_type == _MJGEOM_SPHERE:
        return _unit_sphere(count, rng) * sx
    if geom_type == _MJGEOM_ELLIPSOID:
        return _unit_sphere(count, rng) * np.array([sx, sy, sz])
    if geom_type == _MJGEOM_BOX:
        return _box_surface(count, np.array([sx, sy, sz]), rng)
    if geom_type == _MJGEOM_CYLINDER:
        return _cylinder_surface(count, sx, sy, rng)
    if geom_type == _MJGEOM_CAPSULE:
        return _capsule_surface(count, sx, sy, rng)
    if geom_type == _MJGEOM_MESH:
        return _mesh_surface(model, int(model.geom_dataid[geom_id]), count, rng)
    return np.zeros((0, 3), dtype=np.float64)


def _unit_sphere(count: int, rng: np.random.Generator) -> NDArray[np.float64]:
    vectors = rng.normal(size=(count, 3))
    return vectors / np.linalg.norm(vectors, axis=1, keepdims=True)


def _box_surface(
    count: int, half: NDArray[np.float64], rng: np.random.Generator
) -> NDArray[np.float64]:
    # Pick a face with probability proportional to its area, then a uniform
    # point on it, so long thin boxes do not get over-sampled on the ends.
    face_areas = np.repeat(np.array([half[1] * half[2], half[0] * half[2], half[0] * half[1]]), 2)
    faces = rng.choice(6, size=count, p=face_areas / face_areas.sum())
    points = rng.uniform(-1.0, 1.0, size=(count, 3)) * half
    axis = faces // 2
    points[np.arange(count), axis] = np.where(faces % 2 == 0, half[axis], -half[axis])
    return points


def _cylinder_surface(
    count: int, radius: float, half_height: float, rng: np.random.Generator
) -> NDArray[np.float64]:
    side_area = 2.0 * np.pi * radius * 2.0 * half_height
    cap_area = 2.0 * np.pi * radius**2
    on_side = rng.random(count) < side_area / (side_area + cap_area)
    theta = rng.uniform(0.0, 2.0 * np.pi, size=count)
    r = radius * np.sqrt(rng.random(count))
    radial = np.where(on_side, radius, r)
    z = np.where(
        on_side,
        rng.uniform(-half_height, half_height, size=count),
        np.where(rng.random(count) < 0.5, half_height, -half_height),
    )
    return np.column_stack([radial * np.cos(theta), radial * np.sin(theta), z])


def _capsule_surface(
    count: int, radius: float, half_height: float, rng: np.random.Generator
) -> NDArray[np.float64]:
    side_area = 2.0 * np.pi * radius * 2.0 * half_height
    cap_area = 4.0 * np.pi * radius**2
    on_side = rng.random(count) < side_area / (side_area + cap_area)
    theta = rng.uniform(0.0, 2.0 * np.pi, size=count)
    side = np.column_stack(
        [
            radius * np.cos(theta),
            radius * np.sin(theta),
            rng.uniform(-half_height, half_height, size=count),
        ]
    )
    cap = _unit_sphere(count, rng) * radius
    cap[:, 2] += np.where(cap[:, 2] >= 0.0, half_height, -half_height)
    return np.where(on_side[:, None], side, cap)


def _mesh_surface(
    model: mujoco.MjModel, mesh_id: int, count: int, rng: np.random.Generator
) -> NDArray[np.float64]:
    """Area-weighted points over the mesh's triangles."""
    if mesh_id < 0:
        return np.zeros((0, 3), dtype=np.float64)
    vertices, faces = _mesh_arrays(model, mesh_id)
    if len(faces) == 0:
        return np.zeros((0, 3), dtype=np.float64)
    a, b, c = vertices[faces[:, 0]], vertices[faces[:, 1]], vertices[faces[:, 2]]
    areas = 0.5 * np.linalg.norm(np.cross(b - a, c - a), axis=1)
    total = float(areas.sum())
    if total <= 0.0:
        return vertices[rng.choice(len(vertices), min(count, len(vertices)), replace=False)]
    picked = rng.choice(len(faces), size=count, p=areas / total)
    u = rng.random((count, 1))
    v = rng.random((count, 1))
    flip = (u + v) > 1.0
    u[flip], v[flip] = 1.0 - u[flip], 1.0 - v[flip]
    return a[picked] + u * (b[picked] - a[picked]) + v * (c[picked] - a[picked])


def _mesh_arrays(
    model: mujoco.MjModel, mesh_id: int
) -> tuple[NDArray[np.float64], NDArray[np.int32]]:
    vert_adr = int(model.mesh_vertadr[mesh_id])
    vert_num = int(model.mesh_vertnum[mesh_id])
    face_adr = int(model.mesh_faceadr[mesh_id])
    face_num = int(model.mesh_facenum[mesh_id])
    vertices = np.asarray(
        model.mesh_vert[vert_adr : vert_adr + vert_num], dtype=np.float64
    ).reshape(-1, 3)
    faces = np.asarray(model.mesh_face[face_adr : face_adr + face_num], dtype=np.int32).reshape(
        -1, 3
    )
    return vertices, faces


def _mesh_extent(model: mujoco.MjModel, mesh_id: int) -> NDArray[np.float64]:
    vertices, _ = _mesh_arrays(model, mesh_id)
    if len(vertices) == 0:
        return np.zeros(3, dtype=np.float64)
    return np.ptp(vertices, axis=0)
