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

"""Sensor rays cast through Embree BVHs (static triangles as one scene, each moving body in its own frame), MuJoCo for the rest: mj_multiRay over office 555's 4150 geoms costs ~60 ms a scan."""

from collections.abc import Collection
from typing import Any

import mujoco
import numpy as np
from numpy.typing import NDArray

_BOX_CORNERS = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)], float)
_BOX_FACES = np.array(
    [
        [0, 2, 3], [0, 3, 1], [4, 5, 7], [4, 7, 6], [0, 1, 5], [0, 5, 4],
        [2, 6, 7], [2, 7, 3], [0, 4, 6], [0, 6, 2], [1, 3, 7], [1, 7, 5],
    ]
)  # fmt: skip
_PLANE = int(mujoco.mjtGeom.mjGEOM_PLANE)
_TRIANGULATED = (int(mujoco.mjtGeom.mjGEOM_BOX), int(mujoco.mjtGeom.mjGEOM_MESH))
_CASTABLE = frozenset(
    {
        int(mujoco.mjtGeom.mjGEOM_PLANE),
        int(mujoco.mjtGeom.mjGEOM_SPHERE),
        int(mujoco.mjtGeom.mjGEOM_CAPSULE),
        int(mujoco.mjtGeom.mjGEOM_ELLIPSOID),
        int(mujoco.mjtGeom.mjGEOM_CYLINDER),
        *_TRIANGULATED,
    }
)


class RayScene:
    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        groups: Collection[int],
        blind_to: Collection[int] = (),
    ) -> None:
        """A cast of what `model`'s geoms in `groups` show, posed from `data`, without the bodies in `blind_to`."""
        self._data = data
        seen = [g for g in range(model.ngeom) if _seen(model, g, groups, blind_to)]
        unsupported = {int(model.geom_type[g]) for g in seen} - _CASTABLE
        if unsupported:
            names = sorted(mujoco.mjtGeom(t).name for t in unsupported)
            raise ValueError(f"a ray scene cannot cast against {names} geoms")
        moves = {g: bool(model.body_weldid[model.geom_bodyid[g]] != 0) for g in seen}
        kind = {g: int(model.geom_type[g]) for g in seen}
        static = [g for g in seen if kind[g] in _TRIANGULATED and not moves[g]]
        self._static = _scene(*_triangles(model, data, static, np.eye(4))) if static else None
        by_body: dict[int, list[int]] = {}
        for g in seen:
            if kind[g] in _TRIANGULATED and moves[g]:
                by_body.setdefault(int(model.geom_bodyid[g]), []).append(g)
        self._bodies = np.array(sorted(by_body), dtype=np.int64)
        self._body_scenes, radii = [], []
        for body in self._bodies:
            vertices, faces = _triangles(model, data, by_body[body], _pose(data, body))
            self._body_scenes.append(_scene(vertices, faces))
            radii.append(np.linalg.norm(vertices, axis=1).max())
        self._planes = np.array([g for g in seen if kind[g] == _PLANE], dtype=np.int64)
        self._plane_sizes = {int(g): model.geom_size[g].copy() for g in self._planes}
        rounded = [g for g in seen if kind[g] not in _TRIANGULATED and kind[g] != _PLANE]
        self._rounded = _RoundedCast(model, data, rounded, moves)
        self._radii = np.concatenate([radii, model.geom_rbound[rounded]])

    def cast(
        self, origin: NDArray[np.float64], directions: NDArray[np.float64], max_range: float
    ) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        data = self._data
        directions = np.ascontiguousarray(directions, dtype=np.float64)
        origin = np.asarray(origin, dtype=np.float64)
        dist = np.full(len(directions), np.inf)
        normals = np.zeros((len(directions), 3))
        for g in self._planes:
            _nearer(
                dist,
                normals,
                slice(None),
                *_plane(data, g, self._plane_sizes[g], origin, directions),
            )
        if self._static is not None:
            _nearer(dist, normals, slice(None), *_embree(self._static, origin, directions))
        centers = np.concatenate([data.xpos[self._bodies], self._rounded.centers()]) - origin
        near = _near(centers, self._radii, directions)
        for k in np.flatnonzero(near[: len(self._bodies)].any(axis=1)):
            rays = np.flatnonzero(near[k])
            rotation = data.xmat[self._bodies[k]].reshape(3, 3)
            local_dist, local_normals = _embree(
                self._body_scenes[k], -centers[k] @ rotation, directions[rays] @ rotation
            )
            _nearer(dist, normals, rays, local_dist, local_normals @ rotation.T)
        rays = np.flatnonzero(near[len(self._bodies) :].any(axis=0))
        if len(rays):
            _nearer(dist, normals, rays, *self._rounded.cast(origin, directions[rays], max_range))
        miss = ~(dist <= max_range)
        dist[miss] = -1.0
        normals[miss] = 0.0
        return dist, normals


class _RoundedCast:
    """MuJoCo's cast against spheres, capsules, ellipsoids and cylinders, the moving ones on mocap bodies."""

    def __init__(
        self, model: mujoco.MjModel, data: mujoco.MjData, geoms: list[int], moves: dict[int, bool]
    ) -> None:
        self._data = data
        self._geoms = np.array(geoms, dtype=np.int64)
        self._moving = np.array([g for g in geoms if moves[g]], dtype=np.int64)
        spec = mujoco.MjSpec()
        for g in geoms:
            if not moves[g]:
                _add_geom(spec.worldbody, model, g, data.geom_xpos[g], _quat(data.geom_xmat[g]))
        for g in self._moving:
            _add_geom(spec.worldbody.add_body(mocap=True), model, int(g))
        self._model = spec.compile()
        self._cast_data = mujoco.MjData(self._model)

    def centers(self) -> NDArray[np.float64]:
        return np.asarray(self._data.geom_xpos[self._geoms], dtype=np.float64)

    def cast(
        self, origin: NDArray[np.float64], directions: NDArray[np.float64], max_range: float
    ) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        model, data, n = self._model, self._cast_data, len(directions)
        data.mocap_pos[:] = self._data.geom_xpos[self._moving]
        for i, matrix in enumerate(self._data.geom_xmat[self._moving]):
            mujoco.mju_mat2Quat(data.mocap_quat[i], matrix)
        mujoco.mj_kinematics(model, data)
        dist = np.full(n, -1.0)
        geom = np.full(n, -1, dtype=np.int32)
        normals = np.zeros(3 * n)
        mujoco.mj_multiRay(
            model, data, origin, directions.ravel(), None, 1, -1, geom, dist, normals, n, max_range
        )
        dist[geom < 0] = np.inf
        return dist, normals.reshape(n, 3)


def _plane(
    data: mujoco.MjData,
    g: int,
    size: NDArray[np.float64],
    origin: NDArray[np.float64],
    directions: NDArray[np.float64],
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """mj_ray's plane: hit from its front only, within its half sizes where they are positive."""
    rotation = data.geom_xmat[g].reshape(3, 3)
    point = (origin - data.geom_xpos[g]) @ rotation
    toward = directions @ rotation
    front = toward[:, 2] < -mujoco.mjMINVAL
    dist = np.full(len(directions), np.inf)
    dist[front] = -point[2] / toward[front, 2]
    dist[dist < 0] = np.inf
    for axis in (0, 1):
        if size[axis] > 0:
            with np.errstate(invalid="ignore"):
                dist[np.abs(point[axis] + dist * toward[:, axis]) > size[axis]] = np.inf
    return dist, np.broadcast_to(rotation[:, 2], directions.shape)


def _near(
    centers: NDArray[np.float64], radii: NDArray[np.float64], directions: NDArray[np.float64]
) -> NDArray[np.bool_]:
    """spheres x rays: whether each ray passes through each sphere (centers relative to the origin)."""
    along = centers.astype(np.float32) @ directions.T.astype(np.float32)
    outside = (np.sum(centers**2, axis=1) - radii**2).astype(np.float32)[:, None]
    near: NDArray[np.bool_] = (along * along > outside) & (
        along > -radii.astype(np.float32)[:, None]
    )
    return near


def _seen(
    model: mujoco.MjModel, g: int, groups: Collection[int], blind_to: Collection[int]
) -> bool:
    """Whether a cast over `groups` hits geom g: mj_ray skips a fully transparent geom too."""
    material = model.geom_matid[g]
    alpha = model.mat_rgba[material, 3] if material >= 0 else model.geom_rgba[g, 3]
    return (
        int(model.geom_group[g]) in groups
        and int(model.geom_bodyid[g]) not in blind_to
        and alpha > 0
    )


def _scene(vertices: NDArray[np.float64], faces: NDArray[np.int64]) -> Any:
    import open3d as o3d  # type: ignore[import-untyped]

    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(
        o3d.core.Tensor(vertices.astype(np.float32)), o3d.core.Tensor(faces.astype(np.uint32))
    )
    return scene


def _embree(
    scene: Any,
    origin: NDArray[np.float64],
    directions: NDArray[np.float64],
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """Distance (inf on a miss) and the hit face's normal, turned to face the ray as MuJoCo's is."""
    import open3d as o3d  # type: ignore[import-untyped]

    rays = np.empty((len(directions), 6), dtype=np.float32)
    rays[:, :3], rays[:, 3:] = origin, directions
    result = scene.cast_rays(o3d.core.Tensor(rays))
    dist = result["t_hit"].numpy().astype(np.float64)
    normals = result["primitive_normals"].numpy().astype(np.float64)
    normals *= -np.sign(np.sum(normals * directions, axis=1, keepdims=True))
    return dist, normals


def _nearer(
    dist: NDArray[np.float64],
    normals: NDArray[np.float64],
    rays: slice | NDArray[np.int64],
    other_dist: NDArray[np.float64],
    other_normals: NDArray[np.float64],
) -> None:
    """Keep, ray by ray, whichever of the two hits is nearer."""
    closer = other_dist < dist[rays]
    index = np.flatnonzero(closer) if isinstance(rays, slice) else rays[closer]
    dist[index] = other_dist[closer]
    normals[index] = other_normals[closer]


def _pose(data: mujoco.MjData, body: int) -> NDArray[np.float64]:
    pose = np.eye(4)
    pose[:3, :3], pose[:3, 3] = data.xmat[body].reshape(3, 3), data.xpos[body]
    return pose


def _quat(matrix: NDArray[np.float64]) -> NDArray[np.float64]:
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, matrix)
    return quat


def _mesh(model: mujoco.MjModel, g: int) -> tuple[NDArray[np.float64], NDArray[np.int64]]:
    """A geom's triangles in its own frame: a box's from its half sizes, a mesh's as compiled."""
    if model.geom_type[g] == int(mujoco.mjtGeom.mjGEOM_BOX):
        return _BOX_CORNERS * model.geom_size[g], _BOX_FACES
    mesh = model.geom_dataid[g]
    start, count = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
    face_start, face_count = model.mesh_faceadr[mesh], model.mesh_facenum[mesh]
    return (
        model.mesh_vert[start : start + count].astype(np.float64),
        model.mesh_face[face_start : face_start + face_count].astype(np.int64),
    )


def _triangles(
    model: mujoco.MjModel, data: mujoco.MjData, geoms: list[int], frame: NDArray[np.float64]
) -> tuple[NDArray[np.float64], NDArray[np.int64]]:
    """The geoms' triangles, as posed in `data`, in the frame whose world pose is `frame`."""
    vertices, faces, offset = [], [], 0
    rotation, position = frame[:3, :3], frame[:3, 3]
    for g in geoms:
        local, triangles = _mesh(model, g)
        world = local @ data.geom_xmat[g].reshape(3, 3).T + data.geom_xpos[g]
        vertices.append((world - position) @ rotation)
        faces.append(triangles + offset)
        offset += len(local)
    return np.concatenate(vertices), np.concatenate(faces)


def _add_geom(
    body: Any,
    model: mujoco.MjModel,
    g: int,
    pos: NDArray[np.float64] | None = None,
    quat: NDArray[np.float64] | None = None,
) -> None:
    geom = body.add_geom(contype=0, conaffinity=0, group=0)
    geom.type, geom.size = mujoco.mjtGeom(model.geom_type[g]), model.geom_size[g]
    if pos is not None and quat is not None:
        geom.pos, geom.quat = pos, quat
