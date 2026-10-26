# Copyright 2025-2026 Dimensional Inc.
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

import math
from typing import Any

import mujoco
import numpy as np
from numpy.typing import NDArray
import open3d as o3d  # type: ignore[import-untyped]

from dimos.simulation.mujoco.constants import MAX_HEIGHT, MIN_RANGE


def scan_directions(
    n_azimuth: int, elevation_min_deg: float, elevation_max_deg: float, n_elevation: int
) -> NDArray[np.float64]:
    """Unit vectors (N, 3) in the site frame, z up: a regular azimuth x elevation grid."""
    azimuth, elevation = np.meshgrid(
        np.linspace(0.0, 2 * np.pi, n_azimuth, endpoint=False),
        np.deg2rad(np.linspace(elevation_min_deg, elevation_max_deg, n_elevation)),
    )
    return np.column_stack(
        (
            (np.cos(elevation) * np.cos(azimuth)).ravel(),
            (np.cos(elevation) * np.sin(azimuth)).ravel(),
            np.sin(elevation).ravel(),
        )
    )


# Limit each angular step to a 0.25 mm chord error, leaving room for the
# diagonal edges of sphere triangles and oblique ray intersections.
_CHORD_ERROR = 0.00025
_INFINITE_PLANE_HALF_SIZE = 100.0


def _circle_segments(radius: float) -> int:
    return max(8, math.ceil(math.pi / math.acos(1 - min(_CHORD_ERROR / radius, 1.0))))


def _capsule_triangles(
    radius: float, half_length: float, segments: int
) -> tuple[NDArray[np.float64], NDArray[np.uint32]]:
    # Separate equators join the hemispheres with a cylindrical band. Stretching
    # a sphere's single equator would taper that band instead.
    quarter = math.ceil(segments / 4)
    top = np.linspace(0, np.pi / 2, quarter + 1)[1:]
    bottom = np.linspace(np.pi / 2, np.pi, quarter + 1)[:-1]
    latitude = np.concatenate((top, bottom))
    longitude = np.linspace(0, 2 * np.pi, segments, endpoint=False)
    r = radius * np.sin(latitude)
    z = radius * np.cos(latitude) + np.repeat([half_length, -half_length], quarter)
    rings = np.stack(
        (
            r[:, None] * np.cos(longitude),
            r[:, None] * np.sin(longitude),
            np.broadcast_to(z[:, None], (len(latitude), segments)),
        ),
        axis=-1,
    ).reshape(-1, 3)
    vertices = np.vstack(([0, 0, radius + half_length], [0, 0, -radius - half_length], rings))
    indices = np.arange(2, len(vertices), dtype=np.uint32).reshape(-1, segments)
    following = np.roll(indices, -1, axis=1)
    faces = np.vstack(
        (
            np.column_stack((np.zeros(segments, dtype=np.uint32), indices[0], following[0])),
            np.column_stack((np.ones(segments, dtype=np.uint32), following[-1], indices[-1])),
            np.column_stack((indices[:-1].ravel(), indices[1:].ravel(), following[:-1].ravel())),
            np.column_stack((following[:-1].ravel(), indices[1:].ravel(), following[1:].ravel())),
        )
    )
    return vertices, faces


def _geom_triangles(
    model: mujoco.MjModel, geom_id: int
) -> tuple[NDArray[np.float64], NDArray[np.uint32]]:
    """Triangles in the compiled geom's local frame, including MuJoCo mesh offsets."""
    geom_type = model.geom_type[geom_id]
    size = model.geom_size[geom_id]
    if geom_type == mujoco.mjtGeom.mjGEOM_MESH:
        mesh_id = model.geom_dataid[geom_id]
        va, vn = model.mesh_vertadr[mesh_id], model.mesh_vertnum[mesh_id]
        fa, fn = model.mesh_faceadr[mesh_id], model.mesh_facenum[mesh_id]
        return (
            model.mesh_vert[va : va + vn].astype(np.float64),
            model.mesh_face[fa : fa + fn].astype(np.uint32),
        )
    if geom_type == mujoco.mjtGeom.mjGEOM_PLANE:
        x, y = np.where(size[:2] > 0, size[:2], _INFINITE_PLANE_HALF_SIZE)
        return (
            np.array([[-x, -y, 0], [x, -y, 0], [x, y, 0], [-x, y, 0]]),
            np.array([[0, 1, 2], [0, 2, 3]], dtype=np.uint32),
        )
    if geom_type == mujoco.mjtGeom.mjGEOM_BOX:
        mesh = o3d.geometry.TriangleMesh.create_box(*(2 * size))
        mesh.translate(-size)
    elif geom_type == mujoco.mjtGeom.mjGEOM_CYLINDER:
        mesh = o3d.geometry.TriangleMesh.create_cylinder(
            size[0], 2 * size[1], resolution=_circle_segments(size[0]), split=1
        )
    elif geom_type == mujoco.mjtGeom.mjGEOM_CAPSULE:
        return _capsule_triangles(size[0], size[1], _circle_segments(size[0]))
    elif geom_type in (mujoco.mjtGeom.mjGEOM_SPHERE, mujoco.mjtGeom.mjGEOM_ELLIPSOID):
        radii = np.full(3, size[0]) if geom_type == mujoco.mjtGeom.mjGEOM_SPHERE else size
        mesh = o3d.geometry.TriangleMesh.create_sphere(
            1.0, resolution=math.ceil(_circle_segments(float(radii.max())) / 2)
        )
        return np.asarray(mesh.vertices) * radii, np.asarray(mesh.triangles, dtype=np.uint32)
    else:
        raise ValueError(
            f"Unsupported lidar geom type {mujoco.mjtGeom(geom_type).name}: geom {geom_id}"
        )
    return np.asarray(mesh.vertices), np.asarray(mesh.triangles, dtype=np.uint32)


def _make_scene(
    vertices: NDArray[np.float64], faces: NDArray[np.uint32]
) -> o3d.t.geometry.RaycastingScene:
    scene = o3d.t.geometry.RaycastingScene()
    if len(faces):
        scene.add_triangles(o3d.core.Tensor(vertices.astype(np.float32)), o3d.core.Tensor(faces))
    # Embree commits its BVH on the first query. Pay that cost at load, not in
    # the first simulation iteration.
    scene.cast_rays(o3d.core.Tensor([[0, 0, 0, 1, 0, 0]], dtype=o3d.core.Dtype.Float32))
    return scene


class EmbreeLidar:
    """Cached visible geometry for synchronous, world-frame lidar scans.

    Construct after mj_forward. Static geometry is fixed for this object's
    lifetime; dynamic poses are read from data on every cast. The caller must
    keep model and data unchanged during a cast and serialize calls.
    """

    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        site_id: int,
        directions_local: NDArray[np.float64],
    ) -> None:
        self._model = model
        self._site_id = site_id
        # Contiguous columns let einsum vectorize across rays. Row-major input
        # makes each transform about six times slower on the full scan grid.
        self._directions_local = np.asfortranarray(directions_local)
        self._directions_world = np.empty_like(self._directions_local)
        self._directions_geom = np.empty_like(self._directions_local)
        self._robot_root = model.body_rootid[model.site_bodyid[site_id]]
        self._rays = np.empty((len(directions_local), 6), dtype=np.float32)
        self._ray_tensor = o3d.core.Tensor.from_numpy(self._rays)
        self._dynamic: list[tuple[int, o3d.t.geometry.RaycastingScene]] = []

        # A welded descendant of a mocap body still moves with its ancestor.
        mocap_ancestor = model.body_mocapid >= 0
        for body in range(1, model.nbody):
            mocap_ancestor[body] |= mocap_ancestor[model.body_parentid[body]]

        vertices, faces = [], []
        offset = 0
        for geom in range(model.ngeom):
            body = model.geom_bodyid[geom]
            material = model.geom_matid[geom]
            if (
                model.body_rootid[body] == self._robot_root
                or model.geom_group[geom] not in (0, 1, 2)
                or model.geom_rgba[geom, 3] <= 0
                or (material >= 0 and model.mat_rgba[material, 3] <= 0)
            ):
                continue
            v, f = _geom_triangles(model, geom)
            if model.body_weldid[body] == 0 and not mocap_ancestor[body]:
                rotation = data.geom_xmat[geom].reshape(3, 3)
                vertices.append(np.einsum("ij,kj->ik", v, rotation) + data.geom_xpos[geom])
                faces.append(f + offset)
                offset += len(v)
            else:
                self._dynamic.append((geom, _make_scene(v, f)))
        self._static = _make_scene(
            np.vstack(vertices) if vertices else np.empty((0, 3)),
            np.vstack(faces) if faces else np.empty((0, 3), dtype=np.uint32),
        )

    def _cast_scene(
        self,
        scene: o3d.t.geometry.RaycastingScene,
        origin: NDArray[np.float64],
        directions: NDArray[np.float64],
    ) -> NDArray[np.float32]:
        # Column assignments avoid NumPy's slower broadcast over a strided
        # (N, 3) slice of the interleaved origin/direction buffer.
        for axis in range(3):
            self._rays[:, axis] = origin[axis]
            self._rays[:, axis + 3] = directions[:, axis]
        distances: NDArray[np.float32] = scene.cast_rays(self._ray_tensor)["t_hit"].numpy()
        return distances

    def cast(self, data: mujoco.MjData, max_range: float) -> NDArray[np.float32]:
        """Hit points within the range and world height bands, without robot returns."""
        if len(self._directions_local) == 0:
            return np.empty((0, 3), dtype=np.float32)
        origin: NDArray[np.float64] = data.site_xpos[self._site_id]
        rotation: NDArray[np.float64] = data.site_xmat[self._site_id].reshape(3, 3)
        # Matrix multiplication on the full grid makes OpenBLAS workers spin
        # long after the product finishes. einsum avoids that CPU cost.
        directions_world = self._directions_world
        np.einsum("ij,kj->ik", self._directions_local, rotation, out=directions_world)
        distances = self._cast_scene(self._static, origin, directions_world)
        for geom, scene in self._dynamic:
            position = data.geom_xpos[geom]
            if np.linalg.norm(position - origin) - self._model.geom_rbound[geom] > max_range:
                continue
            rotation = data.geom_xmat[geom].reshape(3, 3)
            local_origin = rotation.T @ (origin - position)
            np.einsum("ij,jk->ik", directions_world, rotation, out=self._directions_geom)
            np.minimum(
                distances,
                self._cast_scene(scene, local_origin, self._directions_geom),
                out=distances,
            )

        # Float32 intersections can land one ULP across an inclusive boundary,
        # even for an axis-aligned ray hitting a flat face.
        near = np.nextafter(np.float32(MIN_RANGE), np.float32(-np.inf))
        far = np.nextafter(np.float32(max_range), np.float32(np.inf))
        valid = np.isfinite(distances) & (distances >= near) & (distances <= far)
        # Misses can have horizontal directions (0 * inf); only multiply hits.
        valid[valid] &= np.abs(directions_world[valid, 2] * distances[valid]).astype(
            np.float32
        ) <= np.float32(MAX_HEIGHT)
        hit_distances = distances[valid]
        np.clip(hit_distances, MIN_RANGE, max_range, out=hit_distances)
        # Keep columns contiguous through filtering and voxel downsampling.
        # This also avoids several full (N, 3) arithmetic temporaries.
        points = np.empty((len(hit_distances), 3), dtype=np.float32, order="F")
        for axis in range(3):
            points[:, axis] = origin[axis] + directions_world[valid, axis] * hit_distances
        return points

    def cast_box(self, data: mujoco.MjData, half_width: float) -> NDArray[np.float32]:
        """Hits in a square following the base's XY translation, without rotating."""
        center = data.xpos[self._robot_root, :2]
        origin = data.site_xpos[self._site_id, :2]
        farthest_xy = np.abs(origin - center) + half_width
        max_range = float(np.sqrt(np.dot(farthest_xy, farthest_xy) + MAX_HEIGHT**2))
        points = self.cast(data, max_range)

        # Compare bounds at the same precision as the returned cloud.
        lower = (center - half_width).astype(np.float32)
        upper = (center + half_width).astype(np.float32)
        # Float32 distances introduce a few ULPs of position error. Snap hits
        # at the boundary so corners stay inclusive and returned points obey
        # the float32 bounds.
        roundoff = 2 * np.spacing(np.float32(max_range))
        inside = (
            (points[:, 0] >= lower[0] - roundoff)
            & (points[:, 0] <= upper[0] + roundoff)
            & (points[:, 1] >= lower[1] - roundoff)
            & (points[:, 1] <= upper[1] + roundoff)
        )
        bounded = np.empty((np.count_nonzero(inside), 3), dtype=np.float32, order="F")
        for axis in range(2):
            np.clip(points[inside, axis], lower[axis], upper[axis], out=bounded[:, axis])
        bounded[:, 2] = points[inside, 2]
        return bounded


def voxel_down_sample(points: NDArray[Any], voxel_size: float) -> NDArray[Any]:
    """Centroid of the points in each voxel.

    Uses open3d's voxel grid (origin at ``min_bound - voxel_size / 2``), so the
    result matches ``PointCloud.voxel_down_sample`` without importing open3d.
    """
    if len(points) == 0:
        return np.array([]).reshape(0, 3)
    min_bound = points.min(axis=0) - voxel_size * 0.5
    index = np.floor((points - min_bound) / voxel_size).astype(np.int64)
    dims = index.max(axis=0) + 1
    key = (index[:, 0] * dims[1] + index[:, 1]) * dims[2] + index[:, 2]
    _, inverse = np.unique(key, return_inverse=True)
    counts = np.bincount(inverse)
    centroids = np.empty((counts.size, 3), dtype=np.float64)
    for axis in range(3):
        centroids[:, axis] = np.bincount(inverse, weights=points[:, axis]) / counts
    return centroids
