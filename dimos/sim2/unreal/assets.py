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

"""Export the authoritative compiled world for the Unreal rendering process.

MuJoCo owns geometry placement. Its compiler has already scaled, centered and
rotated mesh vertices and compensated the geom pose, so exporting source meshes
or applying ``mesh_pos`` a second time would misplace the robot's links.

The wire uses Unreal's left-handed centimetres and xyzw quaternions. Conversion
happens here exactly once; the rendering process must not apply another basis
change. Materials preserve opaque colour and roughness, not source textures.
Partial transparency is rejected because the native bridge uses opaque materials.
"""

from __future__ import annotations

from collections.abc import Sequence
from typing import Any

import mujoco
import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation

_BASIS = np.diag([1.0, -1.0, 1.0])
_CAMERA_AXES = np.array([[0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [-1.0, 0.0, 0.0]])
_DEFAULT_RGBA = np.array([0.5, 0.5, 0.5, 1.0], dtype=np.float32)
_Geometry = tuple[NDArray[np.float64], NDArray[np.int64], NDArray[np.float64]]


def _pose(position: NDArray[np.float64], rotation: NDArray[np.float64]) -> dict[str, Any]:
    return {
        "position": (100.0 * (_BASIS @ position)).tolist(),
        "quaternion": Rotation.from_matrix(rotation).as_quat().tolist(),
    }


def geom_poses(data: mujoco.MjData, geom_ids: Sequence[int]) -> list[dict[str, Any]]:
    """Copy geom poses from one already-forwarded physics state."""
    return [
        {
            "id": int(index),
            **_pose(data.geom_xpos[index], _BASIS @ data.geom_xmat[index].reshape(3, 3) @ _BASIS),
        }
        for index in geom_ids
    ]


def camera_pose(position: NDArray[np.float64], rotation: NDArray[np.float64]) -> dict[str, Any]:
    """Convert a MuJoCo camera pose (-Z forward, +Y up) to Unreal (+X, +Z)."""
    return _pose(np.asarray(position), _BASIS @ np.asarray(rotation).reshape(3, 3) @ _CAMERA_AXES)


def camera_parameters(
    model: mujoco.MjModel, data: mujoco.MjData, camera_id: int, width: int, height: int
) -> dict[str, Any]:
    """Return a named perspective camera's calibration and current world pose."""
    if width <= 0 or height <= 0:
        raise ValueError("camera image dimensions must be positive")
    if not 0 <= camera_id < model.ncam:
        raise ValueError("camera_id must identify a compiled camera")
    if model.cam_projection[camera_id] != 0:
        raise ValueError("Unreal camera bridge currently requires a perspective camera")
    sensor_size = np.asarray(model.cam_sensorsize[camera_id])
    if np.all(sensor_size > 0):
        intrinsic = np.asarray(model.cam_intrinsic[camera_id])
        fx, fy = intrinsic[:2] / sensor_size * (width, height)
        # Match MuJoCo's asymmetric frustum, expressed in pixel-centre coordinates.
        cx = (width - 1) / 2 - intrinsic[2] / sensor_size[0] * width
        cy = (height - 1) / 2 - intrinsic[3] / sensor_size[1] * height
    else:
        fx = fy = height / (2 * np.tan(np.deg2rad(model.cam_fovy[camera_id]) / 2))
        cx, cy = (width - 1) / 2, (height - 1) / 2
    return {
        **camera_pose(data.cam_xpos[camera_id], data.cam_xmat[camera_id]),
        "width": width,
        "height": height,
        "fx": float(fx),
        "fy": float(fy),
        "cx": float(cx),
        "cy": float(cy),
    }


def _box(size: NDArray[np.float64]) -> _Geometry:
    vertices: list[NDArray[np.float64]] = []
    normals: list[NDArray[np.float64]] = []
    triangles: list[tuple[int, int, int]] = []
    for axis in range(3):
        for sign in (-1, 1):
            normal = np.eye(3)[axis] * sign
            u = np.eye(3)[(axis + 1) % 3]
            v = np.cross(normal, u)
            center = normal * size
            start = len(vertices)
            vertices.extend(
                center + (a * u + b * v) * size for a, b in ((-1, -1), (1, -1), (1, 1), (-1, 1))
            )
            normals.extend([normal] * 4)
            triangles.extend(((start, start + 1, start + 2), (start, start + 2, start + 3)))
    return np.asarray(vertices), np.asarray(triangles), np.asarray(normals)


def _round_surface(size: NDArray[np.float64], kind: int) -> _Geometry:
    segments = 32
    angle = np.arange(segments) * (2 * np.pi / segments)
    if kind == mujoco.mjtGeom.mjGEOM_CAPSULE:
        polar = np.concatenate((np.linspace(0, np.pi / 2, 9), np.linspace(np.pi / 2, np.pi, 9)))
        z = np.cos(polar) * size[0] + np.repeat([size[1], -size[1]], 9)
        radius = np.sin(polar) * size[0]
        normals_z, normals_r = np.cos(polar), np.sin(polar)
    else:
        polar = np.linspace(0, np.pi, 17)
        z, radius = np.cos(polar) * size[0], np.sin(polar) * size[0]
        normals_z, normals_r = np.cos(polar), np.sin(polar)
    vertices = np.stack(
        (
            radius[:, None] * np.cos(angle),
            radius[:, None] * np.sin(angle),
            np.broadcast_to(z[:, None], (len(z), segments)),
        ),
        axis=-1,
    ).reshape(-1, 3)
    normals = np.stack(
        (
            normals_r[:, None] * np.cos(angle),
            normals_r[:, None] * np.sin(angle),
            np.broadcast_to(normals_z[:, None], (len(z), segments)),
        ),
        axis=-1,
    ).reshape(-1, 3)
    faces: list[tuple[int, int, int]] = []
    for ring in range(len(z) - 1):
        for section in range(segments):
            a, b = ring * segments + section, ring * segments + (section + 1) % segments
            c, d = a + segments, b + segments
            if ring:
                faces.append((a, c, b))
            if ring < len(z) - 2:
                faces.append((b, c, d))
    return vertices, np.asarray(faces), normals


def _cylinder(size: NDArray[np.float64]) -> _Geometry:
    count = 32
    theta = np.arange(count) * (2 * np.pi / count)
    radial = np.column_stack((np.cos(theta), np.sin(theta), np.zeros(count)))
    bottom, top = radial * size[0], radial * size[0]
    bottom[:, 2], top[:, 2] = -size[1], size[1]
    vertices = np.concatenate((bottom, top, bottom, top, [[0, 0, -size[1]], [0, 0, size[1]]]))
    normals = np.concatenate(
        (
            radial,
            radial,
            np.tile([0, 0, -1], (count, 1)),
            np.tile([0, 0, 1], (count, 1)),
            [[0, 0, -1], [0, 0, 1]],
        )
    )
    faces: list[tuple[int, int, int]] = []
    for i in range(count):
        j = (i + 1) % count
        faces.extend(
            (
                (i, j, i + count),
                (j, j + count, i + count),
                (4 * count, j + 2 * count, i + 2 * count),
                (4 * count + 1, i + 3 * count, j + 3 * count),
            )
        )
    return vertices, np.asarray(faces), normals


def _mesh(model: mujoco.MjModel, mesh_id: int) -> _Geometry:
    va, fa, na = (
        int(getattr(model, f"mesh_{name}adr")[mesh_id]) for name in ("vert", "face", "normal")
    )
    vn, fn, nn = (
        int(getattr(model, f"mesh_{name}num")[mesh_id]) for name in ("vert", "face", "normal")
    )
    vertices = np.asarray(model.mesh_vert[va : va + vn], dtype=np.float64)
    faces = np.asarray(model.mesh_face[fa : fa + fn], dtype=np.int64)
    normals = np.asarray(model.mesh_normal[na : na + nn], dtype=np.float64)
    corner_normals = np.asarray(model.mesh_facenormal[fa : fa + fn], dtype=np.int64)
    if np.any(corner_normals < 0):
        points = vertices[faces].reshape(-1, 3)
        face_normals = np.cross(points[1::3] - points[::3], points[2::3] - points[::3])
        face_normals /= np.maximum(np.linalg.norm(face_normals, axis=1, keepdims=True), 1e-15)
        return points, np.arange(len(points)).reshape(-1, 3), np.repeat(face_normals, 3, axis=0)
    # OBJ files can use separate vertex and normal indices for hard edges.
    indices, inverse = np.unique(
        np.column_stack((faces.ravel(), corner_normals.ravel())), axis=0, return_inverse=True
    )
    return vertices[indices[:, 0]], inverse.reshape(-1, 3), normals[indices[:, 1]]


def _geometry(model: mujoco.MjModel, geom_id: int) -> _Geometry:
    kind = int(model.geom_type[geom_id])
    size = np.asarray(model.geom_size[geom_id], dtype=np.float64)
    if kind == mujoco.mjtGeom.mjGEOM_MESH:
        return _mesh(model, int(model.geom_dataid[geom_id]))
    if kind == mujoco.mjtGeom.mjGEOM_BOX:
        return _box(size)
    if kind in (mujoco.mjtGeom.mjGEOM_SPHERE, mujoco.mjtGeom.mjGEOM_CAPSULE):
        return _round_surface(size, kind)
    if kind == mujoco.mjtGeom.mjGEOM_CYLINDER:
        return _cylinder(size)
    if kind == mujoco.mjtGeom.mjGEOM_PLANE:
        half = np.where(size[:2] > 0, size[:2], max(float(model.stat.extent), 1.0) * 2)
        vertices = np.array(
            [
                [-half[0], -half[1], 0],
                [half[0], -half[1], 0],
                [half[0], half[1], 0],
                [-half[0], half[1], 0],
            ]
        )
        return vertices, np.array([[0, 1, 2], [0, 2, 3]]), np.tile([0.0, 0.0, 1.0], (4, 1))
    raise ValueError(f"geom {geom_id} has unsupported Unreal visual type {kind}")


def export_scene(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    *,
    geom_groups: tuple[int, ...] = (0, 1, 2, 3, 4, 5),
) -> dict[str, Any]:
    """Build a renderer manifest from the compiled world's visible rigid geoms.

    Mesh assets are shared between instances. Hidden groups and fully transparent
    geoms are omitted. Unsupported geometry fails explicitly instead of leaving
    a silently incomplete scene. Physics is never advanced by this operation.
    """
    meshes: dict[str, dict[str, Any]] = {}
    geoms = []
    for index in range(model.ngeom):
        if int(model.geom_group[index]) not in geom_groups:
            continue
        rgba = np.asarray(model.geom_rgba[index]).copy()
        material = int(model.geom_matid[index])
        if material >= 0 and np.array_equal(rgba, _DEFAULT_RGBA):
            rgba = np.asarray(model.mat_rgba[material]).copy()
        if rgba[3] <= 0:
            continue
        if rgba[3] < 1:
            raise ValueError(
                f"geom {index} is partially transparent; Unreal bridge requires opaque materials"
            )
        if int(model.geom_type[index]) == mujoco.mjtGeom.mjGEOM_MESH:
            mesh_id = f"mesh_{int(model.geom_dataid[index])}"
        else:
            mesh_id = f"geom_{index}"
        if mesh_id not in meshes:
            vertices, faces, normals = _geometry(model, index)
            meshes[mesh_id] = {
                "id": mesh_id,
                "vertices": (100.0 * (vertices @ _BASIS)).tolist(),
                # Reflecting Y changes MuJoCo's CCW faces into Unreal's CW faces.
                # Match UKismetProceduralMeshLibrary::GenerateBoxMesh: the usual
                # right-handed edge cross product points opposite the normal.
                "triangles": faces.ravel().tolist(),
                "normals": (normals @ _BASIS).tolist(),
            }
        roughness = float(model.mat_roughness[material]) if material >= 0 else -1.0
        geoms.append(
            {
                **geom_poses(data, [index])[0],
                "mesh_id": mesh_id,
                "rgba": rgba.tolist(),
                "roughness": roughness if roughness >= 0 else 0.6,
            }
        )
    return {"version": 1, "meshes": list(meshes.values()), "geoms": geoms}
