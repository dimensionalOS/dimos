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

"""Import MJCF visual assets once; no physics data or integration is required."""

from __future__ import annotations

from collections.abc import Callable
from typing import Annotated, cast

import mujoco
import numpy as np
from numpy.typing import NDArray
from pydantic import BaseModel, ConfigDict, Field, ValidationError
from scipy.spatial.transform import Rotation

from dimos.sim2.physics import Quat, Vec3
from dimos.sim2.visuals import (
    NativeVisualAsset,
    VisualAttachment,
    VisualCamera,
    VisualGeometry,
    VisualMesh,
    VisualScene,
)

_DEFAULT_RGBA = np.array([0.5, 0.5, 0.5, 1.0], dtype=np.float32)
_Geometry = tuple[NDArray[np.float64], NDArray[np.int64], NDArray[np.float64]]
_Scale = Annotated[float, Field(gt=0, allow_inf_nan=False)]


class _NativeAsset(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True)

    asset: str = Field(pattern=r"^/(Game|Engine)/[^\\\s]+\.[^/\\.\s]+$")
    scale: tuple[_Scale, _Scale, _Scale]


class _NativeAssets(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True)

    version: int = Field(ge=1, le=1)
    geoms: dict[str, _NativeAsset]
    hidden_geoms: list[str] = Field(default_factory=list)


def _native_assets(model: mujoco.MjModel) -> tuple[dict[int, _NativeAsset], set[int]]:
    """Read renderer bindings embedded in the authoritative compiled MJCF."""
    text_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TEXT, "dimos_unreal_assets")
    if text_id < 0:
        return {}, set()
    start = int(model.text_adr[text_id])
    size = int(model.text_size[text_id])
    try:
        metadata = _NativeAssets.model_validate_json(model.text_data[start : start + size - 1])
    except ValidationError as error:
        raise ValueError(f"invalid dimos_unreal_assets: {error}") from error
    if len(metadata.hidden_geoms) != len(set(metadata.hidden_geoms)):
        raise ValueError("dimos_unreal_assets hidden_geoms must not contain duplicate names")
    overlap = set(metadata.geoms).intersection(metadata.hidden_geoms)
    if overlap:
        raise ValueError(f"dimos_unreal_assets cannot hide native asset markers: {sorted(overlap)}")

    def geom_id(name: str) -> int:
        index = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
        if index < 0:
            raise ValueError(f"dimos_unreal_assets references unknown geom {name!r}")
        return int(index)

    return (
        {geom_id(name): asset for name, asset in metadata.geoms.items()},
        {geom_id(name) for name in metadata.hidden_geoms},
    )


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


def load_visuals(
    model: mujoco.MjModel,
    body_names: dict[int, str],
    public_name: Callable[[str], str],
    revision: int,
    *,
    geom_groups: tuple[int, ...] = (0, 1, 2),
) -> VisualScene:
    """Import authored visuals using MJCF's default visible groups, excluding debug groups."""
    native_assets, hidden_geoms = _native_assets(model)
    meshes: dict[str, VisualMesh] = {}
    geometries = []
    for index in range(model.ngeom):
        if index in hidden_geoms or int(model.geom_group[index]) not in geom_groups:
            continue
        attachment = VisualAttachment(
            body_names[int(model.geom_bodyid[index])],
            cast("Vec3", tuple(model.geom_pos[index])),
            cast("Quat", tuple(np.roll(model.geom_quat[index], -1))),
        )
        if index in native_assets:
            asset = native_assets[index]
            geometries.append(
                VisualGeometry(
                    index,
                    attachment,
                    native_asset=NativeVisualAsset("unreal", asset.asset, asset.scale),
                )
            )
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
        mesh_id = (
            f"mesh_{int(model.geom_dataid[index])}"
            if int(model.geom_type[index]) == mujoco.mjtGeom.mjGEOM_MESH
            else f"geom_{index}"
        )
        if mesh_id not in meshes:
            meshes[mesh_id] = VisualMesh(*_geometry(model, index))
        roughness = float(model.mat_roughness[material]) if material >= 0 else -1.0
        geometries.append(
            VisualGeometry(
                index,
                attachment,
                mesh_id,
                cast("tuple[float, float, float, float]", tuple(rgba.tolist())),
                roughness if roughness >= 0 else 0.6,
            )
        )
    frames = {
        public_name(model.site(index).name): VisualAttachment(
            body_names[int(model.site_bodyid[index])],
            cast("Vec3", tuple(model.site_pos[index])),
            cast("Quat", tuple(np.roll(model.site_quat[index], -1))),
        )
        for index in range(model.nsite)
        if model.site(index).name
    }
    cameras = {}
    for index in range(model.ncam):
        name = model.camera(index).name or f"camera_{index}"
        sensor_size = np.asarray(model.cam_sensorsize[index])
        focal_ratio = None
        principal = (0.0, 0.0)
        if np.all(sensor_size > 0):
            intrinsic = np.asarray(model.cam_intrinsic[index])
            focal_ratio = cast("tuple[float, float]", tuple(intrinsic[:2] / sensor_size))
            principal = cast("tuple[float, float]", tuple(-intrinsic[2:] / sensor_size))
        optical = Rotation.from_quat(np.roll(model.cam_quat[index], -1)).as_matrix() @ np.diag(
            [1, -1, -1]
        )
        cameras[public_name(name)] = VisualCamera(
            VisualAttachment(
                body_names[int(model.cam_bodyid[index])],
                cast("Vec3", tuple(model.cam_pos[index])),
                cast("Quat", tuple(Rotation.from_matrix(optical).as_quat())),
            ),
            float(model.cam_fovy[index]),
            focal_ratio,
            principal,
            bool(model.cam_projection[index] == 0),
            bool(model.cam_mode[index] == mujoco.mjtCamLight.mjCAMLIGHT_FIXED),
        )
    return VisualScene(revision, meshes, tuple(geometries), frames, cameras)
