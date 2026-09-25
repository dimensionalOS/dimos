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

"""The numpy depth back-projection and voxel filter must match the open3d calls they replaced."""

import math

import numpy as np
import open3d as o3d  # type: ignore[import-untyped]

from dimos.simulation.mujoco.constants import LIDAR_RESOLUTION, MAX_HEIGHT, MAX_RANGE, MIN_RANGE
from dimos.simulation.mujoco.depth_camera import depth_image_to_point_cloud, voxel_down_sample


def _reference_point_cloud(
    depth: np.ndarray, camera_pos: np.ndarray, camera_mat: np.ndarray, fov_degrees: float
) -> np.ndarray:
    """The open3d path depth_image_to_point_cloud used to run."""
    height, width = depth.shape
    f = height / (2 * math.tan(math.radians(fov_degrees) / 2))
    intrinsics = o3d.camera.PinholeCameraIntrinsic(width, height, f, f, width / 2, height / 2)
    cloud = o3d.geometry.PointCloud.create_from_depth_image(
        o3d.geometry.Image(depth.astype(np.float32)), intrinsics, depth_scale=1.0
    )
    points = np.asarray(cloud.points)
    if points.size == 0:
        return np.array([]).reshape(0, 3)
    points[:, 1] = -points[:, 1]
    points[:, 2] = -points[:, 2]
    mask = (
        (np.abs(points[:, 0]) <= MAX_RANGE)
        & (np.abs(points[:, 1]) <= MAX_HEIGHT)
        & (np.abs(points[:, 2]) >= MIN_RANGE)
        & (np.abs(points[:, 2]) <= MAX_RANGE)
    )
    points = points[mask]
    if points.size == 0:
        return np.array([]).reshape(0, 3)
    return (camera_mat @ points.T).T + camera_pos


def _sorted(points: np.ndarray) -> np.ndarray:
    return points[np.lexsort((points[:, 2], points[:, 1], points[:, 0]))]


def test_back_projection_matches_open3d() -> None:
    rng = np.random.default_rng(0)
    depth = rng.uniform(0.05, 4.0, size=(360, 640)).astype(np.float32)
    depth[::7, ::5] = 0.0  # dropped by the projection
    depth[3, 3] = np.nan
    depth[10, 10] = -1.0
    depth[20, 20] = 2000.0  # kept by the projection, dropped by the range mask
    camera_pos = np.array([1.0, -2.0, 0.5])
    camera_mat = o3d.geometry.get_rotation_matrix_from_xyz((0.1, 0.2, 0.3))

    expected = _reference_point_cloud(depth, camera_pos, camera_mat, 160)
    actual = depth_image_to_point_cloud(depth, camera_pos, camera_mat, fov_degrees=160)

    assert len(expected) > 0
    assert actual.shape == expected.shape
    np.testing.assert_allclose(actual, expected, atol=1e-9)


def test_all_invalid_depth_gives_no_points() -> None:
    depth = np.zeros((4, 6), dtype=np.float32)
    assert depth_image_to_point_cloud(depth, np.zeros(3), np.eye(3)).shape == (0, 3)


def test_voxel_down_sample_matches_open3d() -> None:
    rng = np.random.default_rng(1)
    points = np.vstack([rng.uniform(-3, 3, size=(20_000, 3)), rng.normal(0, 0.02, size=(5_000, 3))])
    cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
    expected = np.asarray(cloud.voxel_down_sample(voxel_size=LIDAR_RESOLUTION).points)

    actual = voxel_down_sample(points, LIDAR_RESOLUTION)

    assert actual.shape == expected.shape
    np.testing.assert_allclose(_sorted(actual), _sorted(expected), atol=1e-9)
    assert voxel_down_sample(np.empty((0, 3)), LIDAR_RESOLUTION).shape == (0, 3)
