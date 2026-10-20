#!/usr/bin/env python3

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

import numpy as np
from numpy.typing import NDArray

from dimos.simulation.mujoco.constants import MAX_HEIGHT, MAX_RANGE, MIN_RANGE


def depth_image_to_point_cloud(
    depth_image: NDArray[Any],
    camera_pos: NDArray[Any],
    camera_mat: NDArray[Any],
    fov_degrees: float = 120,
) -> NDArray[Any]:
    """
    Convert a depth image from a camera to a 3D point cloud using perspective projection.

    Args:
        depth_image: 2D numpy array of depth values in meters
        camera_pos: 3D position of camera in world coordinates
        camera_mat: 3x3 camera rotation matrix in world coordinates
        fov_degrees: Vertical field of view of the camera in degrees
        min_range: Minimum distance from camera to include points (meters)

    Returns:
        numpy array of 3D points in world coordinates, shape (N, 3)
    """
    height, width = depth_image.shape

    fovy = math.radians(fov_degrees)
    f = height / (2 * math.tan(fovy / 2))  # focal length in pixels
    cx = width / 2  # principal point x
    cy = height / 2  # principal point y

    # Pinhole back-projection, the same as open3d's create_from_depth_image on a
    # float depth image: every pixel with d > 0 maps to ((u - cx) d / f, (v - cy) d / f, d).
    depth = depth_image.astype(np.float32)
    v, u = np.nonzero(depth > 0)
    if u.size == 0:
        return np.array([]).reshape(0, 3)
    d = depth[v, u].astype(np.float64)
    camera_points: NDArray[Any] = np.column_stack(((u - cx) * d / f, (v - cy) * d / f, d))

    # Flip y and z axes
    camera_points[:, 1] = -camera_points[:, 1]
    camera_points[:, 2] = -camera_points[:, 2]

    # y (index 1) is up here
    valid_mask = (
        (np.abs(camera_points[:, 0]) <= MAX_RANGE)
        & (np.abs(camera_points[:, 1]) <= MAX_HEIGHT)
        & (np.abs(camera_points[:, 2]) >= MIN_RANGE)
        & (np.abs(camera_points[:, 2]) <= MAX_RANGE)
    )
    camera_points = camera_points[valid_mask]

    if camera_points.size == 0:
        return np.array([]).reshape(0, 3)

    # Transform to world coordinates
    world_points: NDArray[Any] = (camera_mat @ camera_points.T).T + camera_pos

    return world_points


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
