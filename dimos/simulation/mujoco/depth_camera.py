#!/usr/bin/env python3

# Copyright 2025 Dimensional Inc.
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
import numpy as np
import open3d as o3d

RANGE_FINDER_MAX_RANGE = 4


def depth_image_to_point_cloud(
    depth_image: np.ndarray,
    camera_pos: np.ndarray,
    camera_mat: np.ndarray,
    fov_degrees: float = 120,
    min_range: float = 0.0,
) -> np.ndarray:
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

    # Calculate camera intrinsics similar to StackOverflow approach
    fovy = math.radians(fov_degrees)
    f = height / (2 * math.tan(fovy / 2))  # focal length in pixels
    cx = width / 2  # principal point x
    cy = height / 2  # principal point y

    # Create Open3D camera intrinsics
    cam_intrinsics = o3d.camera.PinholeCameraIntrinsic(width, height, f, f, cx, cy)

    # Convert numpy depth array to Open3D Image
    o3d_depth = o3d.geometry.Image(depth_image.astype(np.float32))

    # Create point cloud from depth image using Open3D
    o3d_cloud = o3d.geometry.PointCloud.create_from_depth_image(o3d_depth, cam_intrinsics)

    # Convert Open3D point cloud to numpy array
    camera_points = np.asarray(o3d_cloud.points)

    if camera_points.size == 0:
        return np.array([]).reshape(0, 3)

    # Flip y and z axes
    camera_points[:, 1] = -camera_points[:, 1]
    camera_points[:, 2] = -camera_points[:, 2]

    # Filter points based on depth range (note: z is now negative)
    valid_mask = (camera_points[:, 2] < -min_range) & (
        camera_points[:, 2] > -RANGE_FINDER_MAX_RANGE
    )
    camera_points = camera_points[valid_mask]

    if camera_points.size == 0:
        return np.array([]).reshape(0, 3)

    # Transform to world coordinates
    world_points = (camera_mat @ camera_points.T).T + camera_pos

    return world_points
