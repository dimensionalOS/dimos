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


import numpy as np
import open3d as o3d  # type: ignore[import-untyped]


def fit_cuboid(
    points: np.ndarray | o3d.geometry.PointCloud,  # type: ignore[type-arg]
    method: str = "minimal",
) -> dict | None:  # type: ignore[type-arg]
    """
    Fit a cuboid to a point cloud using Open3D's built-in methods.

    Args:
        points: Nx3 array of points or Open3D PointCloud
        method: Fitting method:
            - 'minimal': Minimal oriented bounding box (best fit)
            - 'oriented': PCA-based oriented bounding box
            - 'axis_aligned': Axis-aligned bounding box

    Returns:
        Dictionary containing:
            - center: 3D center point
            - dimensions: 3D dimensions (extent)
            - rotation: 3x3 rotation matrix
            - error: Fitting error
            - bounding_box: Open3D OrientedBoundingBox object
        Returns None if insufficient points or fitting fails.

    Raises:
        ValueError: If method is invalid or inputs are malformed
    """
    # Validate method
    valid_methods = ["minimal", "oriented", "axis_aligned"]
    if method not in valid_methods:
        raise ValueError(f"method must be one of {valid_methods}, got '{method}'")

    # Convert to point cloud if needed
    if isinstance(points, np.ndarray):
        points = np.asarray(points)
        if len(points.shape) != 2 or points.shape[1] != 3:
            raise ValueError(f"points array must be Nx3, got shape {points.shape}")
        if len(points) < 4:
            return None

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
    elif isinstance(points, o3d.geometry.PointCloud):
        pcd = points
        points = np.asarray(pcd.points)
        if len(points) < 4:
            return None
    else:
        raise ValueError(f"points must be numpy array or Open3D PointCloud, got {type(points)}")

    try:
        # Get bounding box based on method
        if method == "minimal":
            obb = pcd.get_minimal_oriented_bounding_box(robust=True)
        elif method == "oriented":
            obb = pcd.get_oriented_bounding_box(robust=True)
        elif method == "axis_aligned":
            # Convert axis-aligned to oriented format for consistency
            aabb = pcd.get_axis_aligned_bounding_box()
            obb = o3d.geometry.OrientedBoundingBox()
            obb.center = aabb.get_center()
            obb.extent = aabb.get_extent()
            obb.R = np.eye(3)  # Identity rotation for axis-aligned

        # Extract parameters
        center = np.asarray(obb.center)
        dimensions = np.asarray(obb.extent)
        rotation = np.asarray(obb.R)

        # Calculate fitting error
        error = _compute_fitting_error(points, center, dimensions, rotation)

        return {
            "center": center,
            "dimensions": dimensions,
            "rotation": rotation,
            "error": error,
            "bounding_box": obb,
            "method": method,
        }

    except Exception as e:
        # Log error but don't crash - return None for graceful handling
        print(f"Warning: Cuboid fitting failed with method '{method}': {e}")
        return None


def _compute_fitting_error(
    points: np.ndarray,  # type: ignore[type-arg]
    center: np.ndarray,  # type: ignore[type-arg]
    dimensions: np.ndarray,  # type: ignore[type-arg]
    rotation: np.ndarray,  # type: ignore[type-arg]
) -> float:
    """
    Compute fitting error as mean squared distance from points to cuboid surface.

    Args:
        points: Nx3 array of points
        center: 3D center point
        dimensions: 3D dimensions
        rotation: 3x3 rotation matrix

    Returns:
        Mean squared error
    """
    if len(points) == 0:
        return 0.0

    # Transform points to local coordinates
    local_points = (points - center) @ rotation
    half_dims = dimensions / 2

    # Calculate distance to cuboid surface
    dx = np.abs(local_points[:, 0]) - half_dims[0]
    dy = np.abs(local_points[:, 1]) - half_dims[1]
    dz = np.abs(local_points[:, 2]) - half_dims[2]

    # Points outside: distance to nearest face
    # Points inside: negative distance to nearest face
    outside_dist = np.sqrt(np.maximum(dx, 0) ** 2 + np.maximum(dy, 0) ** 2 + np.maximum(dz, 0) ** 2)
    inside_dist = np.minimum(np.minimum(dx, dy), dz)
    distances = np.where((dx > 0) | (dy > 0) | (dz > 0), outside_dist, -inside_dist)

    return float(np.mean(distances**2))
