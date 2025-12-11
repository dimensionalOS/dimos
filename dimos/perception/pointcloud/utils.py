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

"""
Point cloud utilities for RGBD data processing.

This module provides efficient utilities for creating and manipulating point clouds
from RGBD images using Open3D.
"""

import os
from typing import Any

import cv2
import numpy as np
import open3d as o3d  # type: ignore[import-untyped]
from scipy.spatial import cKDTree  # type: ignore[import-untyped]
import yaml

from dimos.perception.common.utils import project_3d_points_to_2d


def load_camera_matrix_from_yaml(
    camera_info: str | list[float] | np.ndarray | dict | None,  # type: ignore[type-arg]
) -> np.ndarray | None:  # type: ignore[type-arg]
    """
    Load camera intrinsic matrix from various input formats.

    Args:
        camera_info: Can be:
            - Path to YAML file containing camera parameters
            - List of [fx, fy, cx, cy]
            - 3x3 numpy array (returned as-is)
            - Dict with camera parameters
            - None (returns None)

    Returns:
        3x3 camera intrinsic matrix or None if input is None

    Raises:
        ValueError: If camera_info format is invalid or file cannot be read
        FileNotFoundError: If YAML file path doesn't exist
    """
    if camera_info is None:
        return None

    # Handle case where camera_info is already a matrix
    if isinstance(camera_info, np.ndarray) and camera_info.shape == (3, 3):
        return camera_info.astype(np.float32)

    # Handle case where camera_info is [fx, fy, cx, cy] format
    if isinstance(camera_info, list) and len(camera_info) == 4:
        fx, fy, cx, cy = camera_info
        return np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float32)

    # Handle case where camera_info is a dict
    if isinstance(camera_info, dict):
        return _extract_matrix_from_dict(camera_info)

    # Handle case where camera_info is a path to a YAML file
    if isinstance(camera_info, str):
        if not os.path.isfile(camera_info):
            raise FileNotFoundError(f"Camera info file not found: {camera_info}")

        try:
            with open(camera_info) as f:
                data = yaml.safe_load(f)
            return _extract_matrix_from_dict(data)
        except Exception as e:
            raise ValueError(f"Failed to read camera info from {camera_info}: {e}")

    raise ValueError(
        f"Invalid camera_info format. Expected str, list, dict, or numpy array, got {type(camera_info)}"
    )


def _extract_matrix_from_dict(data: dict) -> np.ndarray:  # type: ignore[type-arg]
    """Extract camera matrix from dictionary with various formats."""
    # ROS format with 'K' field (most common)
    if "K" in data:
        k_data = data["K"]
        if len(k_data) == 9:
            return np.array(k_data, dtype=np.float32).reshape(3, 3)

    # Standard format with 'camera_matrix'
    if "camera_matrix" in data:
        if "data" in data["camera_matrix"]:
            matrix_data = data["camera_matrix"]["data"]
            if len(matrix_data) == 9:
                return np.array(matrix_data, dtype=np.float32).reshape(3, 3)

    # Explicit intrinsics format
    if all(k in data for k in ["fx", "fy", "cx", "cy"]):
        fx, fy = float(data["fx"]), float(data["fy"])
        cx, cy = float(data["cx"]), float(data["cy"])
        return np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float32)

    # Error case - provide helpful debug info
    available_keys = list(data.keys())
    if "K" in data:
        k_info = f"K field length: {len(data['K']) if hasattr(data['K'], '__len__') else 'unknown'}"
    else:
        k_info = "K field not found"

    raise ValueError(
        f"Cannot extract camera matrix from data. "
        f"Available keys: {available_keys}. {k_info}. "
        f"Expected formats: 'K' (9 elements), 'camera_matrix.data' (9 elements), "
        f"or individual 'fx', 'fy', 'cx', 'cy' fields."
    )


def create_o3d_point_cloud_from_rgbd(
    color_img: np.ndarray,  # type: ignore[type-arg]
    depth_img: np.ndarray,  # type: ignore[type-arg]
    intrinsic: np.ndarray,  # type: ignore[type-arg]
    depth_scale: float = 1.0,
    depth_trunc: float = 3.0,
) -> o3d.geometry.PointCloud:
    """
    Create an Open3D point cloud from RGB and depth images.

    Args:
        color_img: RGB image as numpy array (H, W, 3)
        depth_img: Depth image as numpy array (H, W)
        intrinsic: Camera intrinsic matrix (3x3 numpy array)
        depth_scale: Scale factor to convert depth to meters
        depth_trunc: Maximum depth in meters

    Returns:
        Open3D point cloud object

    Raises:
        ValueError: If input dimensions are invalid
    """
    # Validate inputs
    if len(color_img.shape) != 3 or color_img.shape[2] != 3:
        raise ValueError(f"color_img must be (H, W, 3), got {color_img.shape}")
    if len(depth_img.shape) != 2:
        raise ValueError(f"depth_img must be (H, W), got {depth_img.shape}")
    if color_img.shape[:2] != depth_img.shape:
        raise ValueError(
            f"Color and depth image dimensions don't match: {color_img.shape[:2]} vs {depth_img.shape}"
        )
    if intrinsic.shape != (3, 3):
        raise ValueError(f"intrinsic must be (3, 3), got {intrinsic.shape}")

    # Convert to Open3D format
    color_o3d = o3d.geometry.Image(color_img.astype(np.uint8))

    # Filter out inf and nan values from depth image
    depth_filtered = depth_img.copy()

    # Create mask for valid depth values (finite, positive, non-zero)
    valid_mask = np.isfinite(depth_filtered) & (depth_filtered > 0)

    # Set invalid values to 0 (which Open3D treats as no depth)
    depth_filtered[~valid_mask] = 0.0

    depth_o3d = o3d.geometry.Image(depth_filtered.astype(np.float32))

    # Create Open3D intrinsic object
    height, width = color_img.shape[:2]
    fx, fy = intrinsic[0, 0], intrinsic[1, 1]
    cx, cy = intrinsic[0, 2], intrinsic[1, 2]
    intrinsic_o3d = o3d.camera.PinholeCameraIntrinsic(
        width,
        height,
        fx,
        fy,  # fx, fy
        cx,
        cy,  # cx, cy
    )

    # Create RGBD image
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color_o3d,
        depth_o3d,
        depth_scale=depth_scale,
        depth_trunc=depth_trunc,
        convert_rgb_to_intensity=False,
    )

    # Create point cloud
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic_o3d)

    return pcd


def create_point_cloud_and_extract_masks(
    color_img: np.ndarray,  # type: ignore[type-arg]
    depth_img: np.ndarray,  # type: ignore[type-arg]
    masks: list[np.ndarray],  # type: ignore[type-arg]
    intrinsic: np.ndarray,  # type: ignore[type-arg]
    depth_scale: float = 1.0,
    depth_trunc: float = 3.0,
) -> tuple[o3d.geometry.PointCloud, list[o3d.geometry.PointCloud]]:
    """
    Efficiently create a point cloud once and extract multiple masked regions.

    Args:
        color_img: RGB image (H, W, 3)
        depth_img: Depth image (H, W)
        masks: List of boolean masks, each of shape (H, W)
        intrinsic: Camera intrinsic matrix (3x3 numpy array)
        depth_scale: Scale factor to convert depth to meters
        depth_trunc: Maximum depth in meters

    Returns:
        Tuple of (full_point_cloud, list_of_masked_point_clouds)
    """
    if not masks:
        return o3d.geometry.PointCloud(), []

    # Create the full point cloud
    full_pcd = create_o3d_point_cloud_from_rgbd(
        color_img, depth_img, intrinsic, depth_scale, depth_trunc
    )

    if len(np.asarray(full_pcd.points)) == 0:
        return full_pcd, [o3d.geometry.PointCloud() for _ in masks]

    # Create pixel-to-point mapping
    valid_depth_mask = np.isfinite(depth_img) & (depth_img > 0) & (depth_img <= depth_trunc)

    valid_depth = valid_depth_mask.flatten()
    if not np.any(valid_depth):
        return full_pcd, [o3d.geometry.PointCloud() for _ in masks]

    pixel_to_point = np.full(len(valid_depth), -1, dtype=np.int32)
    pixel_to_point[valid_depth] = np.arange(np.sum(valid_depth))

    # Extract point clouds for each mask
    masked_pcds = []
    max_points = len(np.asarray(full_pcd.points))

    for mask in masks:
        if mask.shape != depth_img.shape:
            masked_pcds.append(o3d.geometry.PointCloud())
            continue

        mask_flat = mask.flatten()
        valid_mask_indices = mask_flat & valid_depth
        point_indices = pixel_to_point[valid_mask_indices]
        valid_point_indices = point_indices[point_indices >= 0]

        if len(valid_point_indices) > 0:
            valid_point_indices = np.clip(valid_point_indices, 0, max_points - 1)
            valid_point_indices = np.unique(valid_point_indices)
            masked_pcd = full_pcd.select_by_index(valid_point_indices.tolist())
        else:
            masked_pcd = o3d.geometry.PointCloud()

        masked_pcds.append(masked_pcd)

    return full_pcd, masked_pcds


def filter_point_cloud_statistical(
    pcd: o3d.geometry.PointCloud, nb_neighbors: int = 20, std_ratio: float = 2.0
) -> tuple[o3d.geometry.PointCloud, np.ndarray]:  # type: ignore[type-arg]
    """
    Apply statistical outlier filtering to point cloud.

    Args:
        pcd: Input point cloud
        nb_neighbors: Number of neighbors to analyze for each point
        std_ratio: Threshold level based on standard deviation

    Returns:
        Tuple of (filtered_point_cloud, outlier_indices)
    """
    if len(np.asarray(pcd.points)) == 0:
        return pcd, np.array([])

    return pcd.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)  # type: ignore[no-any-return]


def filter_point_cloud_radius(
    pcd: o3d.geometry.PointCloud, nb_points: int = 16, radius: float = 0.05
) -> tuple[o3d.geometry.PointCloud, np.ndarray]:  # type: ignore[type-arg]
    """
    Apply radius-based outlier filtering to point cloud.

    Args:
        pcd: Input point cloud
        nb_points: Minimum number of points within radius
        radius: Search radius in meters

    Returns:
        Tuple of (filtered_point_cloud, outlier_indices)
    """
    if len(np.asarray(pcd.points)) == 0:
        return pcd, np.array([])

    return pcd.remove_radius_outlier(nb_points=nb_points, radius=radius)  # type: ignore[no-any-return]


def overlay_point_clouds_on_image(
    base_image: np.ndarray,  # type: ignore[type-arg]
    point_clouds: list[o3d.geometry.PointCloud],
    camera_intrinsics: list[float] | np.ndarray,  # type: ignore[type-arg]
    colors: list[tuple[int, int, int]],
    point_size: int = 2,
    alpha: float = 0.7,
) -> np.ndarray:  # type: ignore[type-arg]
    """
    Overlay multiple colored point clouds onto an image.

    Args:
        base_image: Base image to overlay onto (H, W, 3) - assumed to be RGB
        point_clouds: List of Open3D point cloud objects
        camera_intrinsics: Camera parameters as [fx, fy, cx, cy] list or 3x3 matrix
        colors: List of RGB color tuples for each point cloud. If None, generates distinct colors.
        point_size: Size of points to draw (in pixels)
        alpha: Blending factor for overlay (0.0 = fully transparent, 1.0 = fully opaque)

    Returns:
        Image with overlaid point clouds (H, W, 3)
    """
    if len(point_clouds) == 0:
        return base_image.copy()

    # Create overlay image
    overlay = base_image.copy()
    height, width = base_image.shape[:2]

    # Process each point cloud
    for i, pcd in enumerate(point_clouds):
        if pcd is None:
            continue

        points_3d = np.asarray(pcd.points)
        if len(points_3d) == 0:
            continue

        # Project 3D points to 2D
        points_2d = project_3d_points_to_2d(points_3d, camera_intrinsics)

        if len(points_2d) == 0:
            continue

        # Filter points within image bounds
        valid_mask = (
            (points_2d[:, 0] >= 0)
            & (points_2d[:, 0] < width)
            & (points_2d[:, 1] >= 0)
            & (points_2d[:, 1] < height)
        )
        valid_points_2d = points_2d[valid_mask]

        if len(valid_points_2d) == 0:
            continue

        # Get color for this point cloud
        color = colors[i % len(colors)]

        # Ensure color is a tuple of integers for OpenCV
        if isinstance(color, list | tuple | np.ndarray):
            color = tuple(int(c) for c in color[:3])  # type: ignore[assignment]
        else:
            color = (255, 255, 255)

        # Draw points on overlay
        for point in valid_points_2d:
            u, v = point
            # Draw a small filled circle for each point
            cv2.circle(overlay, (u, v), point_size, color, -1)

    # Blend overlay with base image
    result = cv2.addWeighted(base_image, 1 - alpha, overlay, alpha, 0)

    return result


def create_point_cloud_overlay_visualization(
    base_image: np.ndarray,  # type: ignore[type-arg]
    objects: list[dict],  # type: ignore[type-arg]
    intrinsics: np.ndarray,  # type: ignore[type-arg]
) -> np.ndarray:  # type: ignore[type-arg]
    """
    Create a visualization showing object point clouds and bounding boxes overlaid on a base image.

    Args:
        base_image: Base image to overlay onto (H, W, 3)
        objects: List of object dictionaries containing 'point_cloud', 'color', 'position', 'rotation', 'size' keys
        intrinsics: Camera intrinsics as [fx, fy, cx, cy] or 3x3 matrix

    Returns:
        Visualization image with overlaid point clouds and bounding boxes (H, W, 3)
    """
    # Extract point clouds and colors from objects
    point_clouds = []
    colors = []
    for obj in objects:
        if "point_cloud" in obj and obj["point_cloud"] is not None:
            point_clouds.append(obj["point_cloud"])

            # Convert color to tuple
            color = obj["color"]
            if isinstance(color, np.ndarray):
                color = tuple(int(c) for c in color)
            elif isinstance(color, list | tuple):
                color = tuple(int(c) for c in color[:3])
            colors.append(color)

    # Create visualization
    if point_clouds:
        result = overlay_point_clouds_on_image(
            base_image=base_image,
            point_clouds=point_clouds,
            camera_intrinsics=intrinsics,
            colors=colors,
            point_size=3,
            alpha=0.8,
        )
    else:
        result = base_image.copy()

    # Draw 3D bounding boxes
    height_img, width_img = result.shape[:2]
    for i, obj in enumerate(objects):
        if all(key in obj and obj[key] is not None for key in ["position", "rotation", "size"]):
            try:
                # Create and project 3D bounding box
                corners_3d = create_3d_bounding_box_corners(
                    obj["position"], obj["rotation"], obj["size"]
                )
                corners_2d = project_3d_points_to_2d(corners_3d, intrinsics)

                # Check if any corners are visible
                valid_mask = (
                    (corners_2d[:, 0] >= 0)
                    & (corners_2d[:, 0] < width_img)
                    & (corners_2d[:, 1] >= 0)
                    & (corners_2d[:, 1] < height_img)
                )

                if np.any(valid_mask):
                    # Get color
                    bbox_color = colors[i] if i < len(colors) else (255, 255, 255)
                    draw_3d_bounding_box_on_image(result, corners_2d, bbox_color, thickness=2)
            except:
                continue

    return result


def create_3d_bounding_box_corners(position, rotation, size: int):  # type: ignore[no-untyped-def]
    """
    Create 8 corners of a 3D bounding box from position, rotation, and size.

    Args:
        position: Vector or dict with x, y, z coordinates
        rotation: Vector or dict with roll, pitch, yaw angles
        size: Dict with width, height, depth

    Returns:
        8x3 numpy array of corner coordinates
    """
    # Convert position to numpy array
    if hasattr(position, "x"):  # Vector object
        center = np.array([position.x, position.y, position.z])
    else:  # Dictionary
        center = np.array([position["x"], position["y"], position["z"]])

    # Convert rotation (euler angles) to rotation matrix
    if hasattr(rotation, "x"):  # Vector object (roll, pitch, yaw)
        roll, pitch, yaw = rotation.x, rotation.y, rotation.z
    else:  # Dictionary
        roll, pitch, yaw = rotation["roll"], rotation["pitch"], rotation["yaw"]

    # Create rotation matrix from euler angles (ZYX order)
    cos_r, sin_r = np.cos(roll), np.sin(roll)
    cos_p, sin_p = np.cos(pitch), np.sin(pitch)
    cos_y, sin_y = np.cos(yaw), np.sin(yaw)

    # Rotation matrix for ZYX euler angles
    R = np.array(
        [
            [
                cos_y * cos_p,
                cos_y * sin_p * sin_r - sin_y * cos_r,
                cos_y * sin_p * cos_r + sin_y * sin_r,
            ],
            [
                sin_y * cos_p,
                sin_y * sin_p * sin_r + cos_y * cos_r,
                sin_y * sin_p * cos_r - cos_y * sin_r,
            ],
            [-sin_p, cos_p * sin_r, cos_p * cos_r],
        ]
    )

    # Get dimensions
    width = size.get("width", 0.1)  # type: ignore[attr-defined]
    height = size.get("height", 0.1)  # type: ignore[attr-defined]
    depth = size.get("depth", 0.1)  # type: ignore[attr-defined]

    # Create 8 corners of the bounding box (before rotation)
    corners = np.array(
        [
            [-width / 2, -height / 2, -depth / 2],  # 0
            [width / 2, -height / 2, -depth / 2],  # 1
            [width / 2, height / 2, -depth / 2],  # 2
            [-width / 2, height / 2, -depth / 2],  # 3
            [-width / 2, -height / 2, depth / 2],  # 4
            [width / 2, -height / 2, depth / 2],  # 5
            [width / 2, height / 2, depth / 2],  # 6
            [-width / 2, height / 2, depth / 2],  # 7
        ]
    )

    # Apply rotation and translation
    rotated_corners = corners @ R.T + center

    return rotated_corners


def draw_3d_bounding_box_on_image(image, corners_2d, color, thickness: int = 2) -> None:  # type: ignore[no-untyped-def]
    """
    Draw a 3D bounding box on an image using projected 2D corners.

    Args:
        image: Image to draw on
        corners_2d: 8x2 array of 2D corner coordinates
        color: RGB color tuple
        thickness: Line thickness
    """
    # Define the 12 edges of a cube (connecting corner indices)
    edges = [
        (0, 1),
        (1, 2),
        (2, 3),
        (3, 0),  # Bottom face
        (4, 5),
        (5, 6),
        (6, 7),
        (7, 4),  # Top face
        (0, 4),
        (1, 5),
        (2, 6),
        (3, 7),  # Vertical edges
    ]

    # Draw each edge
    for start_idx, end_idx in edges:
        start_point = tuple(corners_2d[start_idx].astype(int))
        end_point = tuple(corners_2d[end_idx].astype(int))
        cv2.line(image, start_point, end_point, color, thickness)


def extract_and_cluster_misc_points(
    full_pcd: o3d.geometry.PointCloud,
    all_objects: list[dict],  # type: ignore[type-arg]
    eps: float = 0.03,
    min_points: int = 100,
    enable_filtering: bool = True,
    voxel_size: float = 0.02,
) -> tuple[list[o3d.geometry.PointCloud], o3d.geometry.VoxelGrid]:
    """
    Extract miscellaneous/background points and cluster them using DBSCAN.

    Args:
        full_pcd: Complete scene point cloud
        all_objects: List of objects with point clouds to subtract
        eps: DBSCAN epsilon parameter (max distance between points in cluster)
        min_points: DBSCAN min_samples parameter (min points to form cluster)
        enable_filtering: Whether to apply statistical and radius filtering
        voxel_size: Size of voxels for voxel grid generation

    Returns:
        Tuple of (clustered_point_clouds, voxel_grid)
    """
    if full_pcd is None or len(np.asarray(full_pcd.points)) == 0:
        return [], o3d.geometry.VoxelGrid()

    if not all_objects:
        # If no objects detected, cluster the full point cloud
        clusters = _cluster_point_cloud_dbscan(full_pcd, eps, min_points)
        voxel_grid = _create_voxel_grid_from_clusters(clusters, voxel_size)
        return clusters, voxel_grid

    try:
        # Start with a copy of the full point cloud
        misc_pcd = o3d.geometry.PointCloud(full_pcd)

        # Remove object points by combining all object point clouds
        all_object_points = []
        for obj in all_objects:
            if "point_cloud" in obj and obj["point_cloud"] is not None:
                obj_points = np.asarray(obj["point_cloud"].points)
                if len(obj_points) > 0:
                    all_object_points.append(obj_points)

        if not all_object_points:
            # No object points to remove, cluster full point cloud
            clusters = _cluster_point_cloud_dbscan(misc_pcd, eps, min_points)
            voxel_grid = _create_voxel_grid_from_clusters(clusters, voxel_size)
            return clusters, voxel_grid

        # Combine all object points
        combined_obj_points = np.vstack(all_object_points)

        # For efficiency, downsample both point clouds
        misc_downsampled = misc_pcd.voxel_down_sample(voxel_size=0.005)

        # Create object point cloud for efficient operations
        obj_pcd = o3d.geometry.PointCloud()
        obj_pcd.points = o3d.utility.Vector3dVector(combined_obj_points)
        obj_downsampled = obj_pcd.voxel_down_sample(voxel_size=0.005)

        misc_points = np.asarray(misc_downsampled.points)
        obj_points_down = np.asarray(obj_downsampled.points)

        if len(misc_points) == 0 or len(obj_points_down) == 0:
            clusters = _cluster_point_cloud_dbscan(misc_downsampled, eps, min_points)
            voxel_grid = _create_voxel_grid_from_clusters(clusters, voxel_size)
            return clusters, voxel_grid

        # Build tree for object points
        obj_tree = cKDTree(obj_points_down)

        # Find distances from misc points to nearest object points
        distances, _ = obj_tree.query(misc_points, k=1)

        # Keep points that are far enough from any object point
        threshold = 0.015  # 1.5cm threshold
        keep_mask = distances > threshold

        if not np.any(keep_mask):
            return [], o3d.geometry.VoxelGrid()

        # Filter misc points
        misc_indices = np.where(keep_mask)[0]
        final_misc_pcd = misc_downsampled.select_by_index(misc_indices)

        if len(np.asarray(final_misc_pcd.points)) == 0:
            return [], o3d.geometry.VoxelGrid()

        # Apply additional filtering if enabled
        if enable_filtering:
            # Apply statistical outlier filtering
            filtered_misc_pcd, _ = filter_point_cloud_statistical(
                final_misc_pcd, nb_neighbors=30, std_ratio=2.0
            )

            if len(np.asarray(filtered_misc_pcd.points)) == 0:
                return [], o3d.geometry.VoxelGrid()

            # Apply radius outlier filtering
            final_filtered_misc_pcd, _ = filter_point_cloud_radius(
                filtered_misc_pcd,
                nb_points=20,
                radius=0.03,  # 3cm radius
            )

            if len(np.asarray(final_filtered_misc_pcd.points)) == 0:
                return [], o3d.geometry.VoxelGrid()

            final_misc_pcd = final_filtered_misc_pcd

        # Cluster the misc points using DBSCAN
        clusters = _cluster_point_cloud_dbscan(final_misc_pcd, eps, min_points)

        # Create voxel grid from all misc points (before clustering)
        voxel_grid = _create_voxel_grid_from_point_cloud(final_misc_pcd, voxel_size)

        return clusters, voxel_grid

    except Exception as e:
        print(f"Error in misc point extraction and clustering: {e}")
        # Fallback: return downsampled full point cloud as single cluster
        try:
            downsampled = full_pcd.voxel_down_sample(voxel_size=0.02)
            if len(np.asarray(downsampled.points)) > 0:
                voxel_grid = _create_voxel_grid_from_point_cloud(downsampled, voxel_size)
                return [downsampled], voxel_grid
            else:
                return [], o3d.geometry.VoxelGrid()
        except:
            return [], o3d.geometry.VoxelGrid()


def _create_voxel_grid_from_point_cloud(
    pcd: o3d.geometry.PointCloud, voxel_size: float = 0.02
) -> o3d.geometry.VoxelGrid:
    """
    Create a voxel grid from a point cloud.

    Args:
        pcd: Input point cloud
        voxel_size: Size of each voxel

    Returns:
        Open3D VoxelGrid object
    """
    if len(np.asarray(pcd.points)) == 0:
        return o3d.geometry.VoxelGrid()

    try:
        # Create voxel grid from point cloud
        voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size)

        # Color the voxels with a semi-transparent gray
        for voxel in voxel_grid.get_voxels():
            voxel.color = [0.5, 0.5, 0.5]  # Gray color

        print(
            f"Created voxel grid with {len(voxel_grid.get_voxels())} voxels (voxel_size={voxel_size})"
        )
        return voxel_grid

    except Exception as e:
        print(f"Error creating voxel grid: {e}")
        return o3d.geometry.VoxelGrid()


def _create_voxel_grid_from_clusters(
    clusters: list[o3d.geometry.PointCloud], voxel_size: float = 0.02
) -> o3d.geometry.VoxelGrid:
    """
    Create a voxel grid from multiple clustered point clouds.

    Args:
        clusters: List of clustered point clouds
        voxel_size: Size of each voxel

    Returns:
        Open3D VoxelGrid object
    """
    if not clusters:
        return o3d.geometry.VoxelGrid()

    # Combine all clusters into one point cloud
    combined_points = []
    for cluster in clusters:
        points = np.asarray(cluster.points)
        if len(points) > 0:
            combined_points.append(points)

    if not combined_points:
        return o3d.geometry.VoxelGrid()

    # Create combined point cloud
    all_points = np.vstack(combined_points)
    combined_pcd = o3d.geometry.PointCloud()
    combined_pcd.points = o3d.utility.Vector3dVector(all_points)

    return _create_voxel_grid_from_point_cloud(combined_pcd, voxel_size)


def _cluster_point_cloud_dbscan(
    pcd: o3d.geometry.PointCloud, eps: float = 0.05, min_points: int = 50
) -> list[o3d.geometry.PointCloud]:
    """
    Cluster a point cloud using DBSCAN and return list of clustered point clouds.

    Args:
        pcd: Point cloud to cluster
        eps: DBSCAN epsilon parameter
        min_points: DBSCAN min_samples parameter

    Returns:
        List of point clouds, one for each cluster
    """
    if len(np.asarray(pcd.points)) == 0:
        return []

    try:
        # Apply DBSCAN clustering
        labels = np.array(pcd.cluster_dbscan(eps=eps, min_points=min_points))

        # Get unique cluster labels (excluding noise points labeled as -1)
        unique_labels = np.unique(labels)
        cluster_pcds = []

        for label in unique_labels:
            if label == -1:  # Skip noise points
                continue

            # Get indices for this cluster
            cluster_indices = np.where(labels == label)[0]

            if len(cluster_indices) > 0:
                # Create point cloud for this cluster
                cluster_pcd = pcd.select_by_index(cluster_indices)

                # Assign a random color to this cluster
                cluster_color = np.random.rand(3)  # Random RGB color
                cluster_pcd.paint_uniform_color(cluster_color)

                cluster_pcds.append(cluster_pcd)

        print(
            f"DBSCAN clustering found {len(cluster_pcds)} clusters from {len(np.asarray(pcd.points))} points"
        )
        return cluster_pcds

    except Exception as e:
        print(f"Error in DBSCAN clustering: {e}")
        return [pcd]  # Return original point cloud as fallback


def get_standard_coordinate_transform():  # type: ignore[no-untyped-def]
    """
    Get a standard coordinate transformation matrix for consistent visualization.

    This transformation ensures that:
    - X (red) axis points right
    - Y (green) axis points up
    - Z (blue) axis points toward viewer

    Returns:
        4x4 transformation matrix
    """
    # Standard transformation matrix to ensure consistent coordinate frame orientation
    transform = np.array(
        [
            [1, 0, 0, 0],  # X points right
            [0, -1, 0, 0],  # Y points up (flip from OpenCV to standard)
            [0, 0, -1, 0],  # Z points toward viewer (flip depth)
            [0, 0, 0, 1],
        ]
    )
    return transform


def visualize_clustered_point_clouds(
    clustered_pcds: list[o3d.geometry.PointCloud],
    window_name: str = "Clustered Point Clouds",
    point_size: float = 2.0,
    show_coordinate_frame: bool = True,
    coordinate_frame_size: float = 0.1,
) -> None:
    """
    Visualize multiple clustered point clouds with different colors.

    Args:
        clustered_pcds: List of point clouds (already colored)
        window_name: Name of the visualization window
        point_size: Size of points in the visualization
        show_coordinate_frame: Whether to show coordinate frame
        coordinate_frame_size: Size of the coordinate frame
    """
    if not clustered_pcds:
        print("Warning: No clustered point clouds to visualize")
        return

    # Apply standard coordinate transformation
    transform = get_standard_coordinate_transform()  # type: ignore[no-untyped-call]
    geometries = []
    for pcd in clustered_pcds:
        pcd_copy = o3d.geometry.PointCloud(pcd)
        pcd_copy.transform(transform)
        geometries.append(pcd_copy)

    # Add coordinate frame
    if show_coordinate_frame:
        coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=coordinate_frame_size
        )
        coordinate_frame.transform(transform)
        geometries.append(coordinate_frame)

    total_points = sum(len(np.asarray(pcd.points)) for pcd in clustered_pcds)
    print(f"Visualizing {len(clustered_pcds)} clusters with {total_points} total points")

    try:
        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name=window_name, width=1280, height=720)
        for geom in geometries:
            vis.add_geometry(geom)
        render_option = vis.get_render_option()
        render_option.point_size = point_size
        vis.run()
        vis.destroy_window()
    except Exception as e:
        print(f"Failed to create interactive visualization: {e}")
        o3d.visualization.draw_geometries(
            geometries, window_name=window_name, width=1280, height=720
        )


def visualize_pcd(
    pcd: o3d.geometry.PointCloud,
    window_name: str = "Point Cloud Visualization",
    point_size: float = 1.0,
    show_coordinate_frame: bool = True,
    coordinate_frame_size: float = 0.1,
) -> None:
    """
    Visualize an Open3D point cloud using Open3D's visualization window.

    Args:
        pcd: Open3D point cloud to visualize
        window_name: Name of the visualization window
        point_size: Size of points in the visualization
        show_coordinate_frame: Whether to show coordinate frame
        coordinate_frame_size: Size of the coordinate frame
    """
    if pcd is None:
        print("Warning: Point cloud is None, nothing to visualize")
        return

    if len(np.asarray(pcd.points)) == 0:
        print("Warning: Point cloud is empty, nothing to visualize")
        return

    # Apply standard coordinate transformation
    transform = get_standard_coordinate_transform()  # type: ignore[no-untyped-call]
    pcd_copy = o3d.geometry.PointCloud(pcd)
    pcd_copy.transform(transform)
    geometries = [pcd_copy]

    # Add coordinate frame
    if show_coordinate_frame:
        coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=coordinate_frame_size
        )
        coordinate_frame.transform(transform)
        geometries.append(coordinate_frame)

    print(f"Visualizing point cloud with {len(np.asarray(pcd.points))} points")

    try:
        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name=window_name, width=1280, height=720)
        for geom in geometries:
            vis.add_geometry(geom)
        render_option = vis.get_render_option()
        render_option.point_size = point_size
        vis.run()
        vis.destroy_window()
    except Exception as e:
        print(f"Failed to create interactive visualization: {e}")
        o3d.visualization.draw_geometries(
            geometries, window_name=window_name, width=1280, height=720
        )


def visualize_voxel_grid(
    voxel_grid: o3d.geometry.VoxelGrid,
    window_name: str = "Voxel Grid Visualization",
    show_coordinate_frame: bool = True,
    coordinate_frame_size: float = 0.1,
) -> None:
    """
    Visualize an Open3D voxel grid using Open3D's visualization window.

    Args:
        voxel_grid: Open3D voxel grid to visualize
        window_name: Name of the visualization window
        show_coordinate_frame: Whether to show coordinate frame
        coordinate_frame_size: Size of the coordinate frame
    """
    if voxel_grid is None:
        print("Warning: Voxel grid is None, nothing to visualize")
        return

    if len(voxel_grid.get_voxels()) == 0:
        print("Warning: Voxel grid is empty, nothing to visualize")
        return

    # VoxelGrid doesn't support transform, so we need to transform the source points instead
    # For now, just visualize as-is with transformed coordinate frame
    geometries = [voxel_grid]

    # Add coordinate frame
    if show_coordinate_frame:
        coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=coordinate_frame_size
        )
        coordinate_frame.transform(get_standard_coordinate_transform())  # type: ignore[no-untyped-call]
        geometries.append(coordinate_frame)

    print(f"Visualizing voxel grid with {len(voxel_grid.get_voxels())} voxels")

    try:
        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name=window_name, width=1280, height=720)
        for geom in geometries:
            vis.add_geometry(geom)
        vis.run()
        vis.destroy_window()
    except Exception as e:
        print(f"Failed to create interactive visualization: {e}")
        o3d.visualization.draw_geometries(
            geometries, window_name=window_name, width=1280, height=720
        )


def combine_object_pointclouds(
    point_clouds: list[np.ndarray] | list[o3d.geometry.PointCloud],  # type: ignore[type-arg]
    colors: list[np.ndarray] | None = None,  # type: ignore[type-arg]
) -> o3d.geometry.PointCloud:
    """
    Combine multiple point clouds into a single Open3D point cloud.

    Args:
        point_clouds: List of point clouds as numpy arrays or Open3D point clouds
        colors: List of colors as numpy arrays
    Returns:
        Combined Open3D point cloud
    """
    all_points = []
    all_colors = []

    for i, pcd in enumerate(point_clouds):
        if isinstance(pcd, np.ndarray):
            points = pcd[:, :3]
            all_points.append(points)
            if colors:
                all_colors.append(colors[i])

        elif isinstance(pcd, o3d.geometry.PointCloud):
            points = np.asarray(pcd.points)
            all_points.append(points)
            if pcd.has_colors():
                colors = np.asarray(pcd.colors)  # type: ignore[assignment]
                all_colors.append(colors)  # type: ignore[arg-type]

    if not all_points:
        return o3d.geometry.PointCloud()

    combined_pcd = o3d.geometry.PointCloud()
    combined_pcd.points = o3d.utility.Vector3dVector(np.vstack(all_points))

    if all_colors:
        combined_pcd.colors = o3d.utility.Vector3dVector(np.vstack(all_colors))

    return combined_pcd


def extract_centroids_from_masks(
    rgb_image: np.ndarray,  # type: ignore[type-arg]
    depth_image: np.ndarray,  # type: ignore[type-arg]
    masks: list[np.ndarray],  # type: ignore[type-arg]
    camera_intrinsics: list[float] | np.ndarray,  # type: ignore[type-arg]
) -> list[dict[str, Any]]:
    """
    Extract 3D centroids and orientations from segmentation masks.

    Args:
        rgb_image: RGB image (H, W, 3)
        depth_image: Depth image (H, W) in meters
        masks: List of boolean masks (H, W)
        camera_intrinsics: Camera parameters as [fx, fy, cx, cy] or 3x3 matrix

    Returns:
        List of dictionaries containing:
            - centroid: 3D centroid position [x, y, z] in camera frame
            - orientation: Normalized direction vector from camera to centroid
            - num_points: Number of valid 3D points
            - mask_idx: Index of the mask in the input list
    """
    # Extract camera parameters
    if isinstance(camera_intrinsics, list) and len(camera_intrinsics) == 4:
        fx, fy, cx, cy = camera_intrinsics
    else:
        fx = camera_intrinsics[0, 0]  # type: ignore[call-overload]
        fy = camera_intrinsics[1, 1]  # type: ignore[call-overload]
        cx = camera_intrinsics[0, 2]  # type: ignore[call-overload]
        cy = camera_intrinsics[1, 2]  # type: ignore[call-overload]

    results = []

    for mask_idx, mask in enumerate(masks):
        if mask is None or mask.sum() == 0:
            continue

        # Get pixel coordinates where mask is True
        y_coords, x_coords = np.where(mask)

        # Get depth values at mask locations
        depths = depth_image[y_coords, x_coords]

        # Convert to 3D points in camera frame
        X = (x_coords - cx) * depths / fx
        Y = (y_coords - cy) * depths / fy
        Z = depths

        # Calculate centroid
        centroid_x = np.mean(X)
        centroid_y = np.mean(Y)
        centroid_z = np.mean(Z)
        centroid = np.array([centroid_x, centroid_y, centroid_z])

        # Calculate orientation as normalized direction from camera origin to centroid
        # Camera origin is at (0, 0, 0)
        orientation = centroid / np.linalg.norm(centroid)

        results.append(
            {
                "centroid": centroid,
                "orientation": orientation,
                "num_points": int(mask.sum()),
                "mask_idx": mask_idx,
            }
        )

    return results
