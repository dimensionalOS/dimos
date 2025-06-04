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

import numpy as np
import yaml
import os
import cv2
import open3d as o3d
from typing import List, Optional, Tuple, Union


def depth_to_point_cloud(depth_image, camera_matrix, subsample_factor=4):
    """
    Convert depth image to point cloud using camera intrinsics.
    Subsamples points to reduce density.

    Args:
        depth_image: HxW depth image in meters
        camera_matrix: 3x3 camera intrinsic matrix
        subsample_factor: Factor to subsample points (higher = fewer points)

    Returns:
        Nx3 array of 3D points
    """
    # Filter out inf and nan values from depth image
    depth_filtered = depth_image.copy()

    # Create mask for valid depth values (finite, positive, non-zero)
    valid_mask = np.isfinite(depth_filtered) & (depth_filtered > 0)

    # Set invalid values to 0
    depth_filtered[~valid_mask] = 0.0

    # Get focal length and principal point from camera matrix
    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]
    cx = camera_matrix[0, 2]
    cy = camera_matrix[1, 2]

    # Create pixel coordinate grid
    rows, cols = depth_filtered.shape
    x_grid, y_grid = np.meshgrid(
        np.arange(0, cols, subsample_factor), np.arange(0, rows, subsample_factor)
    )

    # Flatten grid and depth
    x = x_grid.flatten()
    y = y_grid.flatten()
    z = depth_filtered[y_grid, x_grid].flatten()

    # Remove points with invalid depth (after filtering, this catches zeros)
    valid = z > 0
    x = x[valid]
    y = y[valid]
    z = z[valid]

    # Convert to 3D points
    X = (x - cx) * z / fx
    Y = (y - cy) * z / fy
    Z = z

    return np.column_stack([X, Y, Z])


def load_camera_matrix_from_yaml(
    camera_info: Optional[Union[str, List[float], np.ndarray, dict]],
) -> Optional[np.ndarray]:
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
            with open(camera_info, "r") as f:
                data = yaml.safe_load(f)
            return _extract_matrix_from_dict(data)
        except Exception as e:
            raise ValueError(f"Failed to read camera info from {camera_info}: {e}")

    raise ValueError(
        f"Invalid camera_info format. Expected str, list, dict, or numpy array, got {type(camera_info)}"
    )


def _extract_matrix_from_dict(data: dict) -> np.ndarray:
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
    color_img: np.ndarray,
    depth_img: np.ndarray,
    intrinsic: np.ndarray,
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
    intrinsic_o3d = o3d.camera.PinholeCameraIntrinsic(
        width,
        height,
        intrinsic[0, 0],
        intrinsic[1, 1],  # fx, fy
        intrinsic[0, 2],
        intrinsic[1, 2],  # cx, cy
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


def o3d_point_cloud_to_numpy(pcd: o3d.geometry.PointCloud) -> np.ndarray:
    """
    Convert Open3D point cloud to numpy array of XYZRGB points.

    Args:
        pcd: Open3D point cloud object

    Returns:
        Nx6 array of XYZRGB points (empty array if no points)
    """
    points = np.asarray(pcd.points)
    if len(points) == 0:
        return np.zeros((0, 6), dtype=np.float32)

    # Get colors if available
    if pcd.has_colors():
        colors = np.asarray(pcd.colors) * 255.0  # Convert from [0,1] to [0,255]
        return np.column_stack([points, colors]).astype(np.float32)
    else:
        # No colors available, return points with zero colors
        zeros = np.zeros((len(points), 3), dtype=np.float32)
        return np.column_stack([points, zeros]).astype(np.float32)


def numpy_to_o3d_point_cloud(points_rgb: np.ndarray) -> o3d.geometry.PointCloud:
    """
    Convert numpy array of XYZRGB points to Open3D point cloud.

    Args:
        points_rgb: Nx6 array of XYZRGB points or Nx3 array of XYZ points

    Returns:
        Open3D point cloud object

    Raises:
        ValueError: If array shape is invalid
    """
    if len(points_rgb) == 0:
        return o3d.geometry.PointCloud()

    if points_rgb.shape[1] < 3:
        raise ValueError(
            f"points_rgb must have at least 3 columns (XYZ), got {points_rgb.shape[1]}"
        )

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points_rgb[:, :3])

    # Add colors if available
    if points_rgb.shape[1] >= 6:
        colors = points_rgb[:, 3:6] / 255.0  # Convert from [0,255] to [0,1]
        colors = np.clip(colors, 0.0, 1.0)  # Ensure valid range
        pcd.colors = o3d.utility.Vector3dVector(colors)

    return pcd


def create_masked_point_cloud(color_img, depth_img, mask, intrinsic, depth_scale=1.0):
    """
    Create a point cloud for a masked region of RGBD data using Open3D.

    Args:
        color_img: RGB image (H, W, 3)
        depth_img: Depth image (H, W)
        mask: Boolean mask of the same size as color_img and depth_img
        intrinsic: Camera intrinsic matrix (3x3 numpy array)
        depth_scale: Scale factor to convert depth to meters

    Returns:
        Open3D point cloud object for the masked region
    """
    # Filter out inf and nan values from depth image
    depth_filtered = depth_img.copy()

    # Create mask for valid depth values (finite, positive, non-zero)
    valid_depth_mask = np.isfinite(depth_filtered) & (depth_filtered > 0)

    # Set invalid values to 0
    depth_filtered[~valid_depth_mask] = 0.0

    # Create masked color and depth images
    masked_color = color_img.copy()
    masked_depth = depth_filtered.copy()

    # Apply mask
    if not mask.shape[:2] == color_img.shape[:2]:
        raise ValueError(f"Mask shape {mask.shape} doesn't match image shape {color_img.shape[:2]}")

    # Create a boolean mask that is properly expanded for the RGB channels
    # For RGB image, we need to properly broadcast the mask to all 3 channels
    if len(color_img.shape) == 3 and color_img.shape[2] == 3:
        # Properly broadcast mask to match the RGB dimensions
        mask_rgb = np.broadcast_to(mask[:, :, np.newaxis], color_img.shape)
        masked_color[~mask_rgb] = 0
    else:
        # For grayscale images
        masked_color[~mask] = 0

    # Apply mask to depth image
    masked_depth[~mask] = 0

    # Create point cloud
    pcd = create_o3d_point_cloud_from_rgbd(masked_color, masked_depth, intrinsic, depth_scale)

    # Remove points with coordinates at origin (0,0,0) which are likely from masked out regions
    points = np.asarray(pcd.points)
    if len(points) > 0:
        # Find points that are not at origin
        dist_from_origin = np.sum(points**2, axis=1)
        valid_indices = dist_from_origin > 1e-6

        # Filter points and colors
        pcd = pcd.select_by_index(np.where(valid_indices)[0])

    return pcd


def create_point_cloud_and_extract_masks(
    color_img: np.ndarray,
    depth_img: np.ndarray,
    masks: List[np.ndarray],
    intrinsic: np.ndarray,
    depth_scale: float = 1.0,
    depth_trunc: float = 3.0,
) -> Tuple[o3d.geometry.PointCloud, List[o3d.geometry.PointCloud]]:
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


def extract_masked_point_cloud_efficient(
    full_pcd: o3d.geometry.PointCloud, depth_img: np.ndarray, mask: np.ndarray
) -> o3d.geometry.PointCloud:
    """
    Extract a masked region from an existing point cloud efficiently.

    This assumes the point cloud was created from the given depth image.
    Use this when you have a pre-computed full point cloud and want to extract
    individual masked regions.

    Args:
        full_pcd: Complete Open3D point cloud
        depth_img: Depth image used to create the point cloud (H, W)
        mask: Boolean mask (H, W)

    Returns:
        Open3D point cloud for the masked region

    Raises:
        ValueError: If mask shape doesn't match depth image
    """
    if mask.shape != depth_img.shape:
        raise ValueError(
            f"Mask shape {mask.shape} doesn't match depth image shape {depth_img.shape}"
        )

    # Early return if no points in full point cloud
    if len(np.asarray(full_pcd.points)) == 0:
        return o3d.geometry.PointCloud()

    # Get valid depth mask
    valid_depth = depth_img.flatten() > 0
    mask_flat = mask.flatten()

    # Find pixels that are both valid and in the mask
    valid_mask_indices = mask_flat & valid_depth

    # Get indices of valid points
    point_indices = np.where(valid_mask_indices[valid_depth])[0]

    # Extract the masked point cloud
    if len(point_indices) > 0:
        return full_pcd.select_by_index(point_indices)
    else:
        return o3d.geometry.PointCloud()


def segment_and_remove_plane(pcd, distance_threshold=0.02, ransac_n=3, num_iterations=1000):
    """
    Segment the dominant plane from a point cloud using RANSAC and remove it.
    Often used to remove table tops, floors, walls, or other planar surfaces.

    Args:
        pcd: Open3D point cloud object
        distance_threshold: Maximum distance a point can be from the plane to be considered an inlier (in meters)
        ransac_n: Number of points to sample for each RANSAC iteration
        num_iterations: Number of RANSAC iterations

    Returns:
        Open3D point cloud with the dominant plane removed
    """
    # Make a copy of the input point cloud to avoid modifying the original
    pcd_filtered = o3d.geometry.PointCloud()
    pcd_filtered.points = o3d.utility.Vector3dVector(np.asarray(pcd.points))
    if pcd.has_colors():
        pcd_filtered.colors = o3d.utility.Vector3dVector(np.asarray(pcd.colors))
    if pcd.has_normals():
        pcd_filtered.normals = o3d.utility.Vector3dVector(np.asarray(pcd.normals))

    # Check if point cloud has enough points
    if len(pcd_filtered.points) < ransac_n:
        return pcd_filtered

    # Run RANSAC to find the largest plane
    _, inliers = pcd_filtered.segment_plane(
        distance_threshold=distance_threshold, ransac_n=ransac_n, num_iterations=num_iterations
    )

    # Remove the dominant plane (regardless of orientation)
    pcd_without_dominant_plane = pcd_filtered.select_by_index(inliers, invert=True)
    return pcd_without_dominant_plane


def filter_point_cloud_statistical(
    pcd: o3d.geometry.PointCloud, nb_neighbors: int = 20, std_ratio: float = 2.0
) -> Tuple[o3d.geometry.PointCloud, np.ndarray]:
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

    return pcd.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)


def filter_point_cloud_radius(
    pcd: o3d.geometry.PointCloud, nb_points: int = 16, radius: float = 0.05
) -> Tuple[o3d.geometry.PointCloud, np.ndarray]:
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

    return pcd.remove_radius_outlier(nb_points=nb_points, radius=radius)


def compute_point_cloud_bounds(pcd: o3d.geometry.PointCloud) -> dict:
    """
    Compute bounding box information for a point cloud.

    Args:
        pcd: Input point cloud

    Returns:
        Dictionary with bounds information
    """
    points = np.asarray(pcd.points)
    if len(points) == 0:
        return {
            "min": np.array([0, 0, 0]),
            "max": np.array([0, 0, 0]),
            "center": np.array([0, 0, 0]),
            "size": np.array([0, 0, 0]),
            "volume": 0.0,
        }

    min_bound = points.min(axis=0)
    max_bound = points.max(axis=0)
    center = (min_bound + max_bound) / 2
    size = max_bound - min_bound
    volume = np.prod(size)

    return {"min": min_bound, "max": max_bound, "center": center, "size": size, "volume": volume}


def project_3d_points_to_2d(points_3d: np.ndarray, camera_matrix: np.ndarray) -> np.ndarray:
    """
    Project 3D points to 2D image coordinates using camera intrinsics.

    Args:
        points_3d: Nx3 array of 3D points (X, Y, Z)
        camera_matrix: 3x3 camera intrinsic matrix

    Returns:
        Nx2 array of 2D image coordinates (u, v)
    """
    if len(points_3d) == 0:
        return np.zeros((0, 2), dtype=np.int32)

    # Filter out points with zero or negative depth
    valid_mask = points_3d[:, 2] > 0
    if not np.any(valid_mask):
        return np.zeros((0, 2), dtype=np.int32)

    valid_points = points_3d[valid_mask]

    # Extract camera parameters
    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]
    cx = camera_matrix[0, 2]
    cy = camera_matrix[1, 2]

    # Project to image coordinates
    u = (valid_points[:, 0] * fx / valid_points[:, 2]) + cx
    v = (valid_points[:, 1] * fy / valid_points[:, 2]) + cy

    # Round to integer pixel coordinates
    points_2d = np.column_stack([u, v]).astype(np.int32)

    return points_2d


def overlay_point_clouds_on_image(
    base_image: np.ndarray,
    point_clouds: List[o3d.geometry.PointCloud],
    camera_matrix: np.ndarray,
    colors: List[Tuple[int, int, int]],
    point_size: int = 2,
    alpha: float = 0.7,
) -> np.ndarray:
    """
    Overlay multiple colored point clouds onto an image.

    Args:
        base_image: Base image to overlay onto (H, W, 3) - assumed to be RGB
        point_clouds: List of Open3D point cloud objects
        camera_matrix: 3x3 camera intrinsic matrix
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
        points_2d = project_3d_points_to_2d(points_3d, camera_matrix)

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
        if isinstance(color, (list, tuple, np.ndarray)):
            color = tuple(int(c) for c in color[:3])
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


def create_3d_bounding_box_corners(position, rotation, size):
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
    if hasattr(position, 'x'):  # Vector object
        center = np.array([position.x, position.y, position.z])
    else:  # Dictionary
        center = np.array([position['x'], position['y'], position['z']])
    
    # Convert rotation (euler angles) to rotation matrix
    if hasattr(rotation, 'x'):  # Vector object (roll, pitch, yaw)
        roll, pitch, yaw = rotation.x, rotation.y, rotation.z
    else:  # Dictionary
        roll, pitch, yaw = rotation['roll'], rotation['pitch'], rotation['yaw']
    
    # Create rotation matrix from euler angles (ZYX order)
    cos_r, sin_r = np.cos(roll), np.sin(roll)
    cos_p, sin_p = np.cos(pitch), np.sin(pitch)
    cos_y, sin_y = np.cos(yaw), np.sin(yaw)
    
    # Rotation matrix for ZYX euler angles
    R = np.array([
        [cos_y * cos_p, cos_y * sin_p * sin_r - sin_y * cos_r, cos_y * sin_p * cos_r + sin_y * sin_r],
        [sin_y * cos_p, sin_y * sin_p * sin_r + cos_y * cos_r, sin_y * sin_p * cos_r - cos_y * sin_r],
        [-sin_p, cos_p * sin_r, cos_p * cos_r]
    ])
    
    # Get dimensions
    width = size.get('width', 0.1)
    height = size.get('height', 0.1) 
    depth = size.get('depth', 0.1)
    
    # Create 8 corners of the bounding box (before rotation)
    corners = np.array([
        [-width/2, -height/2, -depth/2],  # 0
        [ width/2, -height/2, -depth/2],  # 1
        [ width/2,  height/2, -depth/2],  # 2
        [-width/2,  height/2, -depth/2],  # 3
        [-width/2, -height/2,  depth/2],  # 4
        [ width/2, -height/2,  depth/2],  # 5
        [ width/2,  height/2,  depth/2],  # 6
        [-width/2,  height/2,  depth/2],  # 7
    ])
    
    # Apply rotation and translation
    rotated_corners = corners @ R.T + center
    
    return rotated_corners


def draw_3d_bounding_box_on_image(image, corners_2d, color, thickness=2):
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
        (0, 1), (1, 2), (2, 3), (3, 0),  # Bottom face
        (4, 5), (5, 6), (6, 7), (7, 4),  # Top face
        (0, 4), (1, 5), (2, 6), (3, 7),  # Vertical edges
    ]
    
    # Draw each edge
    for start_idx, end_idx in edges:
        start_point = tuple(corners_2d[start_idx].astype(int))
        end_point = tuple(corners_2d[end_idx].astype(int))
        cv2.line(image, start_point, end_point, color, thickness)


def create_point_cloud_overlay_visualization(
    base_image: np.ndarray,
    filtered_objects: List[dict],
    camera_matrix: np.ndarray,
) -> np.ndarray:
    """
    Create a visualization showing object point clouds and bounding boxes overlaid on a base image.
    
    Args:
        base_image: Base image to overlay onto (H, W, 3)
        filtered_objects: List of object dictionaries containing 'point_cloud', 'color', 'position', 'rotation', 'size' keys
        camera_matrix: 3x3 camera intrinsic matrix or list [fx, fy, cx, cy]

    Returns:
        Visualization image with overlaid point clouds and bounding boxes (H, W, 3)
    """
    # Convert camera matrix if needed
    if isinstance(camera_matrix, list) and len(camera_matrix) == 4:
        fx, fy, cx, cy = camera_matrix
        camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float32)

    # Extract point clouds and colors from filtered objects
    point_clouds = []
    colors = []
    for obj in filtered_objects:
        if "point_cloud" in obj and obj["point_cloud"] is not None:
            point_clouds.append(obj["point_cloud"])

            # Convert color to tuple of integers for OpenCV
            color = obj["color"]
            if isinstance(color, np.ndarray):
                color = tuple(int(c) for c in color)
            elif isinstance(color, (list, tuple)):
                color = tuple(int(c) for c in color[:3])
            colors.append(color)

    if not point_clouds:
        return base_image
    
    # Create overlay visualization with point clouds
    result = overlay_point_clouds_on_image(
        base_image=base_image,
        point_clouds=point_clouds,
        camera_matrix=camera_matrix,
        colors=colors,
        point_size=3,
        alpha=0.8,
    )
    
    # Draw 3D bounding boxes
    height, width = result.shape[:2]
    for i, obj in enumerate(filtered_objects):
        if ("position" in obj and "rotation" in obj and "size" in obj and 
            obj["position"] is not None and obj["rotation"] is not None and obj["size"] is not None):
            
            try:
                # Create 3D bounding box corners
                corners_3d = create_3d_bounding_box_corners(obj["position"], obj["rotation"], obj["size"])
                
                # Project to 2D
                corners_2d = project_3d_points_to_2d(corners_3d, camera_matrix)
                
                # Filter corners within image bounds
                valid_mask = (
                    (corners_2d[:, 0] >= 0) & (corners_2d[:, 0] < width) &
                    (corners_2d[:, 1] >= 0) & (corners_2d[:, 1] < height)
                )
                
                # Only draw if at least some corners are visible
                if np.any(valid_mask):
                    # Get color for this object
                    if i < len(colors):
                        bbox_color = colors[i]
                    else:
                        bbox_color = (255, 255, 255)  # Default white
                    
                    # Draw the bounding box
                    draw_3d_bounding_box_on_image(result, corners_2d, bbox_color, thickness=2)
                    
            except Exception:
                # Skip bounding box drawing if there's an error
                continue
    
    return result
