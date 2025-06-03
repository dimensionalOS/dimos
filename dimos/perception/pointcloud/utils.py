"""
Point cloud utilities for RGBD data processing.

This module provides efficient utilities for creating and manipulating point clouds
from RGBD images using Open3D.
"""

import numpy as np
import yaml
import os
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
    # Get focal length and principal point from camera matrix
    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]
    cx = camera_matrix[0, 2]
    cy = camera_matrix[1, 2]

    # Create pixel coordinate grid
    rows, cols = depth_image.shape
    x_grid, y_grid = np.meshgrid(
        np.arange(0, cols, subsample_factor), np.arange(0, rows, subsample_factor)
    )

    # Flatten grid and depth
    x = x_grid.flatten()
    y = y_grid.flatten()
    z = depth_image[y_grid, x_grid].flatten()

    # Remove points with invalid depth
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
    camera_info: Optional[Union[str, List[float], np.ndarray, dict]]
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
            with open(camera_info, 'r') as f:
                data = yaml.safe_load(f)
            return _extract_matrix_from_dict(data)
        except Exception as e:
            raise ValueError(f"Failed to read camera info from {camera_info}: {e}")
    
    raise ValueError(f"Invalid camera_info format. Expected str, list, dict, or numpy array, got {type(camera_info)}")

def _extract_matrix_from_dict(data: dict) -> np.ndarray:
    """Extract camera matrix from dictionary with various formats."""
    # ROS format with 'K' field (most common)
    if 'K' in data:
        k_data = data['K']
        if len(k_data) == 9:
            return np.array(k_data, dtype=np.float32).reshape(3, 3)
    
    # Standard format with 'camera_matrix'
    if 'camera_matrix' in data:
        if 'data' in data['camera_matrix']:
            matrix_data = data['camera_matrix']['data']
            if len(matrix_data) == 9:
                return np.array(matrix_data, dtype=np.float32).reshape(3, 3)
    
    # Explicit intrinsics format
    if all(k in data for k in ['fx', 'fy', 'cx', 'cy']):
        fx, fy = float(data['fx']), float(data['fy'])
        cx, cy = float(data['cx']), float(data['cy'])
        return np.array([
            [fx, 0, cx],
            [0, fy, cy],
            [0, 0, 1]
        ], dtype=np.float32)
    
    # Error case - provide helpful debug info
    available_keys = list(data.keys())
    if 'K' in data:
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
    depth_trunc: float = 3.0
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
        raise ValueError(f"Color and depth image dimensions don't match: {color_img.shape[:2]} vs {depth_img.shape}")
    if intrinsic.shape != (3, 3):
        raise ValueError(f"intrinsic must be (3, 3), got {intrinsic.shape}")
    
    # Convert to Open3D format
    color_o3d = o3d.geometry.Image(color_img.astype(np.uint8))
    depth_o3d = o3d.geometry.Image(depth_img.astype(np.float32))

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
        color_o3d, depth_o3d, 
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
        raise ValueError(f"points_rgb must have at least 3 columns (XYZ), got {points_rgb.shape[1]}")
    
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points_rgb[:, :3])
    
    # Add colors if available
    if points_rgb.shape[1] >= 6:
        colors = points_rgb[:, 3:6] / 255.0  # Convert from [0,255] to [0,1]
        colors = np.clip(colors, 0.0, 1.0)   # Ensure valid range
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
    # Create masked color and depth images
    masked_color = color_img.copy()
    masked_depth = depth_img.copy()

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
    depth_trunc: float = 3.0
) -> Tuple[o3d.geometry.PointCloud, List[o3d.geometry.PointCloud]]:
    """
    Efficiently create a point cloud once and extract multiple masked regions.
    
    This is much more efficient than creating individual point clouds for each mask,
    especially when dealing with many objects.
    
    Args:
        color_img: RGB image (H, W, 3)
        depth_img: Depth image (H, W)
        masks: List of boolean masks, each of shape (H, W)
        intrinsic: Camera intrinsic matrix (3x3 numpy array)
        depth_scale: Scale factor to convert depth to meters
        depth_trunc: Maximum depth in meters
    
    Returns:
        Tuple of (full_point_cloud, list_of_masked_point_clouds)
        
    Raises:
        ValueError: If inputs are invalid
    """
    # Validate inputs
    if not masks:
        raise ValueError("masks list cannot be empty")
    
    for i, mask in enumerate(masks):
        if mask.shape != depth_img.shape:
            raise ValueError(f"Mask {i} shape {mask.shape} doesn't match depth image shape {depth_img.shape}")
    
    # Create the full point cloud once
    full_pcd = create_o3d_point_cloud_from_rgbd(
        color_img, depth_img, intrinsic, depth_scale, depth_trunc
    )
    
    # Early return if no points generated
    if len(np.asarray(full_pcd.points)) == 0:
        empty_pcds = [o3d.geometry.PointCloud() for _ in masks]
        return full_pcd, empty_pcds
    
    # Create mapping from pixel coordinates to point indices
    height, width = depth_img.shape
    valid_depth = depth_img.flatten() > 0
    
    # Map from pixel index to point index (only for valid pixels)
    pixel_to_point = np.cumsum(valid_depth) - 1
    pixel_to_point[~valid_depth] = -1  # Mark invalid pixels
    
    # Extract point clouds for each mask
    masked_pcds = []
    for mask in masks:
        mask_flat = mask.flatten()
        
        # Find pixels that are both valid (have depth) and in the mask
        valid_mask_indices = mask_flat & valid_depth
        
        # Get the point indices for these pixels
        point_indices = pixel_to_point[valid_mask_indices]
        
        # Remove any -1 indices (shouldn't happen but safety check)
        point_indices = point_indices[point_indices >= 0]
        
        # Extract the masked point cloud
        if len(point_indices) > 0:
            masked_pcd = full_pcd.select_by_index(point_indices)
        else:
            masked_pcd = o3d.geometry.PointCloud()
        
        masked_pcds.append(masked_pcd)
    
    return full_pcd, masked_pcds

def extract_masked_point_cloud_efficient(
    full_pcd: o3d.geometry.PointCloud,
    depth_img: np.ndarray,
    mask: np.ndarray
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
        raise ValueError(f"Mask shape {mask.shape} doesn't match depth image shape {depth_img.shape}")
    
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
    pcd: o3d.geometry.PointCloud,
    nb_neighbors: int = 20,
    std_ratio: float = 2.0
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
    pcd: o3d.geometry.PointCloud,
    nb_points: int = 16,
    radius: float = 0.05
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
            "volume": 0.0
        }
    
    min_bound = points.min(axis=0)
    max_bound = points.max(axis=0)
    center = (min_bound + max_bound) / 2
    size = max_bound - min_bound
    volume = np.prod(size)
    
    return {
        "min": min_bound,
        "max": max_bound,
        "center": center,
        "size": size,
        "volume": volume
    }

