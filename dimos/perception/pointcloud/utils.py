import numpy as np
import yaml
import os
import open3d as o3d

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
    x_grid, y_grid = np.meshgrid(np.arange(0, cols, subsample_factor),
                                np.arange(0, rows, subsample_factor))
    
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

def load_camera_matrix_from_yaml(camera_info):
    """
    Load camera matrix from file or dict.
    
    Args:
        camera_info: Path to YAML file, or dict with camera parameters,
                     or directly a 3x3 numpy array, or a list [fx, fy, cx, cy]
    
    Returns:
        3x3 camera intrinsic matrix
    """
    if camera_info is None:
        return None
        
    # Handle case where camera_info is already a matrix
    if isinstance(camera_info, np.ndarray) and camera_info.shape == (3, 3):
        return camera_info
    
    # Handle case where camera_info is [fx, fy, cx, cy] format
    if isinstance(camera_info, list) and len(camera_info) == 4:
        fx, fy, cx, cy = camera_info
        return np.array([
            [fx, 0, cx],
            [0, fy, cy],
            [0, 0, 1]
        ], dtype=np.float32)
    
    # Handle case where camera_info is a dict
    if isinstance(camera_info, dict):
        # Format: {'camera_matrix': {'data': [fx, 0, cx, 0, fy, cy, 0, 0, 1]}}
        if 'camera_matrix' in camera_info:
            data = camera_info['camera_matrix']['data']
            return np.array(data).reshape(3, 3)
        # ROS format with 'K' field
        elif 'K' in camera_info:
            k_data = camera_info['K']
            return np.array(k_data).reshape(3, 3)
        # Format: {'fx': val, 'fy': val, 'cx': val, 'cy': val}
        elif all(k in camera_info for k in ['fx', 'fy', 'cx', 'cy']):
            fx, fy = camera_info['fx'], camera_info['fy']
            cx, cy = camera_info['cx'], camera_info['cy']
            return np.array([
                [fx, 0, cx],
                [0, fy, cy],
                [0, 0, 1]
            ])
        else:
            raise ValueError("Cannot extract camera matrix from provided dict")
    
    # Handle case where camera_info is a path to a YAML file
    if isinstance(camera_info, str) and os.path.isfile(camera_info):
        with open(camera_info, 'r') as f:
            data = yaml.safe_load(f)
        
        # Try different formats in order of likelihood
        
        # ROS format with 'K' field (most common in ROS camera_info)
        if 'K' in data:
            k_data = data['K']
            return np.array(k_data).reshape(3, 3)
        
        # Standard format with 'camera_matrix'
        elif 'camera_matrix' in data:
            if 'data' in data['camera_matrix']:
                matrix_data = data['camera_matrix']['data']
                return np.array(matrix_data).reshape(3, 3)
        
        # Explicit intrinsics
        elif all(k in data for k in ['fx', 'fy', 'cx', 'cy']):
            fx, fy = data['fx'], data['fy']
            cx, cy = data['cx'], data['cy']
            return np.array([
                [fx, 0, cx],
                [0, fy, cy],
                [0, 0, 1]
            ])
        
        # If we can't find a recognized format, print the keys to help debugging
        print(f"Cannot extract camera matrix from file {camera_info}")
        print(f"Available keys: {list(data.keys())}")
        
        if 'K' in data:
            print(f"K field contains: {data['K']}")
        
        raise ValueError(f"Unrecognized camera info format in {camera_info}")
    
    raise ValueError("Invalid camera_info format")

def create_o3d_point_cloud_from_rgbd(color_img, depth_img, intrinsic, depth_scale=1.0, depth_trunc=3.0):
    """
    Create an Open3D point cloud from RGB and depth images.
    
    Args:
        color_img: RGB image as numpy array (H, W, 3)
        depth_img: Depth image as numpy array (H, W)
        intrinsic: Camera intrinsic matrix (3x3 numpy array)
        depth_scale: Scale factor to convert depth to meters (default: 1000.0 for mm to m)
        depth_trunc: Maximum depth in meters
    
    Returns:
        Open3D point cloud object
    """
    # Convert to Open3D format
    color_o3d = o3d.geometry.Image(color_img)
    depth_o3d = o3d.geometry.Image(depth_img.astype(np.float32))
    
    # Create Open3D intrinsic object
    height, width = color_img.shape[:2]
    intrinsic_o3d = o3d.camera.PinholeCameraIntrinsic(
        width, height,
        intrinsic[0, 0], intrinsic[1, 1],  # fx, fy
        intrinsic[0, 2], intrinsic[1, 2]   # cx, cy
    )
    
    # Create RGBD image
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color_o3d, depth_o3d, 
        depth_scale=depth_scale,  # Depth already in meters
        depth_trunc=depth_trunc,
        convert_rgb_to_intensity=False
    )
    
    # Create point cloud
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
        rgbd, intrinsic_o3d
    )
    
    return pcd

def o3d_point_cloud_to_numpy(pcd):
    """
    Convert Open3D point cloud to numpy array of XYZRGB points.
    
    Args:
        pcd: Open3D point cloud object
    
    Returns:
        Nx6 array of XYZRGB points
    """
    # Get points and colors
    points = np.asarray(pcd.points)
    colors = np.asarray(pcd.colors)
    
    # Combine points and colors
    if len(points) > 0:
        # Convert colors from [0,1] to [0,255]
        colors_rgb = colors * 255
        return np.column_stack([points, colors_rgb])
    
    return np.zeros((0, 6))

def numpy_to_o3d_point_cloud(points_rgb):
    """
    Convert numpy array of XYZRGB points to Open3D point cloud.
    
    Args:
        points_rgb: Nx6 array of XYZRGB points
    
    Returns:
        Open3D point cloud object
    """
    if len(points_rgb) == 0:
        return o3d.geometry.PointCloud()
    
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points_rgb[:, :3])
    
    # Convert colors from [0,255] to [0,1]
    if points_rgb.shape[1] >= 6:
        colors = points_rgb[:, 3:6] / 255.0
        pcd.colors = o3d.utility.Vector3dVector(colors)
    
    return pcd

def rotation_to_o3d(rotation_matrix):
    """
    Convert a rotation matrix to proper Open3D format.
    
    Args:
        rotation_matrix: 3x3 rotation matrix
    
    Returns:
        3x3 rotation matrix compatible with Open3D conventions
    """
    # Ensure input is numpy array
    R = np.array(rotation_matrix, dtype=np.float64)
    
    # If rotation matrix comes from PCA, it might have eigenvectors as rows
    # Open3D expects eigenvectors as columns, so we might need to transpose
    # We can determine this by checking the determinant
    det = np.linalg.det(R)
    
    # If determinant is negative, rotation is improper (reflection included)
    if abs(det) < 0.9:
        # Not a valid rotation matrix, try transposing
        R = R.T
        det = np.linalg.det(R)
    
    # Ensure right-handed coordinate system (det > 0)
    if det < 0:
        # Flip the last column to make right-handed
        R[:, 2] = -R[:, 2]
    
    # Ensure it's a proper rotation matrix (orthogonal)
    # We use SVD to find the closest orthogonal matrix
    U, _, Vt = np.linalg.svd(R)
    R = U @ Vt
    
    return R

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

