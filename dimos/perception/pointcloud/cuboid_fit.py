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


import cv2
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


def fit_cuboid_simple(points: np.ndarray | o3d.geometry.PointCloud) -> dict | None:  # type: ignore[type-arg]
    """
    Simple wrapper for minimal oriented bounding box fitting.

    Args:
        points: Nx3 array of points or Open3D PointCloud

    Returns:
        Dictionary with center, dimensions, rotation, and bounding_box,
        or None if insufficient points
    """
    return fit_cuboid(points, method="minimal")


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


def get_cuboid_corners(
    center: np.ndarray,  # type: ignore[type-arg]
    dimensions: np.ndarray,  # type: ignore[type-arg]
    rotation: np.ndarray,  # type: ignore[type-arg]
) -> np.ndarray:  # type: ignore[type-arg]
    """
    Get the 8 corners of a cuboid.

    Args:
        center: 3D center point
        dimensions: 3D dimensions
        rotation: 3x3 rotation matrix

    Returns:
        8x3 array of corner coordinates
    """
    half_dims = dimensions / 2
    corners_local = (
        np.array(
            [
                [-1, -1, -1],  # 0: left  bottom back
                [-1, -1, 1],  # 1: left  bottom front
                [-1, 1, -1],  # 2: left  top    back
                [-1, 1, 1],  # 3: left  top    front
                [1, -1, -1],  # 4: right bottom back
                [1, -1, 1],  # 5: right bottom front
                [1, 1, -1],  # 6: right top    back
                [1, 1, 1],  # 7: right top    front
            ]
        )
        * half_dims
    )

    # Apply rotation and translation
    return corners_local @ rotation.T + center  # type: ignore[no-any-return]


def visualize_cuboid_on_image(
    image: np.ndarray,  # type: ignore[type-arg]
    cuboid_params: dict,  # type: ignore[type-arg]
    camera_matrix: np.ndarray,  # type: ignore[type-arg]
    extrinsic_rotation: np.ndarray | None = None,  # type: ignore[type-arg]
    extrinsic_translation: np.ndarray | None = None,  # type: ignore[type-arg]
    color: tuple[int, int, int] = (0, 255, 0),
    thickness: int = 2,
    show_dimensions: bool = True,
) -> np.ndarray:  # type: ignore[type-arg]
    """
    Draw a fitted cuboid on an image using camera projection.

    Args:
        image: Input image to draw on
        cuboid_params: Dictionary containing cuboid parameters
        camera_matrix: Camera intrinsic matrix (3x3)
        extrinsic_rotation: Optional external rotation (3x3)
        extrinsic_translation: Optional external translation (3x1)
        color: Line color as (B, G, R) tuple
        thickness: Line thickness
        show_dimensions: Whether to display dimension text

    Returns:
        Image with cuboid visualization

    Raises:
        ValueError: If required parameters are missing or invalid
    """
    # Validate inputs
    required_keys = ["center", "dimensions", "rotation"]
    if not all(key in cuboid_params for key in required_keys):
        raise ValueError(f"cuboid_params must contain keys: {required_keys}")

    if camera_matrix.shape != (3, 3):
        raise ValueError(f"camera_matrix must be 3x3, got {camera_matrix.shape}")

    # Get corners in world coordinates
    corners = get_cuboid_corners(
        cuboid_params["center"], cuboid_params["dimensions"], cuboid_params["rotation"]
    )

    # Transform corners if extrinsic parameters are provided
    if extrinsic_rotation is not None and extrinsic_translation is not None:
        if extrinsic_rotation.shape != (3, 3):
            raise ValueError(f"extrinsic_rotation must be 3x3, got {extrinsic_rotation.shape}")
        if extrinsic_translation.shape not in [(3,), (3, 1)]:
            raise ValueError(
                f"extrinsic_translation must be (3,) or (3,1), got {extrinsic_translation.shape}"
            )

        extrinsic_translation = extrinsic_translation.flatten()
        corners = (extrinsic_rotation @ corners.T).T + extrinsic_translation

    try:
        # Project 3D corners to image coordinates
        corners_img, _ = cv2.projectPoints(  # type: ignore[call-overload]
            corners.astype(np.float32),
            np.zeros(3),
            np.zeros(3),  # No additional rotation/translation
            camera_matrix.astype(np.float32),
            None,  # No distortion
        )
        corners_img = corners_img.reshape(-1, 2).astype(int)

        # Check if corners are within image bounds
        h, w = image.shape[:2]
        valid_corners = (
            (corners_img[:, 0] >= 0)
            & (corners_img[:, 0] < w)
            & (corners_img[:, 1] >= 0)
            & (corners_img[:, 1] < h)
        )

        if not np.any(valid_corners):
            print("Warning: All cuboid corners are outside image bounds")
            return image.copy()

    except Exception as e:
        print(f"Warning: Failed to project cuboid corners: {e}")
        return image.copy()

    # Define edges for wireframe visualization
    edges = [
        # Bottom face
        (0, 1),
        (1, 5),
        (5, 4),
        (4, 0),
        # Top face
        (2, 3),
        (3, 7),
        (7, 6),
        (6, 2),
        # Vertical edges
        (0, 2),
        (1, 3),
        (5, 7),
        (4, 6),
    ]

    # Draw edges
    vis_img = image.copy()
    for i, j in edges:
        # Only draw edge if both corners are valid
        if valid_corners[i] and valid_corners[j]:
            cv2.line(vis_img, tuple(corners_img[i]), tuple(corners_img[j]), color, thickness)

    # Add dimension text if requested
    if show_dimensions and np.any(valid_corners):
        dims = cuboid_params["dimensions"]
        dim_text = f"Dims: {dims[0]:.3f} x {dims[1]:.3f} x {dims[2]:.3f}"

        # Find a good position for text (top-left of image)
        text_pos = (10, 30)
        font_scale = 0.7

        # Add background rectangle for better readability
        text_size = cv2.getTextSize(dim_text, cv2.FONT_HERSHEY_SIMPLEX, font_scale, 2)[0]
        cv2.rectangle(
            vis_img,
            (text_pos[0] - 5, text_pos[1] - text_size[1] - 5),
            (text_pos[0] + text_size[0] + 5, text_pos[1] + 5),
            (0, 0, 0),
            -1,
        )

        cv2.putText(vis_img, dim_text, text_pos, cv2.FONT_HERSHEY_SIMPLEX, font_scale, color, 2)

    return vis_img


def compute_cuboid_volume(cuboid_params: dict) -> float:  # type: ignore[type-arg]
    """
    Compute the volume of a cuboid.

    Args:
        cuboid_params: Dictionary containing cuboid parameters

    Returns:
        Volume in cubic units
    """
    if "dimensions" not in cuboid_params:
        raise ValueError("cuboid_params must contain 'dimensions' key")

    dims = cuboid_params["dimensions"]
    return float(np.prod(dims))


def compute_cuboid_surface_area(cuboid_params: dict) -> float:  # type: ignore[type-arg]
    """
    Compute the surface area of a cuboid.

    Args:
        cuboid_params: Dictionary containing cuboid parameters

    Returns:
        Surface area in square units
    """
    if "dimensions" not in cuboid_params:
        raise ValueError("cuboid_params must contain 'dimensions' key")

    dims = cuboid_params["dimensions"]
    return 2.0 * (dims[0] * dims[1] + dims[1] * dims[2] + dims[2] * dims[0])  # type: ignore[no-any-return]


def check_cuboid_quality(cuboid_params: dict, points: np.ndarray) -> dict:  # type: ignore[type-arg]
    """
    Assess the quality of a cuboid fit.

    Args:
        cuboid_params: Dictionary containing cuboid parameters
        points: Original points used for fitting

    Returns:
        Dictionary with quality metrics
    """
    if len(points) == 0:
        return {"error": "No points provided"}

    # Basic metrics
    volume = compute_cuboid_volume(cuboid_params)
    surface_area = compute_cuboid_surface_area(cuboid_params)
    error = cuboid_params.get("error", 0.0)

    # Aspect ratio analysis
    dims = cuboid_params["dimensions"]
    aspect_ratios = [
        dims[0] / dims[1] if dims[1] > 0 else float("inf"),
        dims[1] / dims[2] if dims[2] > 0 else float("inf"),
        dims[2] / dims[0] if dims[0] > 0 else float("inf"),
    ]
    max_aspect_ratio = max(aspect_ratios)

    # Volume ratio (cuboid volume vs convex hull volume)
    try:
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        hull, _ = pcd.compute_convex_hull()
        hull_volume = hull.get_volume()
        volume_ratio = volume / hull_volume if hull_volume > 0 else float("inf")
    except:
        volume_ratio = None

    return {
        "fitting_error": error,
        "volume": volume,
        "surface_area": surface_area,
        "max_aspect_ratio": max_aspect_ratio,
        "volume_ratio": volume_ratio,
        "num_points": len(points),
        "method": cuboid_params.get("method", "unknown"),
    }


# Backward compatibility
def visualize_fit(image, cuboid_params, camera_matrix, R=None, t=None):  # type: ignore[no-untyped-def]
    """
    Legacy function for backward compatibility.
    Use visualize_cuboid_on_image instead.
    """
    return visualize_cuboid_on_image(
        image, cuboid_params, camera_matrix, R, t, show_dimensions=True
    )
