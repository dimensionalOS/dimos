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

"""Utilities for grasp generation and visualization."""

import numpy as np
import open3d as o3d
import cv2
from typing import List, Dict, Tuple, Optional, Union


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


def euler_to_rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """
    Convert Euler angles to rotation matrix.

    Args:
        roll: Roll angle in radians
        pitch: Pitch angle in radians
        yaw: Yaw angle in radians

    Returns:
        3x3 rotation matrix
    """
    Rx = np.array([[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]])

    Ry = np.array(
        [[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]]
    )

    Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])

    # Combined rotation matrix
    R = Rz @ Ry @ Rx

    return R


def create_gripper_geometry(
    grasp_data: dict,
    finger_length: float = 0.08,
    finger_thickness: float = 0.004,
    base_height: float = 0.04,
    color=[0, 0.5, 1],
) -> List[o3d.geometry.TriangleMesh]:
    """
    Create a simple fork-like gripper geometry from grasp data.

    Args:
        grasp_data: Dictionary containing grasp parameters
            - translation: 3D position list
            - rotation_matrix: 3x3 rotation matrix defining gripper coordinate system
                * X-axis: gripper width direction (opening/closing)
                * Y-axis: finger length direction
                * Z-axis: approach direction (toward object)
            - width: Gripper opening width
        finger_length: Length of gripper fingers (longer)
        finger_thickness: Thickness of gripper fingers
        base_height: Height of gripper base (longer)
        color: RGB color for the gripper (solid blue)

    Returns:
        List of Open3D TriangleMesh geometries for the gripper
    """

    translation = np.array(grasp_data["translation"])
    rotation_matrix = np.array(grasp_data["rotation_matrix"])

    width = grasp_data.get("width", 0.04)

    # Create transformation matrix
    transform = np.eye(4)
    transform[:3, :3] = rotation_matrix
    transform[:3, 3] = translation

    geometries = []

    # Gripper dimensions
    finger_width = 0.006  # Thickness of each finger
    base_thickness = finger_thickness  # Make base same thickness as fingers (flat)
    handle_length = 0.05  # Length of handle extending backward

    # Build gripper in local coordinate system:
    # X-axis = width direction (left/right finger separation)
    # Y-axis = finger length direction (fingers extend along +Y)
    # Z-axis = approach direction (toward object, handle extends along -Z)
    # IMPORTANT: Fingertips should be at origin (translation point)

    # Create left finger extending along +Y, positioned at +X
    left_finger = o3d.geometry.TriangleMesh.create_box(
        width=finger_width,  # Thin finger
        height=finger_length,  # Extends along Y (finger length direction)
        depth=finger_thickness,  # Thin in Z direction
    )
    left_finger.translate(
        [
            width / 2 - finger_width / 2,  # Position at +X (half width from center)
            -finger_length,  # Shift so fingertips are at origin
            -finger_thickness / 2,  # Center in Z
        ]
    )

    # Create right finger extending along +Y, positioned at -X
    right_finger = o3d.geometry.TriangleMesh.create_box(
        width=finger_width,  # Thin finger
        height=finger_length,  # Extends along Y (finger length direction)
        depth=finger_thickness,  # Thin in Z direction
    )
    right_finger.translate(
        [
            -width / 2 - finger_width / 2,  # Position at -X (half width from center)
            -finger_length,  # Shift so fingertips are at origin
            -finger_thickness / 2,  # Center in Z
        ]
    )

    # Create base connecting fingers - flat like a stickman body
    base = o3d.geometry.TriangleMesh.create_box(
        width=width + finger_width,  # Full width plus finger thickness
        height=finger_thickness,  # Flat like fingers (stickman style)
        depth=finger_thickness,  # Thin like fingers
    )
    base.translate(
        [
            -width / 2 - finger_width / 2,  # Start from left finger position
            -finger_length - finger_thickness,  # Behind fingers, adjusted for fingertips at origin
            -finger_thickness / 2,  # Center in Z
        ]
    )

    # Create handle extending backward - flat stick like stickman arm
    handle = o3d.geometry.TriangleMesh.create_box(
        width=finger_width,  # Same width as fingers
        height=handle_length,  # Extends backward along Y direction (same plane)
        depth=finger_thickness,  # Thin like fingers (same plane)
    )
    handle.translate(
        [
            -finger_width / 2,  # Center in X
            -finger_length
            - finger_thickness
            - handle_length,  # Extend backward from base, adjusted for fingertips at origin
            -finger_thickness / 2,  # Same Z plane as other components
        ]
    )

    # Use solid red color for all parts (user changed to red)
    solid_color = [1.0, 0.0, 0.0]  # Red color

    left_finger.paint_uniform_color(solid_color)
    right_finger.paint_uniform_color(solid_color)
    base.paint_uniform_color(solid_color)
    handle.paint_uniform_color(solid_color)

    # Apply transformation to all parts
    left_finger.transform(transform)
    right_finger.transform(transform)
    base.transform(transform)
    handle.transform(transform)

    geometries.extend([left_finger, right_finger, base, handle])

    return geometries


def create_all_gripper_geometries(
    grasp_list: List[dict], max_grasps: int = -1
) -> List[o3d.geometry.TriangleMesh]:
    """
    Create gripper geometries for multiple grasps.

    Args:
        grasp_list: List of grasp dictionaries
        max_grasps: Maximum number of grasps to visualize (-1 for all)

    Returns:
        List of all gripper geometries
    """
    all_geometries = []

    grasps_to_show = grasp_list if max_grasps < 0 else grasp_list[:max_grasps]

    for grasp in grasps_to_show:
        gripper_parts = create_gripper_geometry(grasp)
        all_geometries.extend(gripper_parts)

    return all_geometries


def draw_grasps_on_image(
    image: np.ndarray,
    grasp_data: Union[dict, Dict[Union[int, str], List[dict]], List[dict]],
    camera_matrix: np.ndarray,
    max_grasps: int = -1,  # -1 means show all grasps
    finger_length: float = 0.04,
    finger_thickness: float = 0.004,
) -> np.ndarray:
    """
    Draw fork-like gripper visualizations on the image.

    Args:
        image: Base image to draw on
        grasp_data: Can be:
            - A single grasp dict
            - A list of grasp dicts
            - A dictionary mapping object IDs or "scene" to list of grasps
        camera_matrix: 3x3 camera intrinsic matrix
        objects: List of detected objects (optional, for compatibility)
        max_grasps: Maximum number of grasps to show (-1 for all)
        finger_length: Length of gripper fingers
        finger_thickness: Thickness of gripper fingers
        base_height: Height of gripper base

    Returns:
        Image with grasps drawn
    """
    result = image.copy()

    # Convert input to standard format
    if isinstance(grasp_data, dict) and not any(
        key in grasp_data for key in ["scene", 0, 1, 2, 3, 4, 5]
    ):
        # Single grasp
        grasps_to_draw = [(grasp_data, 0)]
    elif isinstance(grasp_data, list):
        # List of grasps
        grasps_to_draw = [(g, i) for i, g in enumerate(grasp_data)]
    else:
        # Dictionary of grasps by object ID
        grasps_to_draw = []
        for obj_id, grasps in grasp_data.items():
            for i, grasp in enumerate(grasps):
                grasps_to_draw.append((grasp, i))

    # Limit number of grasps if specified
    if max_grasps > 0:
        grasps_to_draw = grasps_to_draw[:max_grasps]

    # Define grasp colors (gradient from bright to dark green)
    def get_grasp_color(index: int) -> tuple:
        # Create a gradient of green colors
        max_green = 255
        min_green = 100
        if index < 10:
            green_value = int(max_green - (max_green - min_green) * index / 10)
        else:
            green_value = min_green
        return (0, green_value, 0)

    # Draw each grasp
    for grasp, index in grasps_to_draw:
        try:
            color = get_grasp_color(index)
            thickness = max(1, 4 - index // 3)

            # Extract grasp parameters
            position = grasp["position"]
            rotation = grasp["rotation"]
            width = grasp.get("width", 0.08)
            depth = grasp.get("depth", 0.02)

            # Create gripper vertices in gripper frame
            half_w = width / 2
            half_d = depth / 2

            # Define key points of the fork gripper
            # Base rectangle
            base_corners = np.array(
                [
                    [-half_w, -half_d, 0],
                    [half_w, -half_d, 0],
                    [half_w, half_d, 0],
                    [-half_w, half_d, 0],
                ]
            )

            # Left finger outline
            left_finger = np.array(
                [
                    [-half_w + finger_thickness, half_d, 0],
                    [-half_w, half_d, 0],
                    [-half_w, half_d + finger_length, 0],
                    [-half_w + finger_thickness, half_d + finger_length, 0],
                ]
            )

            # Right finger outline
            right_finger = np.array(
                [
                    [half_w - finger_thickness, half_d, 0],
                    [half_w, half_d, 0],
                    [half_w, half_d + finger_length, 0],
                    [half_w - finger_thickness, half_d + finger_length, 0],
                ]
            )

            # Create rotation matrix from Euler angles
            roll = rotation.get("roll", 0)
            pitch = rotation.get("pitch", 0)
            yaw = rotation.get("yaw", 0)

            # Rotation matrices for each axis
            Rx = np.array(
                [[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]]
            )

            Ry = np.array(
                [[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]]
            )

            Rz = np.array(
                [[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]]
            )

            # Combined rotation matrix
            R = Rz @ Ry @ Rx

            # Transform to world frame
            pos_array = np.array([position["x"], position["y"], position["z"]])

            # Transform and project base
            base_world = (R @ base_corners.T).T + pos_array
            base_2d = project_3d_points_to_2d(base_world, camera_matrix)

            # Transform and project fingers
            left_world = (R @ left_finger.T).T + pos_array
            left_2d = project_3d_points_to_2d(left_world, camera_matrix)

            right_world = (R @ right_finger.T).T + pos_array
            right_2d = project_3d_points_to_2d(right_world, camera_matrix)

            # Draw base rectangle
            for i in range(4):
                p1 = tuple(base_2d[i].astype(int))
                p2 = tuple(base_2d[(i + 1) % 4].astype(int))
                cv2.line(result, p1, p2, color, thickness)

            # Draw left finger
            for i in range(4):
                p1 = tuple(left_2d[i].astype(int))
                p2 = tuple(left_2d[(i + 1) % 4].astype(int))
                cv2.line(result, p1, p2, color, thickness)

            # Draw right finger
            for i in range(4):
                p1 = tuple(right_2d[i].astype(int))
                p2 = tuple(right_2d[(i + 1) % 4].astype(int))
                cv2.line(result, p1, p2, color, thickness)

            # Draw grasp center
            center_2d = project_3d_points_to_2d(pos_array.reshape(1, -1), camera_matrix)[0]
            cv2.circle(result, tuple(center_2d.astype(int)), 3, color, -1)

        except Exception as e:
            # Skip this grasp if there's an error
            pass

    return result


def visualize_grasps_3d(
    point_cloud: o3d.geometry.PointCloud, grasp_list: List[dict], max_grasps: int = -1
):
    """
    Visualize grasps in 3D with point cloud.

    Args:
        point_cloud: Open3D point cloud
        grasp_list: List of grasp dictionaries
        max_grasps: Maximum number of grasps to visualize
    """
    geometries = [point_cloud]

    # Add gripper geometries
    gripper_geometries = create_all_gripper_geometries(grasp_list, max_grasps)
    geometries.extend(gripper_geometries)

    # Add a coordinate frame at origin for reference
    origin_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
    geometries.append(origin_frame)

    # Apply transformation for better view (flip z-axis)
    trans_mat = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    for geom in geometries:
        geom.transform(trans_mat)

    o3d.visualization.draw_geometries(geometries, window_name="3D Grasp Visualization")
