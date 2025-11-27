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

import cv2
import numpy as np
import open3d as o3d  # type: ignore[import-untyped]

from dimos.perception.common.utils import project_3d_points_to_2d


def create_gripper_geometry(
    grasp_data: dict,  # type: ignore[type-arg]
    finger_length: float = 0.08,
    finger_thickness: float = 0.004,
) -> list[o3d.geometry.TriangleMesh]:
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
    grasp_list: list[dict],  # type: ignore[type-arg]
    max_grasps: int = -1,
) -> list[o3d.geometry.TriangleMesh]:
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
    image: np.ndarray,  # type: ignore[type-arg]
    grasp_data: dict | dict[int | str, list[dict]] | list[dict],  # type: ignore[type-arg]
    camera_intrinsics: list[float] | np.ndarray,  # type: ignore[type-arg]  # [fx, fy, cx, cy] or 3x3 matrix
    max_grasps: int = -1,  # -1 means show all grasps
    finger_length: float = 0.08,  # Match 3D gripper
    finger_thickness: float = 0.004,  # Match 3D gripper
) -> np.ndarray:  # type: ignore[type-arg]
    """
    Draw fork-like gripper visualizations on the image matching 3D gripper design.

    Args:
        image: Base image to draw on
        grasp_data: Can be:
            - A single grasp dict
            - A list of grasp dicts
            - A dictionary mapping object IDs or "scene" to list of grasps
        camera_intrinsics: Camera parameters as [fx, fy, cx, cy] list or 3x3 matrix
        max_grasps: Maximum number of grasps to visualize (-1 for all)
        finger_length: Length of gripper fingers (matches 3D design)
        finger_thickness: Thickness of gripper fingers (matches 3D design)

    Returns:
        Image with grasps drawn
    """
    result = image.copy()

    # Convert camera intrinsics to 3x3 matrix if needed
    if isinstance(camera_intrinsics, list) and len(camera_intrinsics) == 4:
        fx, fy, cx, cy = camera_intrinsics
        camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
    else:
        camera_matrix = np.array(camera_intrinsics)

    # Convert input to standard format
    if isinstance(grasp_data, dict) and not any(
        key in grasp_data for key in ["scene", 0, 1, 2, 3, 4, 5]
    ):
        # Single grasp
        grasps_to_draw = [(grasp_data, 0)]
    elif isinstance(grasp_data, list):
        # List of grasps
        grasps_to_draw = [(grasp, i) for i, grasp in enumerate(grasp_data)]
    else:
        # Dictionary of grasps by object ID
        grasps_to_draw = []
        for _obj_id, grasps in grasp_data.items():
            for i, grasp in enumerate(grasps):
                grasps_to_draw.append((grasp, i))

    # Limit number of grasps if specified
    if max_grasps > 0:
        grasps_to_draw = grasps_to_draw[:max_grasps]

    # Define grasp colors (solid red to match 3D design)
    def get_grasp_color(index: int) -> tuple:  # type: ignore[type-arg]
        # Use solid red color for all grasps to match 3D design
        return (0, 0, 255)  # Red in BGR format for OpenCV

    # Draw each grasp
    for grasp, index in grasps_to_draw:
        try:
            color = get_grasp_color(index)
            thickness = max(1, 4 - index // 3)

            # Extract grasp parameters (using translation and rotation_matrix)
            if "translation" not in grasp or "rotation_matrix" not in grasp:
                continue

            translation = np.array(grasp["translation"])
            rotation_matrix = np.array(grasp["rotation_matrix"])
            width = grasp.get("width", 0.04)

            # Match 3D gripper dimensions
            finger_width = 0.006  # Thickness of each finger (matches 3D)
            handle_length = 0.05  # Length of handle extending backward (matches 3D)

            # Create gripper geometry in local coordinate system matching 3D design:
            # X-axis = width direction (left/right finger separation)
            # Y-axis = finger length direction (fingers extend along +Y)
            # Z-axis = approach direction (toward object, handle extends along -Z)
            # IMPORTANT: Fingertips should be at origin (translation point)

            # Left finger extending along +Y, positioned at +X
            left_finger_points = np.array(
                [
                    [
                        width / 2 - finger_width / 2,  # type: ignore[operator]
                        -finger_length,
                        -finger_thickness / 2,
                    ],  # Back left
                    [
                        width / 2 + finger_width / 2,  # type: ignore[operator]
                        -finger_length,
                        -finger_thickness / 2,
                    ],  # Back right
                    [
                        width / 2 + finger_width / 2,  # type: ignore[operator]
                        0,
                        -finger_thickness / 2,
                    ],  # Front right (at origin)
                    [
                        width / 2 - finger_width / 2,  # type: ignore[operator]
                        0,
                        -finger_thickness / 2,
                    ],  # Front left (at origin)
                ]
            )

            # Right finger extending along +Y, positioned at -X
            right_finger_points = np.array(
                [
                    [
                        -width / 2 - finger_width / 2,  # type: ignore[operator]
                        -finger_length,
                        -finger_thickness / 2,
                    ],  # Back left
                    [
                        -width / 2 + finger_width / 2,  # type: ignore[operator]
                        -finger_length,
                        -finger_thickness / 2,
                    ],  # Back right
                    [
                        -width / 2 + finger_width / 2,  # type: ignore[operator]
                        0,
                        -finger_thickness / 2,
                    ],  # Front right (at origin)
                    [
                        -width / 2 - finger_width / 2,  # type: ignore[operator]
                        0,
                        -finger_thickness / 2,
                    ],  # Front left (at origin)
                ]
            )

            # Base connecting fingers - flat rectangle behind fingers
            base_points = np.array(
                [
                    [
                        -width / 2 - finger_width / 2,  # type: ignore[operator]
                        -finger_length - finger_thickness,
                        -finger_thickness / 2,
                    ],  # Back left
                    [
                        width / 2 + finger_width / 2,  # type: ignore[operator]
                        -finger_length - finger_thickness,
                        -finger_thickness / 2,
                    ],  # Back right
                    [
                        width / 2 + finger_width / 2,  # type: ignore[operator]
                        -finger_length,
                        -finger_thickness / 2,
                    ],  # Front right
                    [
                        -width / 2 - finger_width / 2,  # type: ignore[operator]
                        -finger_length,
                        -finger_thickness / 2,
                    ],  # Front left
                ]
            )

            # Handle extending backward - thin rectangle
            handle_points = np.array(
                [
                    [
                        -finger_width / 2,
                        -finger_length - finger_thickness - handle_length,
                        -finger_thickness / 2,
                    ],  # Back left
                    [
                        finger_width / 2,
                        -finger_length - finger_thickness - handle_length,
                        -finger_thickness / 2,
                    ],  # Back right
                    [
                        finger_width / 2,
                        -finger_length - finger_thickness,
                        -finger_thickness / 2,
                    ],  # Front right
                    [
                        -finger_width / 2,
                        -finger_length - finger_thickness,
                        -finger_thickness / 2,
                    ],  # Front left
                ]
            )

            # Transform all points to world frame
            def transform_points(points):  # type: ignore[no-untyped-def]
                # Apply rotation and translation
                world_points = (rotation_matrix @ points.T).T + translation
                return world_points

            left_finger_world = transform_points(left_finger_points)  # type: ignore[no-untyped-call]
            right_finger_world = transform_points(right_finger_points)  # type: ignore[no-untyped-call]
            base_world = transform_points(base_points)  # type: ignore[no-untyped-call]
            handle_world = transform_points(handle_points)  # type: ignore[no-untyped-call]

            # Project to 2D
            left_finger_2d = project_3d_points_to_2d(left_finger_world, camera_matrix)
            right_finger_2d = project_3d_points_to_2d(right_finger_world, camera_matrix)
            base_2d = project_3d_points_to_2d(base_world, camera_matrix)
            handle_2d = project_3d_points_to_2d(handle_world, camera_matrix)

            # Draw left finger
            pts = left_finger_2d.astype(np.int32)
            cv2.polylines(result, [pts], True, color, thickness)

            # Draw right finger
            pts = right_finger_2d.astype(np.int32)
            cv2.polylines(result, [pts], True, color, thickness)

            # Draw base
            pts = base_2d.astype(np.int32)
            cv2.polylines(result, [pts], True, color, thickness)

            # Draw handle
            pts = handle_2d.astype(np.int32)
            cv2.polylines(result, [pts], True, color, thickness)

            # Draw grasp center (fingertips at origin)
            center_2d = project_3d_points_to_2d(translation.reshape(1, -1), camera_matrix)[0]
            cv2.circle(result, tuple(center_2d.astype(int)), 3, color, -1)

        except Exception:
            # Skip this grasp if there's an error
            continue

    return result


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


def visualize_grasps_3d(
    point_cloud: o3d.geometry.PointCloud,
    grasp_list: list[dict],  # type: ignore[type-arg]
    max_grasps: int = -1,
) -> None:
    """
    Visualize grasps in 3D with point cloud.

    Args:
        point_cloud: Open3D point cloud
        grasp_list: List of grasp dictionaries
        max_grasps: Maximum number of grasps to visualize
    """
    # Apply standard coordinate transformation
    transform = get_standard_coordinate_transform()  # type: ignore[no-untyped-call]

    # Transform point cloud
    pc_copy = o3d.geometry.PointCloud(point_cloud)
    pc_copy.transform(transform)
    geometries = [pc_copy]

    # Transform gripper geometries
    gripper_geometries = create_all_gripper_geometries(grasp_list, max_grasps)
    for geom in gripper_geometries:
        geom.transform(transform)
    geometries.extend(gripper_geometries)

    # Add transformed coordinate frame
    origin_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
    origin_frame.transform(transform)
    geometries.append(origin_frame)

    o3d.visualization.draw_geometries(geometries, window_name="3D Grasp Visualization")


def parse_grasp_results(grasps: list[dict]) -> list[dict]:  # type: ignore[type-arg]
    """
    Parse grasp results into visualization format.

    Args:
        grasps: List of grasp dictionaries

    Returns:
        List of dictionaries containing:
        - id: Unique grasp identifier
        - score: Confidence score (float)
        - width: Gripper opening width (float)
        - translation: 3D position [x, y, z]
        - rotation_matrix: 3x3 rotation matrix as nested list
    """
    if not grasps:
        return []

    parsed_grasps = []

    for i, grasp in enumerate(grasps):
        # Extract data from each grasp
        translation = grasp.get("translation", [0, 0, 0])
        rotation_matrix = np.array(grasp.get("rotation_matrix", np.eye(3)))
        score = float(grasp.get("score", 0.0))
        width = float(grasp.get("width", 0.08))

        parsed_grasp = {
            "id": f"grasp_{i}",
            "score": score,
            "width": width,
            "translation": translation,
            "rotation_matrix": rotation_matrix.tolist(),
        }
        parsed_grasps.append(parsed_grasp)

    return parsed_grasps


def create_grasp_overlay(
    rgb_image: np.ndarray,  # type: ignore[type-arg]
    grasps: list[dict],  # type: ignore[type-arg]
    camera_intrinsics: list[float] | np.ndarray,  # type: ignore[type-arg]
) -> np.ndarray:  # type: ignore[type-arg]
    """
    Create grasp visualization overlay on RGB image.

    Args:
        rgb_image: RGB input image
        grasps: List of grasp dictionaries in viz format
        camera_intrinsics: Camera parameters

    Returns:
        RGB image with grasp overlay
    """
    try:
        bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)

        result_bgr = draw_grasps_on_image(
            bgr_image,
            grasps,
            camera_intrinsics,
            max_grasps=-1,
        )
        return cv2.cvtColor(result_bgr, cv2.COLOR_BGR2RGB)
    except Exception:
        return rgb_image.copy()
