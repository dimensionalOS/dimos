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

from dataclasses import dataclass
from typing import Any

import cv2
from dimos_lcm.vision_msgs import Detection2D, Detection3D  # type: ignore[import-untyped]
import numpy as np

from dimos.msgs.geometry_msgs import Pose, Quaternion, Vector3
from dimos.perception.common.utils import project_2d_points_to_3d
from dimos.perception.detection2d.utils import plot_results
from dimos.utils.transform_utils import (
    compose_transforms,
    euler_to_quaternion,
    get_distance,
    matrix_to_pose,
    offset_distance,
    optical_to_robot_frame,
    pose_to_matrix,
    robot_to_optical_frame,
    yaw_towards_point,
)


def match_detection_by_id(
    detection_3d: Detection3D, detections_3d: list[Detection3D], detections_2d: list[Detection2D]
) -> Detection2D | None:
    """
    Find the corresponding Detection2D for a given Detection3D.

    Args:
        detection_3d: The Detection3D to match
        detections_3d: List of all Detection3D objects
        detections_2d: List of all Detection2D objects (must be 1:1 correspondence)

    Returns:
        Corresponding Detection2D if found, None otherwise
    """
    for i, det_3d in enumerate(detections_3d):
        if det_3d.id == detection_3d.id and i < len(detections_2d):
            return detections_2d[i]
    return None


def transform_pose(
    obj_pos: np.ndarray,  # type: ignore[type-arg]
    obj_orientation: np.ndarray,  # type: ignore[type-arg]
    transform_matrix: np.ndarray,  # type: ignore[type-arg]
    to_optical: bool = False,
    to_robot: bool = False,
) -> Pose:
    """
    Transform object pose with optional frame convention conversion.

    Args:
        obj_pos: Object position [x, y, z]
        obj_orientation: Object orientation [roll, pitch, yaw] in radians
        transform_matrix: 4x4 transformation matrix from camera frame to desired frame
        to_optical: If True, input is in robot frame → convert result to optical frame
        to_robot: If True, input is in optical frame → convert to robot frame first

    Returns:
        Object pose in desired frame as Pose
    """
    # Convert euler angles to quaternion using utility function
    euler_vector = Vector3(obj_orientation[0], obj_orientation[1], obj_orientation[2])
    obj_orientation_quat = euler_to_quaternion(euler_vector)

    input_pose = Pose(
        position=Vector3(obj_pos[0], obj_pos[1], obj_pos[2]), orientation=obj_orientation_quat
    )

    # Apply input frame conversion based on flags
    if to_robot:
        # Input is in optical frame → convert to robot frame first
        pose_for_transform = optical_to_robot_frame(input_pose)
    else:
        # Default or to_optical: use input pose as-is
        pose_for_transform = input_pose

    # Create transformation matrix from pose (relative to camera)
    T_camera_object = pose_to_matrix(pose_for_transform)

    # Use compose_transforms to combine transformations
    T_desired_object = compose_transforms(transform_matrix, T_camera_object)

    # Convert back to pose
    result_pose = matrix_to_pose(T_desired_object)

    # Apply output frame conversion based on flags
    if to_optical:
        # Input was robot frame → convert result to optical frame
        desired_pose = robot_to_optical_frame(result_pose)
    else:
        # Default or to_robot: use result as-is
        desired_pose = result_pose

    return desired_pose


def transform_points_3d(
    points_3d: np.ndarray,  # type: ignore[type-arg]
    transform_matrix: np.ndarray,  # type: ignore[type-arg]
    to_optical: bool = False,
    to_robot: bool = False,
) -> np.ndarray:  # type: ignore[type-arg]
    """
    Transform 3D points with optional frame convention conversion.
    Applies the same transformation pipeline as transform_pose but for multiple points.

    Args:
        points_3d: Nx3 array of 3D points [x, y, z]
        transform_matrix: 4x4 transformation matrix from camera frame to desired frame
        to_optical: If True, input is in robot frame → convert result to optical frame
        to_robot: If True, input is in optical frame → convert to robot frame first

    Returns:
        Nx3 array of transformed 3D points in desired frame
    """
    if points_3d.size == 0:
        return np.zeros((0, 3), dtype=np.float32)

    points_3d = np.asarray(points_3d)
    if points_3d.ndim == 1:
        points_3d = points_3d.reshape(1, -1)

    transformed_points = []

    for point in points_3d:
        input_point_pose = Pose(
            position=Vector3(point[0], point[1], point[2]),
            orientation=Quaternion(0.0, 0.0, 0.0, 1.0),  # Identity quaternion
        )

        # Apply input frame conversion based on flags
        if to_robot:
            # Input is in optical frame → convert to robot frame first
            pose_for_transform = optical_to_robot_frame(input_point_pose)
        else:
            # Default or to_optical: use input pose as-is
            pose_for_transform = input_point_pose

        # Create transformation matrix from point pose (relative to camera)
        T_camera_point = pose_to_matrix(pose_for_transform)

        # Use compose_transforms to combine transformations
        T_desired_point = compose_transforms(transform_matrix, T_camera_point)

        # Convert back to pose
        result_pose = matrix_to_pose(T_desired_point)

        # Apply output frame conversion based on flags
        if to_optical:
            # Input was robot frame → convert result to optical frame
            desired_pose = robot_to_optical_frame(result_pose)
        else:
            # Default or to_robot: use result as-is
            desired_pose = result_pose

        transformed_point = [
            desired_pose.position.x,
            desired_pose.position.y,
            desired_pose.position.z,
        ]
        transformed_points.append(transformed_point)

    return np.array(transformed_points, dtype=np.float32)


def select_points_from_depth(
    depth_image: np.ndarray,  # type: ignore[type-arg]
    target_point: tuple[int, int],
    camera_intrinsics: list[float] | np.ndarray,  # type: ignore[type-arg]
    radius: int = 5,
) -> np.ndarray:  # type: ignore[type-arg]
    """
    Select points around a target point within a bounding box and project them to 3D.

    Args:
        depth_image: Depth image in meters (H, W)
        target_point: (x, y) target point coordinates
        radius: Half-width of the bounding box (so bbox size is radius*2 x radius*2)
        camera_intrinsics: Camera parameters as [fx, fy, cx, cy] list or 3x3 matrix

    Returns:
        Nx3 array of 3D points (X, Y, Z) in camera frame
    """
    x_target, y_target = target_point
    height, width = depth_image.shape

    x_min = max(0, x_target - radius)
    x_max = min(width, x_target + radius)
    y_min = max(0, y_target - radius)
    y_max = min(height, y_target + radius)

    # Create coordinate grids for the bounding box (vectorized)
    y_coords, x_coords = np.meshgrid(range(y_min, y_max), range(x_min, x_max), indexing="ij")

    # Flatten to get all coordinate pairs
    x_flat = x_coords.flatten()
    y_flat = y_coords.flatten()

    # Extract corresponding depth values using advanced indexing
    depth_flat = depth_image[y_flat, x_flat]

    valid_mask = (depth_flat > 0) & np.isfinite(depth_flat)

    if not np.any(valid_mask):
        return np.zeros((0, 3), dtype=np.float32)

    points_2d = np.column_stack([x_flat[valid_mask], y_flat[valid_mask]]).astype(np.float32)
    depth_values = depth_flat[valid_mask].astype(np.float32)

    points_3d = project_2d_points_to_3d(points_2d, depth_values, camera_intrinsics)

    return points_3d


def update_target_grasp_pose(
    target_pose: Pose, ee_pose: Pose, grasp_distance: float = 0.0, grasp_pitch_degrees: float = 45.0
) -> Pose | None:
    """
    Update target grasp pose based on current target pose and EE pose.

    Args:
        target_pose: Target pose to grasp
        ee_pose: Current end-effector pose
        grasp_distance: Distance to maintain from target (pregrasp or grasp distance)
        grasp_pitch_degrees: Grasp pitch angle in degrees (default 90° for top-down)

    Returns:
        Target grasp pose or None if target is invalid
    """

    target_pos = target_pose.position

    # Calculate orientation pointing from target towards EE
    yaw_to_ee = yaw_towards_point(target_pos, ee_pose.position)

    # Create target pose with proper orientation
    # Convert grasp pitch from degrees to radians with mapping:
    # 0° (level) -> π/2 (1.57 rad), 90° (top-down) -> π (3.14 rad)
    pitch_radians = 1.57 + np.radians(grasp_pitch_degrees)

    # Convert euler angles to quaternion using utility function
    euler = Vector3(0.0, pitch_radians, yaw_to_ee)  # roll=0, pitch=mapped, yaw=calculated
    target_orientation = euler_to_quaternion(euler)

    updated_pose = Pose(target_pos, target_orientation)

    if grasp_distance > 0.0:
        return offset_distance(updated_pose, grasp_distance)
    else:
        return updated_pose


def is_target_reached(target_pose: Pose, current_pose: Pose, tolerance: float = 0.01) -> bool:
    """
    Check if the target pose has been reached within tolerance.

    Args:
        target_pose: Target pose to reach
        current_pose: Current pose (e.g., end-effector pose)
        tolerance: Distance threshold for considering target reached (meters, default 0.01 = 1cm)

    Returns:
        True if target is reached within tolerance, False otherwise
    """
    # Calculate position error using distance utility
    error_magnitude = get_distance(target_pose, current_pose)
    return error_magnitude < tolerance


@dataclass
class ObjectMatchResult:
    """Result of object matching with confidence metrics."""

    matched_object: Detection3D | None
    confidence: float
    distance: float
    size_similarity: float
    is_valid_match: bool


def calculate_object_similarity(
    target_obj: Detection3D,
    candidate_obj: Detection3D,
    distance_weight: float = 0.6,
    size_weight: float = 0.4,
) -> tuple[float, float, float]:
    """
    Calculate comprehensive similarity between two objects.

    Args:
        target_obj: Target Detection3D object
        candidate_obj: Candidate Detection3D object
        distance_weight: Weight for distance component (0-1)
        size_weight: Weight for size component (0-1)

    Returns:
        Tuple of (total_similarity, distance_m, size_similarity)
    """
    # Extract positions
    target_pos = target_obj.bbox.center.position
    candidate_pos = candidate_obj.bbox.center.position

    target_xyz = np.array([target_pos.x, target_pos.y, target_pos.z])
    candidate_xyz = np.array([candidate_pos.x, candidate_pos.y, candidate_pos.z])

    # Calculate Euclidean distance
    distance = np.linalg.norm(target_xyz - candidate_xyz)
    distance_similarity = 1.0 / (1.0 + distance)  # Exponential decay

    # Calculate size similarity by comparing each dimension individually
    size_similarity = 1.0  # Default if no size info
    target_size = target_obj.bbox.size
    candidate_size = candidate_obj.bbox.size

    if target_size and candidate_size:
        # Extract dimensions
        target_dims = [target_size.x, target_size.y, target_size.z]
        candidate_dims = [candidate_size.x, candidate_size.y, candidate_size.z]

        # Calculate similarity for each dimension pair
        dim_similarities = []
        for target_dim, candidate_dim in zip(target_dims, candidate_dims, strict=False):
            if target_dim == 0.0 and candidate_dim == 0.0:
                dim_similarities.append(1.0)  # Both dimensions are zero
            elif target_dim == 0.0 or candidate_dim == 0.0:
                dim_similarities.append(0.0)  # One dimension is zero, other is not
            else:
                # Calculate similarity as min/max ratio
                max_dim = max(target_dim, candidate_dim)
                min_dim = min(target_dim, candidate_dim)
                dim_similarity = min_dim / max_dim if max_dim > 0 else 0.0
                dim_similarities.append(dim_similarity)

        # Return average similarity across all dimensions
        size_similarity = np.mean(dim_similarities) if dim_similarities else 0.0  # type: ignore[assignment]

    # Weighted combination
    total_similarity = distance_weight * distance_similarity + size_weight * size_similarity

    return total_similarity, distance, size_similarity  # type: ignore[return-value]


def find_best_object_match(
    target_obj: Detection3D,
    candidates: list[Detection3D],
    max_distance: float = 0.1,
    min_size_similarity: float = 0.4,
    distance_weight: float = 0.7,
    size_weight: float = 0.3,
) -> ObjectMatchResult:
    """
    Find the best matching object from candidates using distance and size criteria.

    Args:
        target_obj: Target Detection3D to match against
        candidates: List of candidate Detection3D objects
        max_distance: Maximum allowed distance for valid match (meters)
        min_size_similarity: Minimum size similarity for valid match (0-1)
        distance_weight: Weight for distance in similarity calculation
        size_weight: Weight for size in similarity calculation

    Returns:
        ObjectMatchResult with best match and confidence metrics
    """
    if not candidates or not target_obj.bbox or not target_obj.bbox.center:
        return ObjectMatchResult(None, 0.0, float("inf"), 0.0, False)

    best_match = None
    best_confidence = 0.0
    best_distance = float("inf")
    best_size_sim = 0.0

    for candidate in candidates:
        if not candidate.bbox or not candidate.bbox.center:
            continue

        similarity, distance, size_sim = calculate_object_similarity(
            target_obj, candidate, distance_weight, size_weight
        )

        # Check validity constraints
        is_valid = distance <= max_distance and size_sim >= min_size_similarity

        if is_valid and similarity > best_confidence:
            best_match = candidate
            best_confidence = similarity
            best_distance = distance
            best_size_sim = size_sim

    return ObjectMatchResult(
        matched_object=best_match,
        confidence=best_confidence,
        distance=best_distance,
        size_similarity=best_size_sim,
        is_valid_match=best_match is not None,
    )


def parse_zed_pose(zed_pose_data: dict[str, Any]) -> Pose | None:
    """
    Parse ZED pose data dictionary into a Pose object.

    Args:
        zed_pose_data: Dictionary from ZEDCamera.get_pose() containing:
            - position: [x, y, z] in meters
            - rotation: [x, y, z, w] quaternion
            - euler_angles: [roll, pitch, yaw] in radians
            - valid: Whether pose is valid

    Returns:
        Pose object with position and orientation, or None if invalid
    """
    if not zed_pose_data or not zed_pose_data.get("valid", False):
        return None

    # Extract position
    position = zed_pose_data.get("position", [0, 0, 0])
    pos_vector = Vector3(position[0], position[1], position[2])

    quat = zed_pose_data["rotation"]
    orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])
    return Pose(position=pos_vector, orientation=orientation)


def estimate_object_depth(
    depth_image: np.ndarray,  # type: ignore[type-arg]
    segmentation_mask: np.ndarray | None,  # type: ignore[type-arg]
    bbox: list[float],
) -> float:
    """
    Estimate object depth dimension using segmentation mask and depth data.
    Optimized for real-time performance.

    Args:
        depth_image: Depth image in meters
        segmentation_mask: Binary segmentation mask for the object
        bbox: Bounding box [x1, y1, x2, y2]

    Returns:
        Estimated object depth in meters
    """
    x1, y1, x2, y2 = int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])

    # Extract depth ROI once
    roi_depth = depth_image[y1:y2, x1:x2]

    if segmentation_mask is not None and segmentation_mask.size > 0:
        # Extract mask ROI efficiently
        mask_roi = (
            segmentation_mask[y1:y2, x1:x2]
            if segmentation_mask.shape != roi_depth.shape
            else segmentation_mask
        )

        # Fast mask application using boolean indexing
        valid_mask = mask_roi > 0
        if np.sum(valid_mask) > 10:  # Early exit if not enough points
            masked_depths = roi_depth[valid_mask]

            # Fast percentile calculation using numpy's optimized functions
            depth_90 = np.percentile(masked_depths, 90)
            depth_10 = np.percentile(masked_depths, 10)
            depth_range = depth_90 - depth_10

            # Clamp to reasonable bounds with single operation
            return np.clip(depth_range, 0.02, 0.5)  # type: ignore[no-any-return]

    # Fast fallback using area calculation
    bbox_area = (x2 - x1) * (y2 - y1)

    # Vectorized area-based estimation
    if bbox_area > 10000:
        return 0.15
    elif bbox_area > 5000:
        return 0.10
    else:
        return 0.05


# ============= Visualization Functions =============


def create_manipulation_visualization(  # type: ignore[no-untyped-def]
    rgb_image: np.ndarray,  # type: ignore[type-arg]
    feedback,
    detection_3d_array=None,
    detection_2d_array=None,
) -> np.ndarray:  # type: ignore[type-arg]
    """
    Create simple visualization for manipulation class using feedback.

    Args:
        rgb_image: RGB image array
        feedback: Feedback object containing all state information
        detection_3d_array: Optional 3D detections for object visualization
        detection_2d_array: Optional 2D detections for object visualization

    Returns:
        BGR image with visualization overlays
    """
    # Convert to BGR for OpenCV
    viz = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)

    # Draw detections if available
    if detection_3d_array and detection_2d_array:
        # Extract 2D bboxes
        bboxes_2d = []
        for det_2d in detection_2d_array.detections:
            if det_2d.bbox:
                x1 = det_2d.bbox.center.position.x - det_2d.bbox.size_x / 2
                y1 = det_2d.bbox.center.position.y - det_2d.bbox.size_y / 2
                x2 = det_2d.bbox.center.position.x + det_2d.bbox.size_x / 2
                y2 = det_2d.bbox.center.position.y + det_2d.bbox.size_y / 2
                bboxes_2d.append([x1, y1, x2, y2])

        # Draw basic detections
        rgb_with_detections = visualize_detections_3d(
            rgb_image, detection_3d_array.detections, show_coordinates=True, bboxes_2d=bboxes_2d
        )
        viz = cv2.cvtColor(rgb_with_detections, cv2.COLOR_RGB2BGR)

    # Add manipulation status overlay
    status_y = 30
    cv2.putText(
        viz,
        "Eye-in-Hand Visual Servoing",
        (10, status_y),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (0, 255, 255),
        2,
    )

    # Stage information
    stage_text = f"Stage: {feedback.grasp_stage.value.upper()}"
    stage_color = {
        "idle": (100, 100, 100),
        "pre_grasp": (0, 255, 255),
        "grasp": (0, 255, 0),
        "close_and_retract": (255, 0, 255),
        "place": (0, 150, 255),
        "retract": (255, 150, 0),
    }.get(feedback.grasp_stage.value, (255, 255, 255))

    cv2.putText(
        viz,
        stage_text,
        (10, status_y + 25),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        stage_color,
        1,
    )

    # Target tracking status
    if feedback.target_tracked:
        cv2.putText(
            viz,
            "Target: TRACKED",
            (10, status_y + 45),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            1,
        )
    elif feedback.grasp_stage.value != "idle":
        cv2.putText(
            viz,
            "Target: LOST",
            (10, status_y + 45),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 0, 255),
            1,
        )

    # Waiting status
    if feedback.waiting_for_reach:
        cv2.putText(
            viz,
            "Status: WAITING FOR ROBOT",
            (10, status_y + 65),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 0),
            1,
        )

    # Overall result
    if feedback.success is not None:
        result_text = "Pick & Place: SUCCESS" if feedback.success else "Pick & Place: FAILED"
        result_color = (0, 255, 0) if feedback.success else (0, 0, 255)
        cv2.putText(
            viz,
            result_text,
            (10, status_y + 85),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            result_color,
            2,
        )

    # Control hints (bottom of image)
    hint_text = "Click object to grasp | s=STOP | r=RESET | g=RELEASE"
    cv2.putText(
        viz,
        hint_text,
        (10, viz.shape[0] - 10),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.4,
        (200, 200, 200),
        1,
    )

    return viz


def create_pbvs_visualization(  # type: ignore[no-untyped-def]
    image: np.ndarray,  # type: ignore[type-arg]
    current_target=None,
    position_error=None,
    target_reached: bool = False,
    grasp_stage: str = "idle",
) -> np.ndarray:  # type: ignore[type-arg]
    """
    Create simple PBVS visualization overlay.

    Args:
        image: Input image (RGB or BGR)
        current_target: Current target Detection3D
        position_error: Position error Vector3
        target_reached: Whether target is reached
        grasp_stage: Current grasp stage string

    Returns:
        Image with PBVS overlay
    """
    viz = image.copy()

    # Only show PBVS info if we have a target
    if current_target is None:
        return viz

    # Create status panel at bottom
    height, width = viz.shape[:2]
    panel_height = 100
    panel_y = height - panel_height

    # Semi-transparent overlay
    overlay = viz.copy()
    cv2.rectangle(overlay, (0, panel_y), (width, height), (0, 0, 0), -1)
    viz = cv2.addWeighted(viz, 0.7, overlay, 0.3, 0)

    # PBVS Status
    y_offset = panel_y + 20
    cv2.putText(
        viz,
        "PBVS Control",
        (10, y_offset),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (0, 255, 255),
        2,
    )

    # Position error
    if position_error:
        error_mag = np.linalg.norm([position_error.x, position_error.y, position_error.z])
        error_text = f"Error: {error_mag * 100:.1f}cm"
        error_color = (0, 255, 0) if target_reached else (0, 255, 255)
        cv2.putText(
            viz,
            error_text,
            (10, y_offset + 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            error_color,
            1,
        )

    # Stage
    cv2.putText(
        viz,
        f"Stage: {grasp_stage}",
        (10, y_offset + 45),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (255, 150, 255),
        1,
    )

    # Target reached indicator
    if target_reached:
        cv2.putText(
            viz,
            "TARGET REACHED",
            (width - 150, y_offset + 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
        )

    return viz


def visualize_detections_3d(
    rgb_image: np.ndarray,  # type: ignore[type-arg]
    detections: list[Detection3D],
    show_coordinates: bool = True,
    bboxes_2d: list[list[float]] | None = None,
) -> np.ndarray:  # type: ignore[type-arg]
    """
    Visualize detections with 3D position overlay next to bounding boxes.

    Args:
        rgb_image: Original RGB image
        detections: List of Detection3D objects
        show_coordinates: Whether to show 3D coordinates next to bounding boxes
        bboxes_2d: Optional list of 2D bounding boxes corresponding to detections

    Returns:
        Visualization image
    """
    if not detections:
        return rgb_image.copy()

    # If no 2D bboxes provided, skip visualization
    if bboxes_2d is None:
        return rgb_image.copy()

    # Extract data for plot_results function
    bboxes = bboxes_2d
    track_ids = [int(det.id) if det.id.isdigit() else i for i, det in enumerate(detections)]
    class_ids = [i for i in range(len(detections))]
    confidences = [
        det.results[0].hypothesis.score if det.results_length > 0 else 0.0 for det in detections
    ]
    names = [
        det.results[0].hypothesis.class_id if det.results_length > 0 else "unknown"
        for det in detections
    ]

    # Use plot_results for basic visualization
    viz = plot_results(rgb_image, bboxes, track_ids, class_ids, confidences, names)

    # Add 3D position coordinates if requested
    if show_coordinates and bboxes_2d is not None:
        for i, det in enumerate(detections):
            if det.bbox and det.bbox.center and i < len(bboxes_2d):
                position = det.bbox.center.position
                bbox = bboxes_2d[i]

                pos_xyz = np.array([position.x, position.y, position.z])

                # Get bounding box coordinates
                _x1, y1, x2, _y2 = map(int, bbox)

                # Add position text next to bounding box (top-right corner)
                pos_text = f"({pos_xyz[0]:.2f}, {pos_xyz[1]:.2f}, {pos_xyz[2]:.2f})"
                text_x = x2 + 5  # Right edge of bbox + small offset
                text_y = y1 + 15  # Top edge of bbox + small offset

                # Add background rectangle for better readability
                text_size = cv2.getTextSize(pos_text, cv2.FONT_HERSHEY_SIMPLEX, 0.4, 1)[0]
                cv2.rectangle(
                    viz,
                    (text_x - 2, text_y - text_size[1] - 2),
                    (text_x + text_size[0] + 2, text_y + 2),
                    (0, 0, 0),
                    -1,
                )

                cv2.putText(
                    viz,
                    pos_text,
                    (text_x, text_y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.4,
                    (255, 255, 255),
                    1,
                )

    return viz  # type: ignore[no-any-return]
