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

import numpy as np
from typing import Dict, Any, Optional, List, Tuple
from dataclasses import dataclass

from dimos_lcm.geometry_msgs import Pose, Vector3, Quaternion, Point
from dimos_lcm.vision_msgs import Detection3D, Detection2D
import cv2
from dimos.perception.detection2d.utils import plot_results


@dataclass
class ObjectMatchResult:
    """Result of object matching with confidence metrics."""

    matched_object: Optional[Detection3D]
    confidence: float
    distance: float
    size_similarity: float
    is_valid_match: bool


def calculate_object_similarity(
    target_obj: Detection3D,
    candidate_obj: Detection3D,
    distance_weight: float = 0.6,
    size_weight: float = 0.4,
) -> Tuple[float, float, float]:
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
        for target_dim, candidate_dim in zip(target_dims, candidate_dims):
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
        size_similarity = np.mean(dim_similarities) if dim_similarities else 0.0

    # Weighted combination
    total_similarity = distance_weight * distance_similarity + size_weight * size_similarity

    return total_similarity, distance, size_similarity


def find_best_object_match(
    target_obj: Detection3D,
    candidates: List[Detection3D],
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


def parse_zed_pose(zed_pose_data: Dict[str, Any]) -> Optional[Pose]:
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
    pos_vector = Point(position[0], position[1], position[2])

    quat = zed_pose_data["rotation"]
    orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])
    return Pose(pos_vector, orientation)


def estimate_object_depth(
    depth_image: np.ndarray, segmentation_mask: Optional[np.ndarray], bbox: List[float]
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
            return np.clip(depth_range, 0.02, 0.5)

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


def visualize_detections_3d(
    rgb_image: np.ndarray,
    detections: List[Detection3D],
    show_coordinates: bool = True,
    bboxes_2d: Optional[List[List[float]]] = None,
) -> np.ndarray:
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
                x1, y1, x2, y2 = map(int, bbox)

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

    return viz


def create_pbvs_status_overlay(
    image: np.ndarray,
    current_target: Optional[Detection3D],
    position_error: Optional[Vector3],
    target_reached: bool,
    target_grasp_pose: Optional[Pose],
    grasp_stage: str,
    is_direct_control: bool = False,
) -> np.ndarray:
    """
    Create PBVS status overlay for direct control mode.

    Args:
        image: Input image
        current_target: Current target Detection3D
        position_error: Position error vector
        target_reached: Whether target is reached
        target_grasp_pose: Target grasp pose
        grasp_stage: Current grasp stage
        is_direct_control: Whether in direct control mode

    Returns:
        Image with status overlay
    """
    viz_img = image.copy()
    height, width = image.shape[:2]

    # Status panel
    if current_target is not None:
        panel_height = 175  # Adjusted panel for target, grasp pose, stage, and distance info
        panel_y = height - panel_height
        overlay = viz_img.copy()
        cv2.rectangle(overlay, (0, panel_y), (width, height), (0, 0, 0), -1)
        viz_img = cv2.addWeighted(viz_img, 0.7, overlay, 0.3, 0)

        # Status text
        y = panel_y + 20
        mode_text = "Direct EE" if is_direct_control else "Velocity"
        cv2.putText(
            viz_img,
            f"PBVS Status ({mode_text})",
            (10, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 255),
            2,
        )

        # Add frame info
        cv2.putText(
            viz_img, "Frame: Camera", (250, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1
        )

        if position_error:
            error_mag = np.linalg.norm(
                [
                    position_error.x,
                    position_error.y,
                    position_error.z,
                ]
            )
            color = (0, 255, 0) if target_reached else (0, 255, 255)

            cv2.putText(
                viz_img,
                f"Pos Error: {error_mag:.3f}m ({error_mag * 100:.1f}cm)",
                (10, y + 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                1,
            )

            cv2.putText(
                viz_img,
                f"XYZ: ({position_error.x:.3f}, {position_error.y:.3f}, {position_error.z:.3f})",
                (10, y + 45),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (200, 200, 200),
                1,
            )

        # Show target and grasp poses
        if current_target and current_target.bbox and current_target.bbox.center:
            target_pos = current_target.bbox.center.position
            cv2.putText(
                viz_img,
                f"Target: ({target_pos.x:.3f}, {target_pos.y:.3f}, {target_pos.z:.3f})",
                (10, y + 65),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (255, 255, 0),
                1,
            )

        if target_grasp_pose:
            grasp_pos = target_grasp_pose.position
            cv2.putText(
                viz_img,
                f"Grasp:  ({grasp_pos.x:.3f}, {grasp_pos.y:.3f}, {grasp_pos.z:.3f})",
                (10, y + 80),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (0, 255, 255),
                1,
            )

            # Show pregrasp distance if we have both poses
            if current_target and current_target.bbox and current_target.bbox.center:
                target_pos = current_target.bbox.center.position
                distance = np.sqrt(
                    (grasp_pos.x - target_pos.x) ** 2
                    + (grasp_pos.y - target_pos.y) ** 2
                    + (grasp_pos.z - target_pos.z) ** 2
                )

                # Show current stage and distance
                stage_text = f"Stage: {grasp_stage}"
                cv2.putText(
                    viz_img,
                    stage_text,
                    (10, y + 95),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.4,
                    (255, 150, 255),
                    1,
                )

                distance_text = f"Distance: {distance * 1000:.1f}mm"
                cv2.putText(
                    viz_img,
                    distance_text,
                    (10, y + 110),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.4,
                    (255, 200, 0),
                    1,
                )

        if target_reached:
            cv2.putText(
                viz_img,
                "TARGET REACHED",
                (width - 150, y + 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
            )

    return viz_img


def create_pbvs_controller_overlay(
    image: np.ndarray,
    current_target: Optional[Detection3D],
    position_error: Optional[Vector3],
    rotation_error: Optional[Vector3],
    velocity_cmd: Optional[Vector3],
    angular_velocity_cmd: Optional[Vector3],
    target_reached: bool,
    direct_ee_control: bool = False,
) -> np.ndarray:
    """
    Create PBVS controller status overlay on image.

    Args:
        image: Input image
        current_target: Current target Detection3D (for display)
        position_error: Position error vector
        rotation_error: Rotation error vector
        velocity_cmd: Linear velocity command
        angular_velocity_cmd: Angular velocity command
        target_reached: Whether target is reached
        direct_ee_control: Whether in direct EE control mode

    Returns:
        Image with PBVS status overlay
    """
    viz_img = image.copy()
    height, width = image.shape[:2]

    # Status panel
    if current_target is not None:
        panel_height = 160  # Adjusted panel height
        panel_y = height - panel_height
        overlay = viz_img.copy()
        cv2.rectangle(overlay, (0, panel_y), (width, height), (0, 0, 0), -1)
        viz_img = cv2.addWeighted(viz_img, 0.7, overlay, 0.3, 0)

        # Status text
        y = panel_y + 20
        mode_text = "Direct EE" if direct_ee_control else "Velocity"
        cv2.putText(
            viz_img,
            f"PBVS Status ({mode_text})",
            (10, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 255),
            2,
        )

        # Add frame info
        cv2.putText(
            viz_img, "Frame: Camera", (250, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1
        )

        if position_error:
            error_mag = np.linalg.norm(
                [
                    position_error.x,
                    position_error.y,
                    position_error.z,
                ]
            )
            color = (0, 255, 0) if target_reached else (0, 255, 255)

            cv2.putText(
                viz_img,
                f"Pos Error: {error_mag:.3f}m ({error_mag * 100:.1f}cm)",
                (10, y + 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                1,
            )

            cv2.putText(
                viz_img,
                f"XYZ: ({position_error.x:.3f}, {position_error.y:.3f}, {position_error.z:.3f})",
                (10, y + 45),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (200, 200, 200),
                1,
            )

        if velocity_cmd and not direct_ee_control:
            cv2.putText(
                viz_img,
                f"Lin Vel: ({velocity_cmd.x:.2f}, {velocity_cmd.y:.2f}, {velocity_cmd.z:.2f})m/s",
                (10, y + 65),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 200, 0),
                1,
            )

        if rotation_error:
            cv2.putText(
                viz_img,
                f"Rot Error: ({rotation_error.x:.2f}, {rotation_error.y:.2f}, {rotation_error.z:.2f})rad",
                (10, y + 85),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (200, 200, 200),
                1,
            )

        if angular_velocity_cmd and not direct_ee_control:
            cv2.putText(
                viz_img,
                f"Ang Vel: ({angular_velocity_cmd.x:.2f}, {angular_velocity_cmd.y:.2f}, {angular_velocity_cmd.z:.2f})rad/s",
                (10, y + 105),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 200, 0),
                1,
            )

        if target_reached:
            cv2.putText(
                viz_img,
                "TARGET REACHED",
                (width - 150, y + 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
            )

    return viz_img


def match_detection_by_id(
    detection_3d: Detection3D, detections_3d: List[Detection3D], detections_2d: List[Detection2D]
) -> Optional[Detection2D]:
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
