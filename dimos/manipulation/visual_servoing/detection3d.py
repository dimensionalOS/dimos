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
Real-time 3D object detection processor that extracts object poses from RGB-D data.
"""

import time
from typing import Dict, List, Optional, Any
import numpy as np
import cv2

from dimos.utils.logging_config import setup_logger
from dimos.perception.segmentation.sam_2d_seg import Sam2DSegmenter
from dimos.perception.pointcloud.utils import extract_centroids_from_masks
from dimos.perception.detection2d.utils import plot_results, calculate_object_size_from_bbox

from dimos.msgs.geometry_msgs import Pose, Vector3, Quaternion
from dimos.types.manipulation import ObjectData
from dimos.manipulation.visual_servoing.utils import estimate_object_depth
from dimos.utils.transform_utils import (
    optical_to_robot_frame,
    pose_to_matrix,
    matrix_to_pose,
    euler_to_quaternion,
    compose_transforms,
)

logger = setup_logger("dimos.perception.detection3d")


class Detection3DProcessor:
    """
    Real-time 3D detection processor optimized for speed.

    Uses Sam (FastSAM) for segmentation and mask generation, then extracts
    3D centroids from depth data.
    """

    def __init__(
        self,
        camera_intrinsics: List[float],  # [fx, fy, cx, cy]
        min_confidence: float = 0.6,
        min_points: int = 30,
        max_depth: float = 1.0,
    ):
        """
        Initialize the real-time 3D detection processor.

        Args:
            camera_intrinsics: [fx, fy, cx, cy] camera parameters
            min_confidence: Minimum detection confidence threshold
            min_points: Minimum 3D points required for valid detection
            max_depth: Maximum valid depth in meters
        """
        self.camera_intrinsics = camera_intrinsics
        self.min_points = min_points
        self.max_depth = max_depth

        # Initialize Sam segmenter with tracking enabled but analysis disabled
        self.detector = Sam2DSegmenter(
            use_tracker=False,
            use_analyzer=False,
            device="cuda" if cv2.cuda.getCudaEnabledDeviceCount() > 0 else "cpu",
        )

        # Store confidence threshold for filtering
        self.min_confidence = min_confidence

        logger.info(
            f"Initialized Detection3DProcessor with Sam segmenter, confidence={min_confidence}, "
            f"min_points={min_points}, max_depth={max_depth}m"
        )

    def process_frame(
        self, rgb_image: np.ndarray, depth_image: np.ndarray, transform: Optional[np.ndarray] = None
    ) -> List[ObjectData]:
        """
        Process a single RGB-D frame to extract 3D object detections.

        Args:
            rgb_image: RGB image (H, W, 3)
            depth_image: Depth image (H, W) in meters
            transform: Optional 4x4 transformation matrix to transform objects from camera frame to desired frame

        Returns:
            List of ObjectData objects with 3D pose information
        """

        # Convert RGB to BGR for Sam (OpenCV format)
        bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)

        # Run Sam segmentation with tracking
        masks, bboxes, track_ids, probs, names = self.detector.process_image(bgr_image)

        # Early exit if no detections
        if not masks or len(masks) == 0:
            return []

        # Convert CUDA tensors to numpy arrays if needed
        numpy_masks = []
        for mask in masks:
            if hasattr(mask, "cpu"):  # PyTorch tensor
                numpy_masks.append(mask.cpu().numpy())
            else:  # Already numpy array
                numpy_masks.append(mask)

        # Extract 3D centroids from masks
        poses = extract_centroids_from_masks(
            rgb_image=rgb_image,
            depth_image=depth_image,
            masks=numpy_masks,
            camera_intrinsics=self.camera_intrinsics,
            min_points=self.min_points,
            max_depth=self.max_depth,
        )

        # Build detection results
        detections = []
        pose_dict = {p["mask_idx"]: p for p in poses if p["centroid"][2] < self.max_depth}

        for i, (bbox, name, prob, track_id) in enumerate(zip(bboxes, names, probs, track_ids)):
            # Create ObjectData object
            obj_data: ObjectData = {
                "object_id": track_id,
                "bbox": bbox.tolist() if isinstance(bbox, np.ndarray) else bbox,
                "confidence": float(prob),
                "label": name,
                "movement_tolerance": 1.0,  # Default to freely movable
                "segmentation_mask": numpy_masks[i] if i < len(numpy_masks) else np.array([]),
            }

            # Add 3D pose if available
            if i in pose_dict:
                pose = pose_dict[i]
                obj_cam_pos = pose["centroid"]

                # Set depth and position in camera frame
                obj_data["depth"] = float(obj_cam_pos[2])

                obj_data["rotation"] = None

                # Calculate object size from bbox and depth
                width_m, height_m = calculate_object_size_from_bbox(
                    bbox, obj_cam_pos[2], self.camera_intrinsics
                )

                # Calculate depth dimension using segmentation mask
                depth_m = estimate_object_depth(
                    depth_image, numpy_masks[i] if i < len(numpy_masks) else None, bbox
                )

                obj_data["size"] = {
                    "width": max(width_m, 0.01),  # Minimum 1cm width
                    "height": max(height_m, 0.01),  # Minimum 1cm height
                    "depth": max(depth_m, 0.01),  # Minimum 1cm depth
                }

                # Extract average color from the region
                x1, y1, x2, y2 = map(int, bbox)
                roi = rgb_image[y1:y2, x1:x2]
                if roi.size > 0:
                    avg_color = np.mean(roi.reshape(-1, 3), axis=0)
                    obj_data["color"] = avg_color.astype(np.uint8)
                else:
                    obj_data["color"] = np.array([128, 128, 128], dtype=np.uint8)

                # Transform to desired frame if transform matrix is provided
                if transform is not None:
                    # Get orientation as euler angles, default to no rotation if not available
                    obj_cam_orientation = pose.get(
                        "rotation", np.array([0.0, 0.0, 0.0])
                    )  # Default to no rotation
                    transformed_pose = self._transform_object_pose(
                        obj_cam_pos, obj_cam_orientation, transform
                    )
                    obj_data["position"] = transformed_pose.position
                    obj_data["rotation"] = transformed_pose.orientation
                else:
                    # If no transform, use camera coordinates
                    obj_data["position"] = Vector3(obj_cam_pos[0], obj_cam_pos[1], obj_cam_pos[2])

                detections.append(obj_data)

        return detections

    def _transform_object_pose(
        self, obj_pos: np.ndarray, obj_orientation: np.ndarray, transform_matrix: np.ndarray
    ) -> Pose:
        """
        Transform object pose from optical frame to desired frame using transformation matrix.

        Args:
            obj_pos: Object position in optical frame [x, y, z]
            obj_orientation: Object orientation in optical frame [roll, pitch, yaw] in radians
            transform_matrix: 4x4 transformation matrix from camera frame to desired frame

        Returns:
            Object pose in desired frame as Pose
        """
        # Create object pose in optical frame
        # Convert euler angles to quaternion using utility function
        euler_vector = Vector3(obj_orientation[0], obj_orientation[1], obj_orientation[2])
        obj_orientation_quat = euler_to_quaternion(euler_vector)

        obj_pose_optical = Pose(Vector3(obj_pos[0], obj_pos[1], obj_pos[2]), obj_orientation_quat)

        # Transform object pose from optical frame to robot frame convention first
        obj_pose_robot_frame = optical_to_robot_frame(obj_pose_optical)

        # Create transformation matrix from object pose (relative to camera)
        T_camera_object = pose_to_matrix(obj_pose_robot_frame)

        # Use compose_transforms to combine transformations
        T_desired_object = compose_transforms(transform_matrix, T_camera_object)

        # Convert back to pose
        desired_pose = matrix_to_pose(T_desired_object)

        return desired_pose

    def visualize_detections(
        self,
        rgb_image: np.ndarray,
        detections: List[ObjectData],
        show_coordinates: bool = True,
    ) -> np.ndarray:
        """
        Visualize detections with 3D position overlay next to bounding boxes.

        Args:
            rgb_image: Original RGB image
            detections: List of ObjectData objects
            show_coordinates: Whether to show 3D coordinates next to bounding boxes

        Returns:
            Visualization image
        """
        if not detections:
            return rgb_image.copy()

        # Extract data for plot_results function
        bboxes = [det["bbox"] for det in detections]
        track_ids = [det.get("object_id", i) for i, det in enumerate(detections)]
        class_ids = [i for i in range(len(detections))]
        confidences = [det["confidence"] for det in detections]
        names = [det["label"] for det in detections]

        # Use plot_results for basic visualization
        viz = plot_results(rgb_image, bboxes, track_ids, class_ids, confidences, names)

        # Add 3D position coordinates if requested
        if show_coordinates:
            for det in detections:
                if "position" in det and "bbox" in det:
                    position = det["position"]
                    bbox = det["bbox"]

                    if isinstance(position, Vector3):
                        pos_xyz = np.array([position.x, position.y, position.z])
                    else:
                        pos_xyz = np.array([position["x"], position["y"], position["z"]])

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

    def get_closest_detection(
        self, detections: List[ObjectData], class_filter: Optional[str] = None
    ) -> Optional[ObjectData]:
        """
        Get the closest detection with valid 3D data.

        Args:
            detections: List of ObjectData objects
            class_filter: Optional class name to filter by

        Returns:
            Closest ObjectData or None
        """
        valid_detections = [
            d
            for d in detections
            if "position" in d and (class_filter is None or d["label"] == class_filter)
        ]

        if not valid_detections:
            return None

        # Sort by depth (Z coordinate)
        def get_z_coord(d):
            pos = d["position"]
            if isinstance(pos, Vector3):
                return abs(pos.z)
            return abs(pos["z"])

        return min(valid_detections, key=get_z_coord)

    def cleanup(self):
        """Clean up resources."""
        if hasattr(self.detector, "cleanup"):
            self.detector.cleanup()
        logger.info("Detection3DProcessor cleaned up")
