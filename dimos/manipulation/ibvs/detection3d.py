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

from dimos.types.pose import Pose
from dimos.types.vector import Vector
from dimos.types.manipulation import ObjectData
from dimos.manipulation.ibvs.utils import estimate_object_depth
from dimos.utils.transform_utils import (
    optical_to_robot_frame,
    pose_to_matrix,
    matrix_to_pose,
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
        self, rgb_image: np.ndarray, depth_image: np.ndarray, camera_pose: Optional[Any] = None
    ) -> Dict[str, Any]:
        """
        Process a single RGB-D frame to extract 3D object detections.

        Args:
            rgb_image: RGB image (H, W, 3)
            depth_image: Depth image (H, W) in meters
            camera_pose: Optional camera pose in world frame (Pose object in ZED coordinates)

        Returns:
            Dictionary containing:
                - detections: List of ObjectData objects with 3D pose information
                - processing_time: Total processing time in seconds
        """
        start_time = time.time()

        # Convert RGB to BGR for Sam (OpenCV format)
        bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)

        # Run Sam segmentation with tracking
        masks, bboxes, track_ids, probs, names = self.detector.process_image(bgr_image)

        # Early exit if no detections
        if not masks or len(masks) == 0:
            return {"detections": [], "processing_time": time.time() - start_time}

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

                # Transform to world frame if camera pose is available
                if camera_pose is not None:
                    # Get orientation as euler angles, default to no rotation if not available
                    obj_cam_orientation = pose.get(
                        "rotation", np.array([0.0, 0.0, 0.0])
                    )  # Default to no rotation
                    world_pose = self._transform_to_world(
                        obj_cam_pos, obj_cam_orientation, camera_pose
                    )
                    obj_data["world_position"] = world_pose.pos
                    obj_data["position"] = world_pose.pos  # Use world position
                    obj_data["rotation"] = world_pose.rot  # Use world rotation
                else:
                    # If no camera pose, use camera coordinates
                    obj_data["position"] = Vector(obj_cam_pos[0], obj_cam_pos[1], obj_cam_pos[2])

                detections.append(obj_data)

        return {"detections": detections, "processing_time": time.time() - start_time}

    def _transform_to_world(
        self, obj_pos: np.ndarray, obj_orientation: np.ndarray, camera_pose: Pose
    ) -> Pose:
        """
        Transform object pose from optical frame to world frame.

        Args:
            obj_pos: Object position in optical frame [x, y, z]
            obj_orientation: Object orientation in optical frame [roll, pitch, yaw] in radians
            camera_pose: Camera pose in world frame (x forward, y left, z up)

        Returns:
            Object pose in world frame as Pose
        """
        # Create object pose in optical frame
        obj_pose_optical = Pose(
            Vector(obj_pos[0], obj_pos[1], obj_pos[2]),
            Vector([obj_orientation[0], obj_orientation[1], obj_orientation[2]]),
        )

        # Transform object pose from optical frame to world frame convention
        obj_pose_world_frame = optical_to_robot_frame(obj_pose_optical)

        # Create transformation matrix from camera pose
        T_world_camera = pose_to_matrix(camera_pose)

        # Create transformation matrix from object pose (relative to camera)
        T_camera_object = pose_to_matrix(obj_pose_world_frame)

        # Combine transformations: T_world_object = T_world_camera * T_camera_object
        T_world_object = T_world_camera @ T_camera_object

        # Convert back to pose
        world_pose = matrix_to_pose(T_world_object)

        return world_pose

    def visualize_detections(
        self,
        rgb_image: np.ndarray,
        detections: List[ObjectData],
        pbvs_controller: Optional[Any] = None,
    ) -> np.ndarray:
        """
        Visualize detections with 3D position overlay next to bounding boxes.

        Args:
            rgb_image: Original RGB image
            detections: List of ObjectData objects
            pbvs_controller: Optional PBVS controller to get robot frame coordinates

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

        # Add 3D position overlay next to bounding boxes
        fx, fy, cx, cy = self.camera_intrinsics

        for det in detections:
            if "position" in det and "bbox" in det:
                # Get position to display (robot frame if available, otherwise world frame)
                world_position = det["position"]
                display_position = world_position
                frame_label = ""

                # Check if we should display robot frame coordinates
                if pbvs_controller and pbvs_controller.manipulator_origin is not None:
                    robot_frame_data = pbvs_controller.get_object_pose_robot_frame(world_position)
                    if robot_frame_data:
                        display_position, _ = robot_frame_data
                        frame_label = "[R]"  # Robot frame indicator

                bbox = det["bbox"]

                if isinstance(display_position, Vector):
                    display_xyz = np.array(
                        [display_position.x, display_position.y, display_position.z]
                    )
                else:
                    display_xyz = np.array(
                        [display_position["x"], display_position["y"], display_position["z"]]
                    )

                # Get bounding box coordinates
                x1, y1, x2, y2 = map(int, bbox)

                # Add position text next to bounding box (top-right corner)
                pos_text = f"{frame_label}({display_xyz[0]:.2f}, {display_xyz[1]:.2f}, {display_xyz[2]:.2f})"
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
            if isinstance(pos, Vector):
                return abs(pos.z)
            return abs(pos["z"])

        return min(valid_detections, key=get_z_coord)

    def cleanup(self):
        """Clean up resources."""
        if hasattr(self.detector, "cleanup"):
            self.detector.cleanup()
        logger.info("Detection3DProcessor cleaned up")
