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
from dimos.perception.detection2d.utils import plot_results

logger = setup_logger("dimos.perception.detection3d")


class Detection3DProcessor:
    """
    Real-time 3D detection processor optimized for speed.

    Uses Sam (FastSAM) for segmentation and mask generation, then extracts
    3D centroids and orientations from depth data.
    """

    def __init__(
        self,
        camera_intrinsics: List[float],  # [fx, fy, cx, cy]
        min_confidence: float = 0.6,
        min_points: int = 30,  # Reduced for speed
        max_depth: float = 5.0,  # Reduced for typical manipulation scenarios
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

    def process_frame(self, rgb_image: np.ndarray, depth_image: np.ndarray) -> Dict[str, Any]:
        """
        Process a single RGB-D frame to extract 3D object detections.
        Optimized for real-time performance.

        Args:
            rgb_image: RGB image (H, W, 3)
            depth_image: Depth image (H, W) in meters

        Returns:
            Dictionary containing:
                - detections: List of detection dictionaries with:
                    - bbox: 2D bounding box [x1, y1, x2, y2]
                    - class_name: Object class name
                    - confidence: Detection confidence
                    - centroid: 3D centroid [x, y, z] in camera frame
                    - orientation: Unit vector from camera to object
                    - num_points: Number of valid 3D points
                    - track_id: Tracking ID
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
        pose_dict = {p["mask_idx"]: p for p in poses}

        for i, (bbox, name, prob, track_id) in enumerate(zip(bboxes, names, probs, track_ids)):
            detection = {
                "bbox": bbox.tolist() if isinstance(bbox, np.ndarray) else bbox,
                "class_name": name,
                "confidence": float(prob),
                "track_id": track_id,
            }

            # Add 3D pose if available
            if i in pose_dict:
                pose = pose_dict[i]
                detection["centroid"] = pose["centroid"].tolist()
                detection["orientation"] = pose["orientation"].tolist()
                detection["num_points"] = pose["num_points"]
                detection["has_3d"] = True
            else:
                detection["has_3d"] = False

            detections.append(detection)

        return {"detections": detections, "processing_time": time.time() - start_time}

    def visualize_detections(
        self, rgb_image: np.ndarray, detections: List[Dict[str, Any]], show_3d: bool = True
    ) -> np.ndarray:
        """
        Fast visualization of detections with optional 3D info using plot_results.

        Args:
            rgb_image: Original RGB image
            detections: List of detection dictionaries
            show_3d: Whether to show 3D centroids and orientations

        Returns:
            Visualization image
        """
        if not detections:
            return rgb_image.copy()

        # Extract data for plot_results function
        bboxes = [det["bbox"] for det in detections]
        track_ids = [det.get("track_id", i) for i, det in enumerate(detections)]
        class_ids = [i for i in range(len(detections))]  # Use indices as class IDs
        confidences = [det["confidence"] for det in detections]
        names = [det["class_name"] for det in detections]

        # Use plot_results for basic visualization (bboxes and labels)
        viz = plot_results(rgb_image, bboxes, track_ids, class_ids, confidences, names)

        # Add 3D centroids if requested
        if show_3d:
            for det in detections:
                if det.get("has_3d", False):
                    # Project and draw centroid
                    centroid = np.array(det["centroid"])
                    fx, fy, cx, cy = self.camera_intrinsics

                    if centroid[2] > 0:
                        u = int(centroid[0] * fx / centroid[2] + cx)
                        v = int(centroid[1] * fy / centroid[2] + cy)

                        # Draw centroid circle
                        cv2.circle(viz, (u, v), 6, (255, 0, 0), -1)
                        cv2.circle(viz, (u, v), 8, (255, 255, 255), 2)

        return viz

    def get_closest_detection(
        self, detections: List[Dict[str, Any]], class_filter: Optional[str] = None
    ) -> Optional[Dict[str, Any]]:
        """
        Get the closest detection with valid 3D data.

        Args:
            detections: List of detections
            class_filter: Optional class name to filter by

        Returns:
            Closest detection or None
        """
        valid_detections = [
            d
            for d in detections
            if d.get("has_3d", False) and (class_filter is None or d["class_name"] == class_filter)
        ]

        if not valid_detections:
            return None

        # Sort by depth (Z coordinate)
        return min(valid_detections, key=lambda d: d["centroid"][2])

    def cleanup(self):
        """Clean up resources."""
        if hasattr(self.detector, "cleanup"):
            self.detector.cleanup()
        logger.info("Detection3DProcessor cleaned up")
