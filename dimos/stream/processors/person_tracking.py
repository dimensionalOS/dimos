# Copyright 2025-2026 Dimensional Inc.
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

"""Person tracking stream processor.

This module provides a modular person tracking processor that can be optionally
loaded if YOLO dependencies are available.
"""

from typing import Any, Dict, Optional
import numpy as np
import cv2
import logging

from ..stream_processor import StreamProcessor
from dimos.perception.detection2d.yolo_2d_det import Yolo2DDetector
from dimos.perception.detection2d.utils import filter_detections
from dimos.perception.common.ibvs import PersonDistanceEstimator

logger = logging.getLogger(__name__)


class PersonTrackingProcessor(StreamProcessor):
    """Stream processor for person tracking using YOLO detection and distance estimation."""

    def __init__(self, name: str, config: Optional[Dict[str, Any]] = None):
        """Initialize the person tracking processor.

        Args:
            name: Unique name for this processor
            config: Configuration dictionary with the following keys:
                - model_path: Path to YOLO model (default: "yolo11n.pt")
                - device: Computation device (default: "cpu")
                - camera_intrinsics: Camera intrinsics [fx, fy, cx, cy]
                - camera_pitch: Camera pitch in radians (default: 0.0)
                - camera_height: Camera height in meters (default: 1.0)
        """
        super().__init__(name, config)

        # Set default configuration
        self.model_path = self.config.get("model_path", "yolo11n.pt")
        self.device = self.config.get("device", "cpu")  # Default to CPU for compatibility
        self.camera_intrinsics = self.config.get("camera_intrinsics")
        self.camera_pitch = self.config.get("camera_pitch", 0.0)
        self.camera_height = self.config.get("camera_height", 1.0)

        self.detector = None
        self.distance_estimator = None

    def initialize(self) -> bool:
        """Initialize the YOLO detector and distance estimator."""
        try:
            # Initialize YOLO detector
            self.detector = Yolo2DDetector(model_path=self.model_path, device=self.device)

            # Initialize distance estimator if camera intrinsics are provided
            if self.camera_intrinsics is not None:
                if (
                    not isinstance(self.camera_intrinsics, (list, tuple, np.ndarray))
                    or len(self.camera_intrinsics) != 4
                ):
                    raise ValueError("Camera intrinsics must be provided as [fx, fy, cx, cy]")

                fx, fy, cx, cy = self.camera_intrinsics
                K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float32)

                self.distance_estimator = PersonDistanceEstimator(
                    K=K, camera_pitch=self.camera_pitch, camera_height=self.camera_height
                )
                logger.info(f"Initialized person tracking with distance estimation")
            else:
                logger.info(
                    f"Initialized person tracking without distance estimation (no camera intrinsics)"
                )

            return True

        except Exception as e:
            logger.error(f"Failed to initialize person tracking processor: {e}")
            return False

    def process_frame(
        self, frame: Any, metadata: Optional[Dict[str, Any]] = None
    ) -> Dict[str, Any]:
        """Process a frame for person detection and tracking.

        Args:
            frame: Input video frame
            metadata: Optional metadata

        Returns:
            Dict containing detection results and visualization
        """
        if self.detector is None:
            raise RuntimeError("Processor not initialized")

        try:
            # Detect people in the frame
            bboxes, track_ids, class_ids, confidences, names = self.detector.process_image(frame)

            # Filter to keep only person detections
            (
                filtered_bboxes,
                filtered_track_ids,
                filtered_class_ids,
                filtered_confidences,
                filtered_names,
            ) = filter_detections(
                bboxes,
                track_ids,
                class_ids,
                confidences,
                names,
                class_filter=[0],  # 0 is the class_id for person
                name_filter=["person"],
            )

            # Create visualization
            viz_frame = self.detector.visualize_results(
                frame,
                filtered_bboxes,
                filtered_track_ids,
                filtered_class_ids,
                filtered_confidences,
                filtered_names,
            )

            # Calculate distance and angle for each person
            targets = []
            for i, bbox in enumerate(filtered_bboxes):
                target_data = {
                    "target_id": filtered_track_ids[i] if i < len(filtered_track_ids) else -1,
                    "bbox": bbox,
                    "confidence": filtered_confidences[i]
                    if i < len(filtered_confidences)
                    else None,
                    "class_name": "person",
                }

                # Add distance and angle if distance estimator is available
                if self.distance_estimator is not None:
                    distance, angle = self.distance_estimator.estimate_distance_angle(bbox)
                    target_data["distance"] = distance
                    target_data["angle"] = angle

                    # Add text to visualization
                    x1, y1, x2, y2 = map(int, bbox)
                    dist_text = f"{distance:.2f}m, {np.rad2deg(angle):.1f} deg"

                    # Add black background for better visibility
                    text_size = cv2.getTextSize(dist_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)[0]
                    cv2.rectangle(
                        viz_frame,
                        (x2 - text_size[0], y1 - text_size[1] - 5),
                        (x2, y1),
                        (0, 0, 0),
                        -1,
                    )

                    # Draw text in white at top-right
                    cv2.putText(
                        viz_frame,
                        dist_text,
                        (x2 - text_size[0], y1 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (255, 255, 255),
                        2,
                    )

                targets.append(target_data)

            return {
                "frame": frame,
                "viz_frame": viz_frame,
                "targets": targets,
                "processor": self.name,
                "metadata": metadata or {},
            }

        except Exception as e:
            logger.error(f"Error processing frame in person tracking: {e}")
            return {
                "frame": frame,
                "viz_frame": frame,
                "targets": [],
                "error": str(e),
                "processor": self.name,
                "metadata": metadata or {},
            }

    def cleanup(self) -> None:
        """Clean up resources."""
        try:
            if hasattr(self.detector, "cleanup"):
                self.detector.cleanup()
        except Exception as e:
            logger.error(f"Error during cleanup: {e}")

        self.detector = None
        self.distance_estimator = None
