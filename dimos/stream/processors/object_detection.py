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

"""Object detection stream processor.

This module provides a modular object detection processor that can be optionally
loaded if YOLO dependencies are available.
"""

from typing import Any, Dict, Optional, List
import numpy as np
import cv2
import logging

from ..stream_processor import StreamProcessor
from dimos.perception.detection2d.yolo_2d_det import Yolo2DDetector
from dimos.perception.detection2d.utils import filter_detections

logger = logging.getLogger(__name__)


class ObjectDetectionProcessor(StreamProcessor):
    """Stream processor for object detection using YOLO."""

    def __init__(self, name: str, config: Optional[Dict[str, Any]] = None):
        """Initialize the object detection processor.

        Args:
            name: Unique name for this processor
            config: Configuration dictionary with the following keys:
                - model_path: Path to YOLO model (default: "yolo11n.pt")
                - device: Computation device (default: "cpu")
                - class_filter: List of class IDs to filter (optional)
                - name_filter: List of class names to filter (optional)
                - confidence_threshold: Minimum confidence threshold (default: 0.5)
        """
        super().__init__(name, config)

        # Set default configuration
        self.model_path = self.config.get("model_path", "yolo11n.pt")
        self.device = self.config.get("device", "cpu")  # Default to CPU for compatibility
        self.class_filter = self.config.get("class_filter")
        self.name_filter = self.config.get("name_filter")
        self.confidence_threshold = self.config.get("confidence_threshold", 0.5)

        self.detector = None

    def initialize(self) -> bool:
        """Initialize the YOLO detector."""
        try:
            # Initialize YOLO detector
            self.detector = Yolo2DDetector(model_path=self.model_path, device=self.device)

            logger.info(f"Initialized object detection with model: {self.model_path}")
            return True

        except Exception as e:
            logger.error(f"Failed to initialize object detection processor: {e}")
            return False

    def process_frame(
        self, frame: Any, metadata: Optional[Dict[str, Any]] = None
    ) -> Dict[str, Any]:
        """Process a frame for object detection.

        Args:
            frame: Input video frame
            metadata: Optional metadata

        Returns:
            Dict containing detection results and visualization
        """
        if self.detector is None:
            raise RuntimeError("Processor not initialized")

        try:
            # Detect objects in the frame
            bboxes, track_ids, class_ids, confidences, names = self.detector.process_image(frame)

            # Apply filtering if specified
            if self.class_filter is not None or self.name_filter is not None:
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
                    class_filter=self.class_filter,
                    name_filter=self.name_filter,
                )
            else:
                # No filtering, use all detections
                filtered_bboxes = bboxes
                filtered_track_ids = track_ids
                filtered_class_ids = class_ids
                filtered_confidences = confidences
                filtered_names = names

            # Apply confidence threshold
            if self.confidence_threshold > 0:
                valid_indices = [
                    i
                    for i, conf in enumerate(filtered_confidences)
                    if conf >= self.confidence_threshold
                ]

                filtered_bboxes = [filtered_bboxes[i] for i in valid_indices]
                filtered_track_ids = (
                    [filtered_track_ids[i] for i in valid_indices] if filtered_track_ids else []
                )
                filtered_class_ids = [filtered_class_ids[i] for i in valid_indices]
                filtered_confidences = [filtered_confidences[i] for i in valid_indices]
                filtered_names = [filtered_names[i] for i in valid_indices]

            # Create visualization
            viz_frame = self.detector.visualize_results(
                frame,
                filtered_bboxes,
                filtered_track_ids,
                filtered_class_ids,
                filtered_confidences,
                filtered_names,
            )

            # Create target data
            targets = []
            for i, bbox in enumerate(filtered_bboxes):
                target_data = {
                    "target_id": filtered_track_ids[i] if i < len(filtered_track_ids) else -1,
                    "bbox": bbox,
                    "confidence": filtered_confidences[i]
                    if i < len(filtered_confidences)
                    else None,
                    "class_id": filtered_class_ids[i] if i < len(filtered_class_ids) else None,
                    "class_name": filtered_names[i] if i < len(filtered_names) else "unknown",
                }
                targets.append(target_data)

            return {
                "frame": frame,
                "viz_frame": viz_frame,
                "targets": targets,
                "processor": self.name,
                "metadata": metadata or {},
            }

        except Exception as e:
            logger.error(f"Error processing frame in object detection: {e}")
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
