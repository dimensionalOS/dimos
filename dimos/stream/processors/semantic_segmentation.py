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

"""Semantic segmentation stream processor.

This module provides a modular semantic segmentation processor that can be optionally
loaded if FastSAM dependencies are available.
"""

from typing import Any, Dict, Optional, List
import numpy as np
import cv2
import logging

from ..stream_processor import StreamProcessor
from ultralytics import FastSAM

logger = logging.getLogger(__name__)


class SemanticSegmentationProcessor(StreamProcessor):
    """Stream processor for semantic segmentation using FastSAM."""

    def __init__(self, name: str, config: Optional[Dict[str, Any]] = None):
        """Initialize the semantic segmentation processor.

        Args:
            name: Unique name for this processor
            config: Configuration dictionary with the following keys:
                - model_path: Path to FastSAM model (default: "FastSAM-s.pt")
                - device: Computation device (default: "cpu")
                - confidence_threshold: Minimum confidence threshold (default: 0.4)
                - iou_threshold: IoU threshold for NMS (default: 0.9)
                - max_detections: Maximum number of detections (default: 100)
                - retina_masks: Whether to use retina masks (default: True)
        """
        super().__init__(name, config)

        # Set default configuration
        self.model_path = self.config.get("model_path", "FastSAM-s.pt")
        self.device = self.config.get("device", "cpu")  # Default to CPU for compatibility
        self.confidence_threshold = self.config.get("confidence_threshold", 0.4)
        self.iou_threshold = self.config.get("iou_threshold", 0.9)
        self.max_detections = self.config.get("max_detections", 100)
        self.retina_masks = self.config.get("retina_masks", True)

        self.model = None

    def initialize(self) -> bool:
        """Initialize the FastSAM model."""
        try:
            # Initialize FastSAM model
            self.model = FastSAM(self.model_path)

            logger.info(f"Initialized semantic segmentation with model: {self.model_path}")
            return True

        except Exception as e:
            logger.error(f"Failed to initialize semantic segmentation processor: {e}")
            return False

    def process_frame(
        self, frame: Any, metadata: Optional[Dict[str, Any]] = None
    ) -> Dict[str, Any]:
        """Process a frame for semantic segmentation.

        Args:
            frame: Input video frame
            metadata: Optional metadata

        Returns:
            Dict containing segmentation results and visualization
        """
        if self.model is None:
            raise RuntimeError("Processor not initialized")

        try:
            # Run segmentation
            results = self.model(
                frame,
                device=self.device,
                retina_masks=self.retina_masks,
                imgsz=1024,
                conf=self.confidence_threshold,
                iou=self.iou_threshold,
                max_det=self.max_detections,
            )

            # Extract results
            if len(results) > 0 and results[0].masks is not None:
                masks = results[0].masks.data.cpu().numpy()
                boxes = results[0].boxes.xyxy.cpu().numpy() if results[0].boxes is not None else []
                confidences = (
                    results[0].boxes.conf.cpu().numpy() if results[0].boxes is not None else []
                )

                # Create visualization
                viz_frame = frame.copy()

                # Generate random colors for masks
                colors = np.random.randint(0, 255, size=(len(masks), 3), dtype=np.uint8)

                # Overlay masks
                for i, mask in enumerate(masks):
                    # Resize mask to match frame size if needed
                    if mask.shape != frame.shape[:2]:
                        mask = cv2.resize(mask.astype(np.uint8), (frame.shape[1], frame.shape[0]))

                    # Create colored mask
                    colored_mask = np.zeros_like(frame)
                    colored_mask[mask > 0.5] = colors[i]

                    # Blend with original frame
                    viz_frame = cv2.addWeighted(viz_frame, 0.7, colored_mask, 0.3, 0)

                # Draw bounding boxes if available
                if len(boxes) > 0:
                    for i, box in enumerate(boxes):
                        x1, y1, x2, y2 = map(int, box)
                        confidence = confidences[i] if i < len(confidences) else 0.0

                        # Draw bounding box
                        cv2.rectangle(viz_frame, (x1, y1), (x2, y2), (255, 255, 255), 2)

                        # Add confidence text
                        text = f"Segment {i}: {confidence:.2f}"
                        text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0]
                        cv2.rectangle(
                            viz_frame,
                            (x1, y1 - text_size[1] - 5),
                            (x1 + text_size[0], y1),
                            (0, 0, 0),
                            -1,
                        )
                        cv2.putText(
                            viz_frame,
                            text,
                            (x1, y1 - 5),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (255, 255, 255),
                            1,
                        )

                # Create segment data
                segments = []
                for i in range(len(masks)):
                    segment_data = {
                        "segment_id": i,
                        "mask": masks[i],
                        "bbox": boxes[i].tolist() if i < len(boxes) else None,
                        "confidence": float(confidences[i]) if i < len(confidences) else None,
                        "area": float(np.sum(masks[i] > 0.5)),
                    }
                    segments.append(segment_data)

            else:
                # No segments found
                viz_frame = frame.copy()
                segments = []
                masks = []

            return {
                "frame": frame,
                "viz_frame": viz_frame,
                "segments": segments,
                "masks": masks,
                "processor": self.name,
                "metadata": metadata or {},
            }

        except Exception as e:
            logger.error(f"Error processing frame in semantic segmentation: {e}")
            return {
                "frame": frame,
                "viz_frame": frame,
                "segments": [],
                "masks": [],
                "error": str(e),
                "processor": self.name,
                "metadata": metadata or {},
            }

    def cleanup(self) -> None:
        """Clean up resources."""
        self.model = None
