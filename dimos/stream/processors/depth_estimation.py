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

"""Depth estimation stream processor.

This module provides a modular depth estimation processor that can be optionally
loaded if Metric3D dependencies are available.
"""

from typing import Any, Dict, Optional
import numpy as np
import cv2
import logging

from ..stream_processor import StreamProcessor
from dimos.models.depth.metric3d import Metric3D

logger = logging.getLogger(__name__)


class DepthEstimationProcessor(StreamProcessor):
    """Stream processor for depth estimation using Metric3D."""

    def __init__(self, name: str, config: Optional[Dict[str, Any]] = None):
        """Initialize the depth estimation processor.

        Args:
            name: Unique name for this processor
            config: Configuration dictionary with the following keys:
                - gt_depth_scale: Ground truth depth scale factor (default: 1000.0)
                - camera_intrinsics: Camera intrinsics [fx, fy, cx, cy] (optional)
                - device: Device for depth model (default: "cpu")
                - colormap: OpenCV colormap for visualization (default: cv2.COLORMAP_PLASMA)
        """
        super().__init__(name, config)

        # Set default configuration
        self.gt_depth_scale = self.config.get("gt_depth_scale", 1000.0)
        self.camera_intrinsics = self.config.get("camera_intrinsics")
        self.device = self.config.get("device", "cpu")
        self.colormap = self.config.get("colormap", cv2.COLORMAP_PLASMA)

        self.depth_model = None

    def initialize(self) -> bool:
        """Initialize the depth estimation model."""
        try:
            # Initialize depth model
            self.depth_model = Metric3D(self.gt_depth_scale)

            # Update intrinsics if provided
            if self.camera_intrinsics is not None:
                if (
                    not isinstance(self.camera_intrinsics, (list, tuple, np.ndarray))
                    or len(self.camera_intrinsics) != 4
                ):
                    raise ValueError("Camera intrinsics must be provided as [fx, fy, cx, cy]")
                self.depth_model.update_intrinsic(self.camera_intrinsics)
                logger.info(f"Initialized depth estimation with camera intrinsics")
            else:
                logger.info(f"Initialized depth estimation without camera intrinsics")

            return True

        except Exception as e:
            logger.error(f"Failed to initialize depth estimation processor: {e}")
            return False

    def process_frame(
        self, frame: Any, metadata: Optional[Dict[str, Any]] = None
    ) -> Dict[str, Any]:
        """Process a frame for depth estimation.

        Args:
            frame: Input video frame
            metadata: Optional metadata

        Returns:
            Dict containing depth map and visualization
        """
        if self.depth_model is None:
            raise RuntimeError("Processor not initialized")

        try:
            # Estimate depth
            depth_map = self.depth_model.infer_depth(frame)
            depth_array = np.array(depth_map)

            # Convert depth to meters (from mm)
            depth_meters = depth_array / 1000.0

            # Create visualization
            # Normalize depth for visualization (0-10 meters range)
            depth_vis = np.clip(depth_meters, 0, 10) / 10.0
            depth_vis = (depth_vis * 255).astype(np.uint8)
            depth_colored = cv2.applyColorMap(depth_vis, self.colormap)

            # Create side-by-side visualization
            h, w = frame.shape[:2]
            viz_frame = np.zeros((h, w * 2, 3), dtype=np.uint8)
            viz_frame[:, :w] = frame
            viz_frame[:, w:] = depth_colored

            # Add labels
            cv2.putText(viz_frame, "RGB", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.putText(
                viz_frame, "Depth", (w + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2
            )

            return {
                "frame": frame,
                "viz_frame": viz_frame,
                "depth_map": depth_array,
                "depth_meters": depth_meters,
                "depth_colored": depth_colored,
                "processor": self.name,
                "metadata": metadata or {},
            }

        except Exception as e:
            logger.error(f"Error processing frame in depth estimation: {e}")
            return {
                "frame": frame,
                "viz_frame": frame,
                "depth_map": None,
                "error": str(e),
                "processor": self.name,
                "metadata": metadata or {},
            }

    def cleanup(self) -> None:
        """Clean up resources."""
        self.depth_model = None
