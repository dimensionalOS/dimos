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

"""Object tracking stream plugin implementation."""

from typing import Dict, Any, Optional
from reactivex import Observable
import logging

from ..stream_interface import StreamInterface, StreamConfig

logger = logging.getLogger(__name__)


class ObjectTrackingPlugin(StreamInterface):
    """Plugin wrapper for ObjectTrackingStream.

    This plugin provides object tracking capabilities using OpenCV CSRT tracker
    with optional depth estimation using Metric3D.
    """

    def __init__(self, config: StreamConfig):
        """Initialize the object tracking plugin.

        Args:
            config: StreamConfig with the following parameters:
                - camera_intrinsics: List [fx, fy, cx, cy]
                - camera_pitch: Camera pitch angle in radians
                - camera_height: Camera height in meters
                - reid_threshold: Min matches for re-identification (default: 5)
                - reid_fail_tolerance: Consecutive failures before stopping (default: 10)
                - gt_depth_scale: Depth scale factor (default: 1000.0)
                - use_depth_model: Whether to use Metric3D for depth (default: True)
        """
        super().__init__(config)
        self.tracker = None

    def initialize(self, dependencies: Dict[str, StreamInterface] = None) -> bool:
        """Initialize the object tracking stream.

        Args:
            dependencies: Not used for this stream

        Returns:
            bool: True if initialization successful
        """
        try:
            # Get parameters from config
            camera_intrinsics = self.config.parameters.get("camera_intrinsics")
            camera_pitch = self.config.parameters.get("camera_pitch", 0.0)
            camera_height = self.config.parameters.get("camera_height", 1.0)
            reid_threshold = self.config.parameters.get("reid_threshold", 5)
            reid_fail_tolerance = self.config.parameters.get("reid_fail_tolerance", 10)
            gt_depth_scale = self.config.parameters.get("gt_depth_scale", 1000.0)
            use_depth_model = self.config.parameters.get("use_depth_model", True)

            # Check if we should use depth model based on device availability
            if use_depth_model and self.config.device == "cpu":
                logger.warning(
                    "Metric3D depth estimation may be slow on CPU. Consider disabling with use_depth_model=False"
                )

            # Import here to avoid loading models at module import time
            if use_depth_model:
                # Full ObjectTrackingStream with Metric3D
                from dimos.perception.object_tracker import ObjectTrackingStream

                self.tracker = ObjectTrackingStream(
                    camera_intrinsics=camera_intrinsics,
                    camera_pitch=camera_pitch,
                    camera_height=camera_height,
                    reid_threshold=reid_threshold,
                    reid_fail_tolerance=reid_fail_tolerance,
                    gt_depth_scale=gt_depth_scale,
                )
            else:
                # Create a lightweight version without Metric3D
                logger.info("Creating lightweight object tracker without depth estimation")
                from dimos.perception.object_tracker_lite import ObjectTrackingStreamLite

                self.tracker = ObjectTrackingStreamLite(
                    camera_intrinsics=camera_intrinsics,
                    camera_pitch=camera_pitch,
                    camera_height=camera_height,
                    reid_threshold=reid_threshold,
                    reid_fail_tolerance=reid_fail_tolerance,
                )

            self._initialized = True
            logger.info(f"Object tracking plugin initialized on {self.config.device}")
            return True

        except ImportError as e:
            logger.error(f"Failed to import object tracking dependencies: {e}")
            if "ObjectTrackingStreamLite" in str(e):
                logger.info("Creating lightweight tracker as fallback...")
                # We'll need to create this lightweight version
                return self._create_lite_tracker()
            return False
        except Exception as e:
            logger.error(f"Failed to initialize object tracking: {e}")
            return False

    def _create_lite_tracker(self) -> bool:
        """Create a lightweight tracker without heavy model dependencies."""
        try:
            # For now, we'll create a simple passthrough that doesn't do tracking
            # This ensures the robot can still run without GPU
            logger.info("Creating passthrough object tracker (no tracking)")

            class PassthroughTracker:
                def create_stream(self, input_stream):
                    def passthrough(frame):
                        return {"frame": frame, "viz_frame": frame, "targets": []}

                    return input_stream.pipe(ops.map(passthrough))

                def cleanup(self):
                    pass

            from reactivex import operators as ops

            self.tracker = PassthroughTracker()
            self._initialized = True
            return True

        except Exception as e:
            logger.error(f"Failed to create passthrough tracker: {e}")
            return False

    def create_stream(self, input_stream: Observable) -> Observable:
        """Create the object tracking stream.

        Args:
            input_stream: Observable that emits video frames

        Returns:
            Observable that emits object tracking results
        """
        if not self._initialized or self.tracker is None:
            raise RuntimeError("Object tracking plugin not initialized")

        return self.tracker.create_stream(input_stream)

    def cleanup(self):
        """Clean up resources."""
        if self.tracker is not None:
            self.tracker.cleanup()
            self.tracker = None
        self._initialized = False
