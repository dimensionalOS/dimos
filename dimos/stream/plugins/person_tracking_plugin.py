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

"""Person tracking stream plugin implementation."""

from typing import Dict, Any, Optional
from reactivex import Observable
import logging

from ..stream_interface import StreamInterface, StreamConfig

logger = logging.getLogger(__name__)


class PersonTrackingPlugin(StreamInterface):
    """Plugin wrapper for PersonTrackingStream.

    This plugin provides person detection and tracking capabilities using YOLO.
    It can be configured to run on CPU or GPU.
    """

    def __init__(self, config: StreamConfig):
        """Initialize the person tracking plugin.

        Args:
            config: StreamConfig with the following parameters:
                - model_path: Path to YOLO model (default: "yolo11n.pt")
                - camera_intrinsics: List [fx, fy, cx, cy]
                - camera_pitch: Camera pitch angle in radians
                - camera_height: Camera height in meters
        """
        super().__init__(config)
        self.tracker = None

    def initialize(self, dependencies: Dict[str, StreamInterface] = None) -> bool:
        """Initialize the person tracking stream.

        Args:
            dependencies: Not used for this stream

        Returns:
            bool: True if initialization successful
        """
        try:
            # Import here to avoid loading models at module import time
            from dimos.perception.person_tracker import PersonTrackingStream

            # Get parameters from config
            model_path = self.config.parameters.get("model_path", "yolo11n.pt")
            camera_intrinsics = self.config.parameters.get("camera_intrinsics")
            camera_pitch = self.config.parameters.get("camera_pitch", 0.0)
            camera_height = self.config.parameters.get("camera_height", 1.0)

            if camera_intrinsics is None:
                logger.error("camera_intrinsics parameter is required")
                return False

            # Create the tracker
            self.tracker = PersonTrackingStream(
                model_path=model_path,
                device=self.config.device,
                camera_intrinsics=camera_intrinsics,
                camera_pitch=camera_pitch,
                camera_height=camera_height,
            )

            self._initialized = True
            logger.info(f"Person tracking plugin initialized on {self.config.device}")
            return True

        except ImportError as e:
            logger.error(f"Failed to import PersonTrackingStream: {e}")
            logger.info("PersonTrackingStream requires YOLO. Install with: pip install ultralytics")
            return False
        except Exception as e:
            logger.error(f"Failed to initialize person tracking: {e}")
            return False

    def create_stream(self, input_stream: Observable) -> Observable:
        """Create the person tracking stream.

        Args:
            input_stream: Observable that emits video frames

        Returns:
            Observable that emits person tracking results
        """
        if not self._initialized or self.tracker is None:
            raise RuntimeError("Person tracking plugin not initialized")

        return self.tracker.create_stream(input_stream)

    def cleanup(self):
        """Clean up resources."""
        if self.tracker is not None:
            self.tracker.cleanup()
            self.tracker = None
        self._initialized = False
