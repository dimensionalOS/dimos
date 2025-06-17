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

import logging
import time
from typing import TypedDict

import cv2
import numpy as np
from dask.distributed import get_client
from streamz import Stream

logger = logging.getLogger(__name__)


class VideoFrame(TypedDict):
    frame: np.ndarray  # The actual image data from cv2
    timestamp: float  # Unix timestamp when frame was captured
    frame_number: int  # Sequential frame number


class CameraActor:
    stream: Stream = Stream(asynchronous=True)

    def __init__(self, camera_index=None, width=640, height=480):
        """
        Initialize the camera loop.

        Args:
            camera_index: Camera device index (None for auto-select first working camera)
            width: Frame width in pixels
            height: Frame height in pixels
        """
        self.client = get_client()
        self.camera_index = camera_index

        self.width = width
        self.height = height
        self.cap = None
        self.frame_count = 0

    def _initialize_camera(self):
        """Initialize the camera capture."""
        if self.cap is None or not self.cap.isOpened():
            if self.cap:
                self.cap.release()

            self.cap = cv2.VideoCapture(self.camera_index)
            if not self.cap.isOpened():
                raise RuntimeError(f"Failed to open camera {self.camera_index}")

            # Set camera properties
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

            # Get actual properties (camera might not support exact values)
            actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

            logger.info(f"Camera initialized: {actual_width}x{actual_height}")

    def add_processor(self, processor):
        """Add a processor to receive camera frames."""
        self.stream.sink(processor.receive_frame)

    def add_processors(self, *processors):
        """Add multiple processors to receive camera frames."""
        for processor in processors:
            self.add_processor(processor)

    async def run(self, total_frames=None):
        """
        Run the camera loop to capture and emit frames.

        Args:
            total_frames: Maximum number of frames to capture (None for infinite)
        """

        self._initialize_camera()

        start_time = time.time()

        while True:
            # Capture frame
            ret, frame = self.cap.read()
            if not ret:
                logger.error("Failed to capture frame from camera")
                break

            # Create frame data with timestamp and frame number
            frame_data: VideoFrame = {
                "frame": frame,
                "timestamp": time.time(),
                "frame_number": self.frame_count,
            }

            # Emit the frame
            print("CameraActor emitting frame", self.frame_count)
            await self.stream.emit(frame_data)
            self.frame_count += 1

            # Check if we've reached the frame limit
            if total_frames is not None and self.frame_count >= total_frames:
                break

        total_time = time.time() - start_time
        avg_fps = self.frame_count / total_time if total_time > 0 else 0
        logger.info(
            f"Camera loop completed: {self.frame_count} frames in {total_time:.2f}s (avg {avg_fps:.1f} FPS)"
        )

    def cleanup(self):
        """Clean up camera resources."""
        if self.cap and self.cap.isOpened():
            self.cap.release()
            logger.info("Camera released")

    def __del__(self):
        """Destructor to ensure camera is released."""
        self.cleanup()
