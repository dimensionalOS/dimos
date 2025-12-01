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
from streamz.dask import Stream

# from streamz import Stream
from dimos.utils.testing import _get_data_dir

logger = logging.getLogger(__name__)


class VideoFrame(TypedDict):
    frame: np.ndarray  # The actual image data from cv2
    timestamp: float  # Unix timestamp when frame was captured
    frame_number: int  # Sequential frame number


class VideoActor:
    stream: Stream = Stream(asynchronous=True)

    def __init__(self, video_path=None, width=None, height=None):
        """
        Initialize the video player.

        Args:
            video_path: Path to video file (defaults to office.mp4 in data dir)
            width: Frame width in pixels (None to use original video dimensions)
            height: Frame height in pixels (None to use original video dimensions)
        """
        self.client = get_client()
        self.video_path = video_path or str(_get_data_dir() / "video" / "office.mp4")

        self.width = width
        self.height = height
        self.cap = None
        self.frame_count = 0
        self.total_video_frames = 0

    def _initialize_video(self):
        """Initialize the video capture."""
        if self.cap is None or not self.cap.isOpened():
            if self.cap:
                self.cap.release()

            self.cap = cv2.VideoCapture(self.video_path)
            if not self.cap.isOpened():
                raise RuntimeError(f"Failed to open video file {self.video_path}")

            # Get video properties
            self.total_video_frames = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
            actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            fps = self.cap.get(cv2.CAP_PROP_FPS)

            # Set resize dimensions if specified
            if self.width is not None and self.height is not None:
                logger.info(
                    f"Video will be resized from {actual_width}x{actual_height} to {self.width}x{self.height}"
                )
            else:
                self.width = actual_width
                self.height = actual_height

            logger.info(f"Video initialized: {self.video_path}")
            logger.info(
                f"Dimensions: {actual_width}x{actual_height}, FPS: {fps:.1f}, Total frames: {self.total_video_frames}"
            )

    def add_processor(self, processor):
        """Add a processor to receive video frames."""
        self.stream.sink(processor.receive_frame)

    def add_processors(self, *processors):
        """Add multiple processors to receive video frames."""
        for processor in processors:
            self.add_processor(processor)

    async def run(self, total_frames=None, loop=False):
        """
        Run the video playback loop to emit frames.

        Args:
            total_frames: Maximum number of frames to emit (None for all video frames)
            loop: Whether to loop the video when it reaches the end
        """
        self._initialize_video()

        start_time = time.time()

        while True:
            # Capture frame
            ret, frame = self.cap.read()
            if not ret:
                if loop:
                    # Reset video to beginning
                    self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    ret, frame = self.cap.read()
                    if not ret:
                        logger.error("Failed to restart video from beginning")
                        break
                else:
                    logger.info("Reached end of video")
                    break

            # Resize frame if dimensions specified
            if frame.shape[:2] != (self.height, self.width):
                frame = cv2.resize(frame, (self.width, self.height))

            # Create frame data with timestamp and frame number
            frame_data: VideoFrame = {
                "frame": frame,
                "timestamp": time.time(),
                "frame_number": self.frame_count,
            }

            # Emit the frame
            # print("VideoActor emitting frame", self.frame_count)
            await self.stream.emit(frame_data)
            self.frame_count += 1

            # Check if we've reached the frame limit
            if total_frames is not None and self.frame_count >= total_frames:
                break

        total_time = time.time() - start_time
        avg_fps = self.frame_count / total_time if total_time > 0 else 0
        logger.info(
            f"Video playback completed: {self.frame_count} frames in {total_time:.2f}s (avg {avg_fps:.1f} FPS)"
        )

    def cleanup(self):
        """Clean up video resources."""
        if self.cap and self.cap.isOpened():
            self.cap.release()
            logger.info("Video capture released")

    def __del__(self):
        """Destructor to ensure video capture is released."""
        self.cleanup()
