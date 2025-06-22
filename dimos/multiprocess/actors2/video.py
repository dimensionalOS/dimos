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
from reactivex.subject import Subject

from dimos.multiprocess.actors2.meta import Out, module, rpc
from dimos.utils.testing import testData

logger = logging.getLogger(__name__)


class VideoFrame(TypedDict):
    frame: np.ndarray  # The actual image data from cv2
    timestamp: float  # Unix timestamp when frame was captured
    frame_number: int  # Sequential frame number


@module
class Video:
    video_stream: Out[VideoFrame]
    width: int
    height: int
    total_frames: int

    def __init__(self, video_name="office.mp4"):
        self.video_name = video_name
        self.video_stream = Subject()

        self.cap = None

    @rpc
    async def get_video_properties(self) -> dict:
        if self.cap is None or not self.cap.isOpened():
            raise RuntimeError("Video capture is not initialized. Call play() first.")

        return {
            "name": self.video_name,
            "width": self.width,
            "height": self.height,
            "total_frames": self.total_frames,
        }

    @rpc
    async def stop(self) -> bool: ...

    @rpc
    async def play(self, target_frames: int | None) -> bool:
        self.video_path = testData("video").joinpath(self.video_name)

        if self.cap is None or not self.cap.isOpened():
            if self.cap:
                self.cap.release()

        self.cap = cv2.VideoCapture(self.video_path)
        if not self.cap.isOpened():
            raise RuntimeError(f"Failed to open video file {self.video_path}")

        # Get video properties
        self.total_frames = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = self.cap.get(cv2.CAP_PROP_FPS)

        logger.info(f"Video initialized: {self.video_path}")
        logger.info(
            f"Dimensions: {self.width}x{self.height}, FPS: {fps:.1f}, Total frames: {self.total_frames}"
        )

        start_time = time.time()

        frame_count = 0
        while True:
            ret, frame = self.cap.read()
            if not ret:
                logger.info("Reached end of video")
                break

            frame_data: VideoFrame = {
                "frame": frame,
                "timestamp": time.time(),
                "frame_number": frame_count,
            }

            self.video_stream.on_next(frame_data)
            frame_count += 1

            if target_frames is not None and frame_count >= target_frames:
                break

        total_time = time.time() - start_time
        avg_fps = frame_count / total_time if total_time > 0 else 0
        logger.info(
            f"Video playback completed: {frame_count} frames in {total_time:.2f}s (avg {avg_fps:.1f} FPS)"
        )
