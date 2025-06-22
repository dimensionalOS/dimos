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

import cv2
import numpy as np

from dimos.multiprocess.actors2.meta import In, Out, module, rpc
from dimos.utils.testing import testData

logger = logging.getLogger(__name__)


@module
class Video:
    video_stream: Out[np.ndarray]
    width: int
    height: int
    total_frames: int

    def __init__(self, video_name="office.mp4"):
        self.video_name = video_name
        self.cap = None

    @rpc
    def get_video_properties(self) -> dict:
        if self.cap is None or not self.cap.isOpened():
            raise RuntimeError("Video capture is not initialized. Call play() first.")

        return {
            "name": self.video_name,
            "width": self.width,
            "height": self.height,
            "total_frames": self.total_frames,
        }

    @rpc
    def play(self, frames: int) -> bool:
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
