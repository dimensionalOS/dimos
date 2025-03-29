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

import cv2

class VideoStream:
    def __init__(self, source=0):
        """
        Initialize the video stream from a camera source.
        
        Args:
            source (int or str): Camera index or video file path.
        """
        self.capture = cv2.VideoCapture(source)
        if not self.capture.isOpened():
            raise ValueError(f"Unable to open video source {source}")

    def __iter__(self):
        return self

    def __next__(self):
        ret, frame = self.capture.read()
        if not ret:
            self.capture.release()
            raise StopIteration
        return frame

    def release(self):
        self.capture.release()
