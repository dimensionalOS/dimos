#!/usr/bin/env python3
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

import time

import cv2

from dimos.multiprocess.actor import Actor


class CameraAgent(Actor):
    def __init__(self, device="/dev/video0", fps=30, queue=None):
        self.queue = queue
        self.cap = cv2.VideoCapture(device)
        self.sleep = 1.0 / fps
        self._run = True
        super().__init__()

    def loop(self):
        """Blocking loop that pushes frames to the queue."""
        if not self.cap.isOpened():
            print("Warning: Camera not available, using dummy frames")
            import numpy as np

            while self._run:
                # Create a dummy frame if camera is not available
                dummy_frame = np.zeros((480, 640, 3), dtype=np.uint8)
                if self.queue:
                    self.queue.put(dummy_frame)
                time.sleep(self.sleep)
        else:
            while self._run:
                ok, frame = self.cap.read()
                if ok and self.queue:
                    self.queue.put(frame)
                time.sleep(self.sleep)

    def stop(self):
        self._run = False
