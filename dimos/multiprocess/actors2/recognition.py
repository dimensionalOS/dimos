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
from typing import List, TypedDict

import cv2
from reactivex import operators as ops

from dimos.multiprocess.actors.env import getenv
from dimos.multiprocess.actors2.meta import In, Out, module, rpc
from dimos.multiprocess.actors2.video import VideoFrame
from dimos.multiprocess.types import Frame

logger = logging.getLogger(__name__)
from dimos.multiprocess.utils.testing import dask_client

print(dask_client)


class Detection(TypedDict):
    x: int  # Top-left x coordinate of bounding box
    y: int  # Top-left y coordinate of bounding box
    w: int  # Width of bounding box
    h: int  # Height of bounding box
    confidence: float  # Detection confidence (0.0 to 1.0)


class RecognitionFrame(VideoFrame):
    detections: List[Detection]  # List of detected objects/faces


@module
class Recognition:
    output_stream: Out[RecognitionFrame]

    def __init__(self, input_stream: In[VideoFrame]):
        input_stream.pipe(ops.map(self.detect_faces)).subscribe(self.output_stream.emit)

    @rpc
    def detect_faces(self, frame: Frame) -> RecognitionFrame:
        face_cascade = getenv(
            "face_cascade",
            lambda: cv2.CascadeClassifier(
                cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
            ),
        )

        print("got", face_cascade)
        # Convert to grayscale for face detection
        gray = cv2.cvtColor(frame["frame"], cv2.COLOR_BGR2GRAY)

        # Detect faces
        faces = face_cascade.detectMultiScale(
            gray,
            scaleFactor=1.1,
            minNeighbors=5,
            minSize=(30, 30),  # Minimum face size
        )

        # Convert to our Detection format
        detections: List[Detection] = []
        for x, y, w, h in faces:
            detection: Detection = {
                "x": int(x),
                "y": int(y),
                "w": int(w),
                "h": int(h),
                "confidence": 1.0,  # Haar cascades don't provide confidence scores
            }
            detections.append(detection)

        # Create recognition frame
        recognition_frame: RecognitionFrame = {
            "frame": frame["frame"],
            "timestamp": frame["timestamp"],
            "frame_number": frame["frame_number"],
            "detections": detections,
        }
        print("returning frame", recognition_frame["frame_number"])
        return recognition_frame
