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
from streamz import Stream

from dimos.multiprocess.types import Frame

logger = logging.getLogger(__name__)


class Detection(TypedDict):
    x: int  # Top-left x coordinate of bounding box
    y: int  # Top-left y coordinate of bounding box
    w: int  # Width of bounding box
    h: int  # Height of bounding box
    confidence: float  # Detection confidence (0.0 to 1.0)


class RecognitionFrame(Frame):
    detections: List[Detection]  # List of detected objects/faces


class FaceRecognitionActor:
    """Simple face detection actor using OpenCV Haar cascades."""

    def __init__(self, name="FaceRecognition", verbose=False, min_neighbors=5, scale_factor=1.1):
        """
        Initialize the face recognition actor.

        Args:
            name: Actor name for logging
            verbose: Whether to print detection info
            min_neighbors: Minimum neighbors required for detection (higher = fewer false positives)
            scale_factor: How much the image size is reduced at each scale (closer to 1.0 = more thorough)
        """
        self.name = name
        self.verbose = verbose
        self.min_neighbors = min_neighbors
        self.scale_factor = scale_factor

        # Initialize the face cascade classifier
        self.face_cascade = cv2.CascadeClassifier(
            cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
        )
        if self.face_cascade.empty():
            raise RuntimeError("Failed to load face cascade classifier")

        # Create output stream - this is where processors will be connected
        self.stream = Stream(asynchronous=True)
        self.has_processors = False

        logger.info(f"FaceRecognitionActor '{name}' initialized")

    def _detect_faces(self, frame: Frame) -> RecognitionFrame:
        """Detect faces in the frame and return frame with detections."""
        # Convert to grayscale for face detection
        gray = cv2.cvtColor(frame["frame"], cv2.COLOR_BGR2GRAY)

        # Detect faces
        faces = self.face_cascade.detectMultiScale(
            gray,
            scaleFactor=self.scale_factor,
            minNeighbors=self.min_neighbors,
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

        if self.verbose:
            print(f"{self.name}: Frame {frame['frame_number']} - {len(detections)} faces detected")

        return recognition_frame

    def add_processor(self, processor):
        """Add a processor to receive recognition frames."""
        self.stream.sink(processor.receive_frame)
        self.has_processors = True
        logger.info(f"Added processor to {self.name}")

    def add_processors(self, *processors):
        """Add multiple processors to receive recognition frames."""
        for processor in processors:
            self.add_processor(processor)

    async def receive_frame(self, frame: Frame) -> None:
        """Receive a frame from upstream (e.g., camera actor)."""
        # Only process if we have processors registered
        if not self.has_processors:
            if self.verbose:
                logger.info(
                    f"{self.name}: No processors registered, skipping frame {frame['frame_number']}"
                )
            return

        # Process the frame and emit recognition results
        recognition_frame = self._detect_faces(frame)
        await self.stream.emit(recognition_frame)
