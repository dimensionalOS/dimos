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

from dataclasses import dataclass, field
from functools import cache
import threading
import time
from typing import Literal

import cv2
from dimos_lcm.sensor_msgs import CameraInfo  # type: ignore[import-untyped]
from reactivex import create
from reactivex.observable import Observable

from dimos.hardware.sensors.camera.spec import CameraConfig, CameraHardware
from dimos.msgs.sensor_msgs import Image
from dimos.msgs.sensor_msgs.Image import ImageFormat
from dimos.utils.reactive import backpressure


@dataclass
class WebcamConfig(CameraConfig):
    camera_index: int = 0  # /dev/videoN
    frame_width: int = 640
    frame_height: int = 480
    frequency: int = 15
    camera_info: CameraInfo = field(default_factory=CameraInfo)
    frame_id_prefix: str | None = None
    stereo_slice: Literal["left", "right"] | None = None  # For stereo cameras


class Webcam(CameraHardware[WebcamConfig]):
    default_config = WebcamConfig

    def __init__(self, *args, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(*args, **kwargs)
        self._capture = None
        self._capture_thread = None
        self._stop_event = threading.Event()
        self._observer = None

    @cache
    def image_stream(self) -> Observable[Image]:
        """Create an observable that starts/stops camera on subscription"""

        def subscribe(observer, scheduler=None):  # type: ignore[no-untyped-def]
            # Store the observer so emit() can use it
            self._observer = observer

            # Start the camera when someone subscribes
            try:
                self.start()  # type: ignore[no-untyped-call]
            except Exception as e:
                observer.on_error(e)
                return

            # Return a dispose function to stop camera when unsubscribed
            def dispose() -> None:
                self._observer = None
                self.stop()

            return dispose

        return backpressure(create(subscribe))

    def start(self):  # type: ignore[no-untyped-def]
        if self._capture_thread and self._capture_thread.is_alive():
            return

        # Open the video capture
        self._capture = cv2.VideoCapture(self.config.camera_index)  # type: ignore[assignment]
        if not self._capture.isOpened():  # type: ignore[attr-defined]
            raise RuntimeError(f"Failed to open camera {self.config.camera_index}")

        # Set camera properties
        self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.frame_width)  # type: ignore[attr-defined]
        self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.frame_height)  # type: ignore[attr-defined]

        # Clear stop event and start the capture thread
        self._stop_event.clear()
        self._capture_thread = threading.Thread(target=self._capture_loop, daemon=True)  # type: ignore[assignment]
        self._capture_thread.start()  # type: ignore[attr-defined]

    def stop(self) -> None:
        """Stop capturing frames"""
        # Signal thread to stop
        self._stop_event.set()

        # Wait for thread to finish
        if self._capture_thread and self._capture_thread.is_alive():
            self._capture_thread.join(timeout=(1.0 / self.config.frequency) + 0.1)

        # Release the capture
        if self._capture:
            self._capture.release()
            self._capture = None

    def _frame(self, frame: str):  # type: ignore[no-untyped-def]
        if not self.config.frame_id_prefix:
            return frame
        else:
            return f"{self.config.frame_id_prefix}/{frame}"

    def capture_frame(self) -> Image:
        # Read frame
        ret, frame = self._capture.read()  # type: ignore[attr-defined]
        if not ret:
            raise RuntimeError(f"Failed to read frame from camera {self.config.camera_index}")

        # Convert BGR to RGB (OpenCV uses BGR by default)
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Create Image message
        # Using Image.from_numpy() since it's designed for numpy arrays
        # Setting format to RGB since we converted from BGR->RGB above
        image = Image.from_numpy(
            frame_rgb,
            format=ImageFormat.RGB,  # We converted to RGB above
            frame_id=self._frame("camera_optical"),  # Standard frame ID for camera images
            ts=time.time(),  # Current timestamp
        )

        if self.config.stereo_slice in ("left", "right"):
            half_width = image.width // 2
            if self.config.stereo_slice == "left":
                image = image.crop(0, 0, half_width, image.height)
            else:
                image = image.crop(half_width, 0, half_width, image.height)

        return image

    def _capture_loop(self) -> None:
        """Capture frames at the configured frequency"""
        frame_interval = 1.0 / self.config.frequency
        next_frame_time = time.time()

        while self._capture and not self._stop_event.is_set():
            image = self.capture_frame()

            # Emit the image to the observer only if not stopping
            if self._observer and not self._stop_event.is_set():
                self._observer.on_next(image)

            # Wait for next frame time or until stopped
            next_frame_time += frame_interval
            sleep_time = next_frame_time - time.time()
            if sleep_time > 0:
                # Use event.wait so we can be interrupted by stop
                if self._stop_event.wait(timeout=sleep_time):
                    break  # Stop was requested
            else:
                # We're running behind, reset timing
                next_frame_time = time.time()

    @property
    def camera_info(self) -> CameraInfo:
        return self.config.camera_info

    def emit(self, image: Image) -> None: ...
