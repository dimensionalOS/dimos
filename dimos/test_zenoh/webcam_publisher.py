# Copyright 2025-2026 Dimensional Inc.
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

"""Webcam publisher module for Zenoh transport demonstration."""

from __future__ import annotations

import threading
import time
from typing import TYPE_CHECKING

import cv2
import numpy as np

from dimos.core import Module, Out, rpc
from dimos.msgs.sensor_msgs import Image, ImageFormat
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class WebcamPublisher(Module):
    """Webcam publisher that captures images from a webcam and publishes via Zenoh.

    Publishes:
        - color_image: RGB images from webcam

    Example usage with Zenoh transport:
        from dimos.core.transport import ZenohTransport

        webcam = dimos.deploy(WebcamPublisher)
        webcam.color_image.transport = ZenohTransport("camera/color")
        webcam.start()
    """

    color_image: Out[Image]

    def __init__(
        self,
        camera_id: int = 0,
        target_fps: float = 30.0,
        width: int = 640,
        height: int = 480,
        **kwargs,
    ) -> None:
        """Initialize webcam publisher.

        Args:
            camera_id: Camera device ID (default: 0 for first webcam)
            target_fps: Target frames per second
            width: Frame width
            height: Frame height
        """
        super().__init__(**kwargs)
        self.camera_id = camera_id
        self.target_fps = target_fps
        self.width = width
        self.height = height

        self._cap: cv2.VideoCapture | None = None
        self._running = False
        self._capture_thread: threading.Thread | None = None
        self._stop_event = threading.Event()

    @rpc
    def start(self) -> None:
        """Start capturing and publishing webcam images."""
        super().start()

        if self._running:
            logger.warning("Webcam publisher already running")
            return

        self._cap = cv2.VideoCapture(self.camera_id)
        if not self._cap.isOpened():
            raise RuntimeError(f"Failed to open camera {self.camera_id}")

        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self._cap.set(cv2.CAP_PROP_FPS, self.target_fps)

        self._running = True
        self._stop_event.clear()

        self._capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._capture_thread.start()

        logger.info(
            f"Webcam publisher started (camera={self.camera_id}, "
            f"{self.width}x{self.height} @ {self.target_fps}fps)"
        )

    @rpc
    def stop(self) -> None:
        """Stop capturing and release webcam."""
        if not self._running:
            return

        self._running = False
        self._stop_event.set()

        if self._capture_thread and self._capture_thread.is_alive():
            self._capture_thread.join(timeout=2.0)

        if self._cap is not None:
            self._cap.release()
            self._cap = None

        super().stop()
        logger.info("Webcam publisher stopped")

    def _capture_loop(self) -> None:
        """Main capture loop running in a separate thread."""
        frame_interval = 1.0 / self.target_fps

        while not self._stop_event.is_set():
            loop_start = time.time()

            if self._cap is None or not self._cap.isOpened():
                break

            ret, frame = self._cap.read()
            if not ret:
                logger.warning("Failed to read frame from webcam")
                time.sleep(0.01)
                continue

            # Convert BGR to RGB
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            # Create Image message
            img_msg = Image(
                data=frame_rgb,
                format=ImageFormat.RGB,
                frame_id="webcam",
                ts=time.time(),
            )

            # Publish
            self.color_image.publish(img_msg)
            logger.debug(f"Published frame: shape={frame_rgb.shape}")

            # Maintain target FPS
            elapsed = time.time() - loop_start
            sleep_time = frame_interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

        logger.info("Capture loop ended")


webcam_publisher = WebcamPublisher.blueprint

__all__ = ["WebcamPublisher", "webcam_publisher"]
