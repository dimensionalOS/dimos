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

"""Canny edge detection subscriber that streams to Rerun visualization."""

from __future__ import annotations

from typing import TYPE_CHECKING

import cv2
import numpy as np

from dimos.core import In, Module, rpc
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    import rerun as rr

    from dimos.msgs.sensor_msgs import Image

logger = setup_logger()


class CannySubscriber(Module):
    """Subscriber that applies Canny edge detection and streams to Rerun.

    Subscribes to:
        - color_image: RGB images from webcam

    Visualizes in Rerun:
        - Original image
        - Canny edge detection result

    Example usage with Zenoh transport:
        from dimos.core.transport import ZenohTransport

        canny = dimos.deploy(CannySubscriber)
        canny.color_image.transport = ZenohTransport("camera/color")
        canny.start()
    """

    color_image: In[Image]

    def __init__(
        self,
        canny_low: int = 50,
        canny_high: int = 150,
        spawn_viewer: bool = True,
        **kwargs,
    ) -> None:
        """Initialize Canny subscriber.

        Args:
            canny_low: Lower threshold for Canny edge detection
            canny_high: Upper threshold for Canny edge detection
            spawn_viewer: Whether to spawn the Rerun viewer window
        """
        super().__init__(**kwargs)
        self.canny_low = canny_low
        self.canny_high = canny_high
        self.spawn_viewer = spawn_viewer

        self._rr: rr | None = None
        self._running = False
        self._frame_count = 0

    @rpc
    def start(self) -> None:
        """Start the subscriber and initialize Rerun."""
        super().start()

        if self._running:
            logger.warning("Canny subscriber already running")
            return

        # Initialize Rerun
        try:
            import rerun as rr

            self._rr = rr
            rr.init("zenoh_canny_demo", spawn=self.spawn_viewer)
            logger.info("Rerun initialized")
        except ImportError:
            logger.error(
                "Rerun not installed. Install with: pip install rerun-sdk\nThen run: rerun"
            )
            raise

        self._running = True

        # Subscribe to color images
        self.color_image.subscribe(self._on_image)

        logger.info(f"Canny subscriber started (thresholds: {self.canny_low}-{self.canny_high})")

    @rpc
    def stop(self) -> None:
        """Stop the subscriber."""
        if not self._running:
            return

        self._running = False
        super().stop()
        logger.info("Canny subscriber stopped")

    def _on_image(self, msg: Image) -> None:
        """Process incoming image with Canny edge detection."""
        if not self._running or self._rr is None:
            return

        try:
            rr = self._rr
            self._frame_count += 1

            # Get image data
            img = msg.data
            if img is None:
                return

            # Log original image to Rerun
            rr.log("camera/original", msg.to_rerun())

            # Convert to grayscale for Canny
            if len(img.shape) == 3:
                gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
            else:
                gray = img

            # Apply Canny edge detection
            edges = cv2.Canny(gray, self.canny_low, self.canny_high)

            # Log edges to Rerun (as single channel image)
            rr.log("camera/canny_edges", rr.Image(edges))

            logger.debug(f"Processed frame {self._frame_count}")

        except Exception as e:
            logger.error(f"Error processing image: {e}")


canny_subscriber = CannySubscriber.blueprint

__all__ = ["CannySubscriber", "canny_subscriber"]
