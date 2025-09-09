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

import logging
import os
import sys
import time

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Import the GStreamer camera module
from dimos.hardware.gstreamer_camera import GstreamerCameraModule
from dimos import core
from dimos.protocol import pubsub
from dimos.msgs.sensor_msgs import Image


def main():
    """Test the GStreamer camera module."""

    # Initialize LCM
    pubsub.lcm.autoconf()

    # Start dimos
    logger.info("Starting dimos...")
    dimos = core.start(8)

    # Get network interface from environment or use default
    multicast_iface = os.getenv("MULTICAST_IFACE", "enp109s0")

    # Deploy the GStreamer camera module
    logger.info(f"Deploying GStreamer camera module (interface: {multicast_iface})...")
    camera = dimos.deploy(
        GstreamerCameraModule,
        multicast_group="224.0.0.1",
        port=5000,
        multicast_iface=multicast_iface,
        frame_id="zed_camera",
    )

    # Set up LCM transport for the video output
    camera.video.transport = core.LCMTransport("/zed/video", Image)

    # Counter for received frames
    frame_count = [0]
    last_log_time = [time.time()]

    def on_frame(msg):
        """Callback for received frames."""
        frame_count[0] += 1
        current_time = time.time()

        # Log stats every 2 seconds
        if current_time - last_log_time[0] >= 2.0:
            fps = frame_count[0] / (current_time - last_log_time[0])
            logger.info(
                f"Received {frame_count[0]} frames - FPS: {fps:.1f} - Resolution: {msg.width}x{msg.height}"
            )
            frame_count[0] = 0
            last_log_time[0] = current_time

    # Subscribe to video output for monitoring
    camera.video.subscribe(on_frame)

    # Start the camera
    logger.info("Starting GStreamer camera...")
    camera.start()

    logger.info("GStreamer camera module is running. Press Ctrl+C to stop.")
    logger.info(
        f"Listening for video on multicast group 224.0.0.1:{5000} on interface {multicast_iface}"
    )
    logger.info("Publishing frames to LCM topic: /zed/video")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        logger.info("Shutting down...")
        camera.stop()
        logger.info("Stopped.")


if __name__ == "__main__":
    main()
