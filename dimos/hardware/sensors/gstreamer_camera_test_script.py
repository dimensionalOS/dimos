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

import argparse
import logging
import time

from dimos import core
from dimos.hardware.sensors.gstreamer_camera import GstreamerCameraModule
from dimos.msgs.sensor_msgs import Image
from dimos.protocol import pubsub

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def main() -> None:
    parser = argparse.ArgumentParser(description="Test script for GStreamer TCP camera module")

    # Network options
    parser.add_argument(
        "--host", default="localhost", help="TCP server host to connect to (default: localhost)"
    )
    parser.add_argument("--port", type=int, default=5000, help="TCP server port (default: 5000)")

    # Camera options
    parser.add_argument(
        "--frame-id",
        default="zed_camera",
        help="Frame ID for published images (default: zed_camera)",
    )
    parser.add_argument(
        "--reconnect-interval",
        type=float,
        default=5.0,
        help="Seconds to wait before attempting reconnection (default: 5.0)",
    )

    # Logging options
    parser.add_argument("--verbose", action="store_true", help="Enable verbose logging")

    args = parser.parse_args()

    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)

    # Initialize LCM
    pubsub.lcm.autoconf()  # type: ignore[attr-defined]

    # Start dimos
    logger.info("Starting dimos...")
    dimos = core.start(8)

    # Deploy the GStreamer camera module
    logger.info(f"Deploying GStreamer TCP camera module (connecting to {args.host}:{args.port})...")
    camera = dimos.deploy(  # type: ignore[attr-defined]
        GstreamerCameraModule,
        host=args.host,
        port=args.port,
        frame_id=args.frame_id,
        reconnect_interval=args.reconnect_interval,
    )

    # Set up LCM transport for the video output
    camera.video.transport = core.LCMTransport("/zed/video", Image)

    # Counter for received frames
    frame_count = [0]
    last_log_time = [time.time()]
    first_timestamp = [None]

    def on_frame(msg) -> None:  # type: ignore[no-untyped-def]
        frame_count[0] += 1
        current_time = time.time()

        # Capture first timestamp to show absolute timestamps are preserved
        if first_timestamp[0] is None:
            first_timestamp[0] = msg.ts
            logger.info(f"First frame absolute timestamp: {msg.ts:.6f}")

        # Log stats every 2 seconds
        if current_time - last_log_time[0] >= 2.0:
            fps = frame_count[0] / (current_time - last_log_time[0])
            timestamp_delta = msg.ts - first_timestamp[0]
            logger.info(
                f"Received {frame_count[0]} frames - FPS: {fps:.1f} - "
                f"Resolution: {msg.width}x{msg.height} - "
                f"Timestamp: {msg.ts:.3f} (delta: {timestamp_delta:.3f}s)"
            )
            frame_count[0] = 0
            last_log_time[0] = current_time

    # Subscribe to video output for monitoring
    camera.video.subscribe(on_frame)

    # Start the camera
    logger.info("Starting GStreamer camera...")
    camera.start()

    logger.info("GStreamer TCP camera module is running. Press Ctrl+C to stop.")
    logger.info(f"Connecting to TCP server at {args.host}:{args.port}")
    logger.info("Publishing frames to LCM topic: /zed/video")
    logger.info("")
    logger.info("To start the sender on the camera machine, run:")
    logger.info(
        f"  python3 dimos/hardware/gstreamer_sender.py --device /dev/video0 --host 0.0.0.0 --port {args.port}"
    )

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        logger.info("Shutting down...")
        camera.stop()
        logger.info("Stopped.")


if __name__ == "__main__":
    main()
