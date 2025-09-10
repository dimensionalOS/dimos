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
import time
import argparse

from dimos.hardware.tcp_camera import TCPCameraModule
from dimos import core
from dimos.protocol import pubsub
from dimos.msgs.sensor_msgs import Image

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def main():
    """Test the TCP camera module with absolute timestamps."""

    parser = argparse.ArgumentParser(description="Test TCP camera module with absolute timestamps")
    parser.add_argument("--host", default="localhost", help="TCP server host (default: localhost)")
    parser.add_argument("--port", type=int, default=5555, help="TCP server port (default: 5555)")
    parser.add_argument(
        "--frame-id",
        default="zed_camera",
        help="Frame ID for published images (default: zed_camera)",
    )
    parser.add_argument(
        "--lcm-topic", default="/zed/video", help="LCM topic for publishing (default: /zed/video)"
    )

    args = parser.parse_args()

    # Initialize LCM
    pubsub.lcm.autoconf()

    # Start dimos
    logger.info("Starting dimos...")
    dimos = core.start(8)

    # Deploy the TCP camera module
    logger.info(f"Deploying TCP camera module...")
    logger.info(f"  Server: {args.host}:{args.port}")
    logger.info(f"  Frame ID: {args.frame_id}")

    camera = dimos.deploy(
        TCPCameraModule,
        host=args.host,
        port=args.port,
        frame_id=args.frame_id,
    )

    # Set up LCM transport for the video output
    camera.video.transport = core.LCMTransport(args.lcm_topic, Image)

    # Counter for received frames
    frame_count = [0]
    last_log_time = [time.time()]
    first_timestamp = [None]
    timestamp_validated = [False]

    def on_frame(msg):
        frame_count[0] += 1
        current_time = time.time()

        # Validate timestamp on first frame
        if first_timestamp[0] is None:
            first_timestamp[0] = msg.ts
            logger.info(f"First frame timestamp: {msg.ts:.6f}")

            # Check if this is an absolute Unix timestamp
            if msg.ts > 1577836800:  # > Jan 1, 2020
                timestamp_validated[0] = True
                drift = abs(msg.ts - current_time)
                logger.info("✓ Timestamps are ABSOLUTE (Unix epoch)")
                logger.info(f"  Current system time: {current_time:.6f}")
                logger.info(f"  Time drift: {drift:.3f} seconds")
            else:
                logger.error("✗ Timestamps are NOT absolute Unix timestamps!")
                logger.error(f"  Received: {msg.ts:.6f} (expected > 1577836800)")

        # Log stats every 2 seconds
        if current_time - last_log_time[0] >= 2.0:
            fps = frame_count[0] / (current_time - last_log_time[0])

            # Calculate latency if timestamps are absolute
            if timestamp_validated[0]:
                latency = current_time - msg.ts
                logger.info(
                    f"Frames: {frame_count[0]} | FPS: {fps:.1f} | "
                    f"Resolution: {msg.width}x{msg.height} | "
                    f"Timestamp: {msg.ts:.3f} | Latency: {latency:.3f}s"
                )
            else:
                logger.info(
                    f"Frames: {frame_count[0]} | FPS: {fps:.1f} | "
                    f"Resolution: {msg.width}x{msg.height}"
                )

            frame_count[0] = 0
            last_log_time[0] = current_time

    # Subscribe to video output for monitoring
    camera.video.subscribe(on_frame)

    # Start the camera
    logger.info("Starting TCP camera...")
    camera.start()

    logger.info("=" * 60)
    logger.info("TCP camera module is running. Press Ctrl+C to stop.")
    logger.info(f"Receiving TCP stream from: {args.host}:{args.port}")
    logger.info(f"Publishing frames to LCM topic: {args.lcm_topic}")
    logger.info("=" * 60)
    logger.info("")
    logger.info("To start the TCP server on the sender machine:")
    logger.info(f"  python3 tcp_sender.py --device /dev/video0 --port {args.port}")
    logger.info("")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        logger.info("\nShutting down...")
        camera.stop()

        if timestamp_validated[0]:
            logger.info("✓ Successfully received frames with absolute timestamps")
        else:
            logger.warning("⚠ Did not receive proper absolute timestamps")

        logger.info("Stopped.")


if __name__ == "__main__":
    main()
