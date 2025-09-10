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
import time
import argparse

from dimos.hardware.rtsp_camera import RTSPCameraModule
from dimos import core
from dimos.protocol import pubsub
from dimos.msgs.sensor_msgs import Image

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def main():
    """Test the RTSP camera module."""

    parser = argparse.ArgumentParser(description="Test RTSP camera module")
    parser.add_argument(
        "--rtsp-url",
        default="rtsp://localhost:8554/video",
        help="RTSP stream URL (default: rtsp://localhost:8554/video)",
    )
    parser.add_argument("--rtsp-host", help="RTSP server host (overrides URL)")
    parser.add_argument(
        "--rtsp-port", type=int, default=8554, help="RTSP server port (default: 8554)"
    )
    parser.add_argument(
        "--mount-point", default="/video", help="RTSP mount point (default: /video)"
    )
    parser.add_argument(
        "--tcp", action="store_true", help="Use TCP for RTSP transport instead of UDP"
    )
    parser.add_argument(
        "--frame-id",
        default="zed_camera",
        help="Frame ID for published images (default: zed_camera)",
    )
    parser.add_argument(
        "--lcm-topic", default="/zed/video", help="LCM topic for publishing (default: /zed/video)"
    )
    args = parser.parse_args()

    # Build RTSP URL if host is specified
    if args.rtsp_host:
        rtsp_url = f"rtsp://{args.rtsp_host}:{args.rtsp_port}{args.mount_point}"
    else:
        rtsp_url = args.rtsp_url

    # Initialize LCM
    pubsub.lcm.autoconf()

    # Start dimos
    logger.info("Starting dimos...")
    dimos = core.start(8)

    # Deploy the RTSP camera module
    logger.info(f"Deploying RTSP camera module...")
    logger.info(f"  RTSP URL: {rtsp_url}")
    logger.info(f"  Transport: {'TCP' if args.tcp else 'UDP'}")
    logger.info(f"  Frame ID: {args.frame_id}")

    camera = dimos.deploy(
        RTSPCameraModule,
        rtsp_url=rtsp_url,
        frame_id=args.frame_id,
        tcp_mode=args.tcp,
    )

    # Set up LCM transport for the video output
    camera.video.transport = core.LCMTransport(args.lcm_topic, Image)

    # Counter for received frames
    frame_count = [0]
    last_log_time = [time.time()]
    first_timestamp = [None]
    timestamp_issues = [0]

    def on_frame(msg):
        frame_count[0] += 1
        current_time = time.time()

        # Check timestamp validity
        if first_timestamp[0] is None:
            first_timestamp[0] = msg.ts
            logger.info(f"First frame timestamp: {msg.ts:.6f}")
            # Check if this looks like an absolute timestamp
            if msg.ts > 1577836800:  # > Jan 1, 2020
                logger.info("✓ Timestamps appear to be absolute (Unix epoch)")
            else:
                logger.warning("⚠ Timestamps appear to be relative (not Unix epoch)")
                timestamp_issues[0] += 1

        # Check for timestamp monotonicity
        if frame_count[0] > 1:
            if hasattr(on_frame, "last_ts"):
                if msg.ts <= on_frame.last_ts:
                    logger.warning(
                        f"⚠ Non-monotonic timestamp: {msg.ts:.6f} <= {on_frame.last_ts:.6f}"
                    )
                    timestamp_issues[0] += 1
        on_frame.last_ts = msg.ts

        # Log stats every 2 seconds
        if current_time - last_log_time[0] >= 2.0:
            fps = frame_count[0] / (current_time - last_log_time[0])

            # Calculate timestamp drift
            timestamp_drift = msg.ts - current_time

            logger.info(
                f"Frames: {frame_count[0]} | FPS: {fps:.1f} | "
                f"Resolution: {msg.width}x{msg.height} | "
                f"Timestamp: {msg.ts:.3f} | Drift: {timestamp_drift:+.3f}s"
            )

            if timestamp_issues[0] > 0:
                logger.warning(f"⚠ Timestamp issues detected: {timestamp_issues[0]}")

            frame_count[0] = 0
            last_log_time[0] = current_time

    # Subscribe to video output for monitoring
    camera.video.subscribe(on_frame)

    # Start the camera
    logger.info("Starting RTSP camera...")
    camera.start()

    logger.info("=" * 60)
    logger.info("RTSP camera module is running. Press Ctrl+C to stop.")
    logger.info(f"Receiving RTSP stream from: {rtsp_url}")
    logger.info(f"Publishing frames to LCM topic: {args.lcm_topic}")
    logger.info("=" * 60)
    logger.info("")
    logger.info("To start the RTSP server on the sender machine:")
    logger.info("  python3 rtsp_sender.py --device /dev/video0")
    logger.info("")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        logger.info("\nShutting down...")
        camera.stop()

        if timestamp_issues[0] > 0:
            logger.warning(f"Total timestamp issues: {timestamp_issues[0]}")
        else:
            logger.info("✓ No timestamp issues detected")

        logger.info("Stopped.")


if __name__ == "__main__":
    main()
