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

"""
WebRTC to Zenoh Bridge Process

This process connects to the Unitree Go2 robot via WebRTC and bridges sensor data
to Zenoh topics while subscribing to command topics from other processes.

Sensor Data (Published to Zenoh with binary serialization):
- sensors/camera/image: Image data
- sensors/lidar/pointcloud: LiDAR point cloud data
- sensors/odometry: Robot odometry data
- sensors/lowstate: Robot low-level state

Command Data (Subscribed from Zenoh with JSON):
- commands/movement: Movement velocity commands
- commands/api_request: WebRTC API requests (standup, liedown, etc.)
"""

import asyncio
import time
import json
import signal
import sys
import os
from typing import Optional
import zenoh
import numpy as np

from dimos.types.image import Image, ImageFormat
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.robot.unitree_webrtc.type.odometry import Odometry
from dimos.robot.unitree_webrtc.type.lowstate import LowStateMsg
from go2_webrtc_driver.webrtc_driver import Go2WebRTCConnection, WebRTCConnectionMethod
from aiortc import MediaStreamTrack
from go2_webrtc_driver.constants import RTC_TOPIC, VUI_COLOR, SPORT_CMD
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.robot.unitree_webrtc.run_webrtc")


class WebRTCZenohBridge:
    """Bridge between WebRTC robot connection and Zenoh communication."""

    def __init__(self, robot_ip: str, mode: str = "ai"):
        """Initialize the WebRTC to Zenoh bridge.

        Args:
            robot_ip: IP address of the robot
            mode: Robot mode (ai, etc.)
        """
        self.robot_ip = robot_ip
        self.mode = mode
        self.running = False
        self.loop = None  # Store the main event loop reference

        # Initialize Zenoh session
        self.zenoh_session = zenoh.open(zenoh.Config())
        logger.info("Zenoh session opened")

        # WebRTC connection
        self.webrtc_conn = None

        # Publishers (sensor data)
        self.camera_publisher = self.zenoh_session.declare_publisher("sensors/camera/image")
        self.lidar_publisher = self.zenoh_session.declare_publisher("sensors/lidar/pointcloud")
        self.odometry_publisher = self.zenoh_session.declare_publisher("sensors/odometry")
        self.lowstate_publisher = self.zenoh_session.declare_publisher("sensors/lowstate")

        logger.info("Zenoh publishers declared")

    async def connect_webrtc(self):
        """Establish WebRTC connection to the robot."""
        try:
            self.webrtc_conn = Go2WebRTCConnection(
                WebRTCConnectionMethod.LocalSTA, ip=self.robot_ip
            )

            await self.webrtc_conn.connect()
            await self.webrtc_conn.datachannel.disableTrafficSaving(True)
            self.webrtc_conn.datachannel.set_decoder(decoder_type="native")

            # Set robot mode
            await self.webrtc_conn.datachannel.pub_sub.publish_request_new(
                RTC_TOPIC["MOTION_SWITCHER"], {"api_id": 1002, "parameter": {"name": self.mode}}
            )

            # Setup sensor data subscriptions
            self._setup_sensor_subscriptions()

            # Setup video stream
            await self._setup_video_stream()

            logger.info(f"WebRTC connected to robot at {self.robot_ip}")

        except Exception as e:
            logger.error(f"Failed to connect to robot: {e}")
            raise

    def _setup_sensor_subscriptions(self):
        """Setup subscriptions to robot sensor data."""
        # LiDAR subscription
        self.webrtc_conn.datachannel.pub_sub.subscribe(
            RTC_TOPIC["ULIDAR_ARRAY"], self._handle_lidar_data
        )

        # Odometry subscription
        self.webrtc_conn.datachannel.pub_sub.subscribe(
            RTC_TOPIC["ROBOTODOM"], self._handle_odometry_data
        )

        # Low state subscription
        self.webrtc_conn.datachannel.pub_sub.subscribe(
            RTC_TOPIC["LOW_STATE"], self._handle_lowstate_data
        )

        logger.info("Sensor subscriptions setup complete")

    async def _setup_video_stream(self):
        """Setup video stream from robot camera."""

        async def video_track_callback(track: MediaStreamTrack):
            """Handle incoming video frames."""
            try:
                while self.running:
                    frame = await track.recv()
                    # Convert frame to numpy array
                    frame_array = frame.to_ndarray(format="bgr24")

                    # Create Image object and publish
                    image = Image.from_opencv(frame_array, format=ImageFormat.BGR)
                    binary_data = image.to_zenoh_binary()
                    self.camera_publisher.put(binary_data)

            except Exception as e:
                if self.running:  # Only log if we're still supposed to be running
                    logger.error(f"Video stream error: {e}")

        self.webrtc_conn.video.add_track_callback(video_track_callback)
        self.webrtc_conn.video.switchVideoChannel(True)

        logger.info("Video stream setup complete")

    def _handle_lidar_data(self, raw_data):
        """Handle incoming LiDAR data and publish to Zenoh."""
        try:
            lidar_msg = LidarMessage.from_msg(raw_data)
            binary_data = lidar_msg.to_zenoh_binary()
            self.lidar_publisher.put(binary_data)
        except Exception as e:
            logger.error(f"Error processing LiDAR data: {e}")

    def _handle_odometry_data(self, raw_data):
        """Handle incoming odometry data and publish to Zenoh."""
        try:
            odom_msg = Odometry.from_msg(raw_data)
            binary_data = odom_msg.to_zenoh_binary()
            self.odometry_publisher.put(binary_data)
        except Exception as e:
            logger.error(f"Error processing odometry data: {e}")

    def _handle_lowstate_data(self, raw_data):
        """Handle incoming low state data and publish to Zenoh."""
        try:
            # Just publish the raw data as JSON
            json_data = json.dumps(raw_data).encode("utf-8")
            self.lowstate_publisher.put(json_data)
        except Exception as e:
            logger.error(f"Error processing low state data: {e}")

    def _handle_movement_command(self, sample):
        """Handle movement commands from Zenoh."""
        try:
            # Handle ZBytes from Zenoh
            if hasattr(sample.payload, "to_bytes"):
                payload_bytes = sample.payload.to_bytes()
            else:
                payload_bytes = sample.payload

            command_data = json.loads(payload_bytes.decode("utf-8"))
            x = command_data.get("x", 0.0)
            y = command_data.get("y", 0.0)
            yaw = command_data.get("yaw", 0.0)
            duration = command_data.get("duration", 0.0)

            # Schedule async work on the main event loop
            if self.loop and self.loop.is_running():
                asyncio.run_coroutine_threadsafe(
                    self._send_movement_command(x, y, yaw, duration), self.loop
                )

        except Exception as e:
            logger.error(f"Error handling movement command: {e}")

    def _handle_api_request(self, sample):
        """Handle API requests from Zenoh."""
        try:
            # Handle ZBytes from Zenoh
            if hasattr(sample.payload, "to_bytes"):
                payload_bytes = sample.payload.to_bytes()
            else:
                payload_bytes = sample.payload

            request_data = json.loads(payload_bytes.decode("utf-8"))
            topic = request_data.get("topic")
            data = request_data.get("data", {})

            # Schedule async work on the main event loop
            if self.loop and self.loop.is_running():
                asyncio.run_coroutine_threadsafe(self._send_api_request(topic, data), self.loop)

        except Exception as e:
            logger.error(f"Error handling API request: {e}")

    async def _send_movement_command(self, x: float, y: float, yaw: float, duration: float = 0.0):
        """Send movement command to robot via WebRTC."""
        try:
            # WebRTC coordinate mapping:
            # x - Positive right, negative left
            # y - positive forward, negative backwards
            # yaw - Positive rotate right, negative rotate left

            if duration > 0:
                # Send continuous commands for duration
                start_time = time.time()
                while time.time() - start_time < duration and self.running:
                    await self.webrtc_conn.datachannel.pub_sub.publish_without_callback(
                        RTC_TOPIC["WIRELESS_CONTROLLER"],
                        data={"lx": y, "ly": x, "rx": -yaw, "ry": 0},
                    )
                    await asyncio.sleep(0.01)  # 100Hz update rate
            else:
                # Single command
                await self.webrtc_conn.datachannel.pub_sub.publish_without_callback(
                    RTC_TOPIC["WIRELESS_CONTROLLER"], data={"lx": y, "ly": x, "rx": -yaw, "ry": 0}
                )

        except Exception as e:
            logger.error(f"Error sending movement command: {e}")

    async def _send_api_request(self, topic: str, data: dict):
        """Send API request to robot via WebRTC."""
        try:
            await self.webrtc_conn.datachannel.pub_sub.publish_request_new(topic, data)
        except Exception as e:
            logger.error(f"Error sending API request: {e}")

    async def run(self):
        """Main run loop for the bridge."""
        self.running = True
        self.loop = asyncio.get_running_loop()  # Store the main event loop reference

        try:
            # Connect to robot
            await self.connect_webrtc()

            # Setup Zenoh command subscribers (async)
            movement_subscriber = self.zenoh_session.declare_subscriber(
                "commands/movement", self._handle_movement_command
            )
            api_subscriber = self.zenoh_session.declare_subscriber(
                "commands/api_request", self._handle_api_request
            )

            logger.info("Zenoh command subscribers setup complete")

            # Keep the connection alive
            while self.running:
                await asyncio.sleep(1.0)

        except Exception as e:
            logger.error(f"Bridge run error: {e}")
            raise
        finally:
            await self.cleanup()

    async def cleanup(self):
        """Clean up resources."""
        self.running = False

        if self.webrtc_conn:
            try:
                if hasattr(self.webrtc_conn, "video"):
                    self.webrtc_conn.video.switchVideoChannel(False)

                await self.webrtc_conn.disconnect()
            except Exception as e:
                logger.error(f"Error disconnecting WebRTC: {e}")

        if hasattr(self, "zenoh_session"):
            self.zenoh_session.close()

        logger.info("Bridge cleanup complete")


def signal_handler(signum, frame):
    """Handle shutdown signals."""
    logger.info(f"Received signal {signum}, shutting down...")
    sys.exit(0)


async def main():
    """Main entry point."""
    import argparse

    parser = argparse.ArgumentParser(description="WebRTC to Zenoh Bridge")
    parser.add_argument("--ip", default=os.getenv("ROBOT_IP"), help="Robot IP address")
    parser.add_argument("--mode", default="normal", help="Robot mode (default: normal)")
    args = parser.parse_args()

    # Setup signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    bridge = WebRTCZenohBridge(args.ip, args.mode)

    try:
        await bridge.run()
    except KeyboardInterrupt:
        logger.info("Shutting down due to keyboard interrupt")
    except Exception as e:
        logger.error(f"Bridge failed: {e}")
        raise
    finally:
        await bridge.cleanup()


if __name__ == "__main__":
    asyncio.run(main())
