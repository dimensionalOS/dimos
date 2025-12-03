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

import json
import threading
import time
from typing import Optional, Dict, Any
import zenoh
import numpy as np
from reactivex.subject import Subject
from reactivex.observable import Observable
from reactivex import operators as ops

from dimos.types.image import Image
from dimos.types.position import Position
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.robot.unitree_webrtc.type.odometry import Odometry
from dimos.robot.unitree_webrtc.type.lowstate import LowStateMsg
from dimos.robot.connection_interface import ConnectionInterface
from dimos.utils.reactive import backpressure, callback_to_observable
from dimos.stream.zenoh_video_provider import ZenohVideoProvider
from dimos.utils.logging_config import setup_logger
from go2_webrtc_driver.constants import RTC_TOPIC, VUI_COLOR, SPORT_CMD

logger = setup_logger("dimos.robot.unitree_webrtc.connection")


class RobotMode:
    """Enum for robot modes (matching ROS version)"""

    UNKNOWN = "unknown"
    INITIALIZING = "initializing"
    IDLE = "idle"
    MOVING = "moving"
    ERROR = "error"


class WebRTCRobot(ConnectionInterface):
    """WebRTC Robot connection interface using Zenoh communication"""

    # Default configuration values for Unitree Go2
    DEFAULT_MAX_LINEAR_VELOCITY = 1.0
    DEFAULT_MAX_ANGULAR_VELOCITY = 2.0

    def __init__(
        self,
        ip: str,
        mode: str = "normal",
        node_name: str = None,
        max_linear_velocity: float = None,
        max_angular_velocity: float = None,
        enable_video: bool = True,
    ):
        """Initialize WebRTC robot interface.

        Note: This now connects via Zenoh instead of direct WebRTC.
        The actual WebRTC connection should be handled by run_webrtc.py

        Args:
            ip: Robot IP address (for logging/compatibility)
            mode: Robot mode (for logging/compatibility)
            node_name: Name for this interface instance
            max_linear_velocity: Maximum linear velocity in m/s
            max_angular_velocity: Maximum angular velocity in rad/s
            enable_video: Whether to enable video stream (typically True)
        """
        self.ip = ip
        self.mode = mode
        self.node_name = node_name or f"webrtc_robot_{ip.replace('.', '_')}"

        # Use default values if not provided
        self.MAX_LINEAR_VELOCITY = max_linear_velocity or self.DEFAULT_MAX_LINEAR_VELOCITY
        self.MAX_ANGULAR_VELOCITY = max_angular_velocity or self.DEFAULT_MAX_ANGULAR_VELOCITY

        # Initialize Zenoh session
        self.zenoh_session = zenoh.open(zenoh.Config())
        logger.info(f"Zenoh session opened for {self.node_name}")

        # State tracking
        self._robot_state = None
        self._imu_state = None
        self._odom_data = None
        self._costmap_data = None
        self._mode = RobotMode.INITIALIZING
        self._last_sensor_data_time = time.time()

        # Data streams
        self._camera_subject = Subject()
        self._lidar_subject = Subject()
        self._odometry_subject = Subject()
        self._lowstate_subject = Subject()

        # Video provider
        self._video_provider = None
        if enable_video:
            self._video_provider = ZenohVideoProvider(dev_name=f"{self.node_name}_video")

        # Publishers (commands to WebRTC process)
        self.movement_publisher = self.zenoh_session.declare_publisher("commands/movement")
        self.api_publisher = self.zenoh_session.declare_publisher("commands/api_request")

        # Subscribers (sensor data from WebRTC process)
        self.camera_subscriber = self.zenoh_session.declare_subscriber(
            "sensors/camera/image", self._handle_camera_data
        )
        self.lidar_subscriber = self.zenoh_session.declare_subscriber(
            "sensors/lidar/pointcloud", self._handle_lidar_data
        )
        self.odometry_subscriber = self.zenoh_session.declare_subscriber(
            "sensors/odometry", self._handle_odometry_data
        )
        self.lowstate_subscriber = self.zenoh_session.declare_subscriber(
            "sensors/lowstate", self._handle_lowstate_data
        )

        logger.info(f"WebRTC Robot interface initialized for {ip} (Zenoh-based)")
        logger.info("Waiting for sensor data from WebRTC bridge process...")

        # Set mode to idle (assume WebRTC process handles connection)
        self._mode = RobotMode.IDLE

    def _convert_zenoh_payload(self, payload):
        """Convert Zenoh payload to bytes consistently."""
        try:
            if hasattr(payload, "to_bytes"):
                # Zenoh ZBytes object
                return payload.to_bytes()
            elif hasattr(payload, "__bytes__"):
                # Object that can be converted to bytes
                return bytes(payload)
            else:
                # Assume it's already bytes
                return payload
        except Exception as e:
            logger.error(f"Error converting Zenoh payload: {e}")
            return None

    def _handle_camera_data(self, sample):
        """Handle incoming camera data from Zenoh."""
        try:
            payload_bytes = self._convert_zenoh_payload(sample.payload)
            if payload_bytes is None:
                logger.error("Failed to convert camera payload")
                return

            image = Image.from_zenoh_binary(payload_bytes)
            frame_array = image.data  # Use .data property instead of .to_array()

            # Push to video provider if available
            if self._video_provider:
                self._video_provider.push_data(frame_array)

            # Emit to camera subject
            self._camera_subject.on_next(frame_array)

            # Update last sensor data time
            self._last_sensor_data_time = time.time()

        except Exception as e:
            logger.error(f"Error processing camera data: {e}")

    def _handle_lidar_data(self, sample):
        """Handle incoming LiDAR data from Zenoh."""
        try:
            payload_bytes = self._convert_zenoh_payload(sample.payload)
            if payload_bytes is None:
                logger.error("Failed to convert LiDAR payload")
                return

            lidar_msg = LidarMessage.from_zenoh_binary(payload_bytes)
            self._lidar_subject.on_next(lidar_msg)

            # Update last sensor data time
            self._last_sensor_data_time = time.time()

        except Exception as e:
            logger.error(f"Error processing LiDAR data: {e}")

    def _handle_odometry_data(self, sample):
        """Handle incoming odometry data from Zenoh."""
        try:
            payload_bytes = self._convert_zenoh_payload(sample.payload)
            if payload_bytes is None:
                logger.error("Failed to convert odometry payload")
                return

            odom_msg = Odometry.from_zenoh_binary(payload_bytes)
            self._odom_data = odom_msg
            self._odometry_subject.on_next(odom_msg)

            # Update last sensor data time
            self._last_sensor_data_time = time.time()

        except Exception as e:
            logger.error(f"Error processing odometry data: {e}")

    def _handle_lowstate_data(self, sample):
        """Handle incoming low state data from Zenoh."""
        try:
            payload_bytes = self._convert_zenoh_payload(sample.payload)
            if payload_bytes is None:
                logger.error("Failed to convert low state payload")
                return

            lowstate_data = json.loads(payload_bytes.decode("utf-8"))
            self._lowstate_subject.on_next(lowstate_data)

            # Update last sensor data time
            self._last_sensor_data_time = time.time()

        except Exception as e:
            logger.error(f"Error processing low state data: {e}")

    def _clamp_velocity(self, velocity: float, max_velocity: float) -> float:
        """Clamp velocity within safe limits"""
        return max(min(velocity, max_velocity), -max_velocity)

    def is_bridge_connected(self) -> bool:
        """Check if the WebRTC bridge process is sending data."""
        # Consider bridge connected if we received sensor data within last 5 seconds
        return (time.time() - self._last_sensor_data_time) < 5.0

    # ConnectionInterface implementation
    def move(self, x: float, y: float, yaw: float, duration: float = 0.0) -> bool:
        """Send movement command to the robot using velocity commands.

        Args:
            x: Forward/backward velocity (m/s)
            y: Left/right velocity (m/s)
            yaw: Rotational velocity (rad/s)
            duration: How long to move (seconds). If 0, command is continuous

        Returns:
            bool: True if command was sent successfully
        """
        try:
            if not self.is_bridge_connected():
                logger.warning(
                    "WebRTC bridge appears disconnected - movement command may not be processed"
                )

            # Clamp velocities to safe limits
            x = self._clamp_velocity(x, self.MAX_LINEAR_VELOCITY)
            y = self._clamp_velocity(y, self.MAX_LINEAR_VELOCITY)
            yaw = self._clamp_velocity(yaw, self.MAX_ANGULAR_VELOCITY)

            # Create movement command
            command = {"x": float(x), "y": float(y), "yaw": float(yaw), "duration": float(duration)}

            # Publish to Zenoh
            self.movement_publisher.put(json.dumps(command).encode("utf-8"))
            logger.debug(f"Sent movement command: x={x}, y={y}, yaw={yaw}, duration={duration}")
            return True

        except Exception as e:
            logger.error(f"Failed to send movement command: {e}")
            return False

    def stop(self) -> bool:
        """Stop the robot's movement."""
        return self.move(0.0, 0.0, 0.0)

    def get_video_stream(self, fps: int = 30) -> Optional[Observable]:
        """Get the video stream from the robot's camera.

        Args:
            fps: Frames per second for the video stream

        Returns:
            Observable: An observable stream of video frames or None if not available
        """
        if not self._video_provider:
            logger.warning("Video provider not available")
            return None

        if not self.is_bridge_connected():
            logger.warning("WebRTC bridge appears disconnected - video stream may not have data")

        return self._video_provider.get_stream(fps=fps)

    def disconnect(self) -> None:
        """Disconnect from the robot and clean up resources."""
        try:
            logger.info("Disconnecting WebRTC robot interface...")

            # Close data streams
            self._camera_subject.on_completed()
            self._lidar_subject.on_completed()
            self._odometry_subject.on_completed()
            self._lowstate_subject.on_completed()

            # Close video provider
            if self._video_provider:
                # Video provider doesn't have a close method, but we can stop pushing data
                pass

            # Close Zenoh session
            if hasattr(self, "zenoh_session"):
                self.zenoh_session.close()

            logger.info("Zenoh connection cleanup complete")

        except Exception as e:
            logger.error(f"Error during disconnect: {e}")

    # Robot-specific methods (compatible with existing WebRTC robot interface)
    def raw_lidar_stream(self) -> Subject[LidarMessage]:
        """Get raw LiDAR stream."""
        return backpressure(self._lidar_subject)

    def raw_odom_stream(self) -> Subject[Position]:
        """Get raw odometry stream."""
        return backpressure(self._odometry_subject)

    def lidar_stream(self) -> Subject[LidarMessage]:
        """Get processed LiDAR stream."""
        return backpressure(self._lidar_subject)

    def odom_stream(self) -> Subject[Position]:
        """Get processed odometry stream."""
        return backpressure(self._odometry_subject)

    def lowstate_stream(self) -> Subject[LowStateMsg]:
        """Get low state stream."""
        return backpressure(self._lowstate_subject)

    def video_stream(self) -> Observable:
        """Get video stream."""
        return self._camera_subject.pipe(ops.share())

    def publish_request(self, topic: str, data: dict):
        """Send API request to the robot.

        Args:
            topic: API topic to publish to
            data: Data dictionary to send
        """
        try:
            if not self.is_bridge_connected():
                logger.warning(
                    "WebRTC bridge appears disconnected - API request may not be processed"
                )

            request = {"topic": topic, "data": data}
            self.api_publisher.put(json.dumps(request).encode("utf-8"))
            logger.debug(f"Sent API request to {topic}: {data}")
        except Exception as e:
            logger.error(f"Failed to send API request: {e}")

    def standup_ai(self):
        """Make robot stand up in AI mode."""
        return self.publish_request(RTC_TOPIC["SPORT_MOD"], {"api_id": SPORT_CMD["BalanceStand"]})

    def standup_normal(self):
        """Make robot stand up in normal mode."""
        self.publish_request(RTC_TOPIC["SPORT_MOD"], {"api_id": SPORT_CMD["StandUp"]})
        time.sleep(0.5)
        self.publish_request(RTC_TOPIC["SPORT_MOD"], {"api_id": SPORT_CMD["RecoveryStand"]})
        return True

    def standup(self):
        """Make robot stand up (uses AI mode by default)."""
        return self.standup_ai()

    def liedown(self):
        """Make robot lie down."""
        return self.publish_request(RTC_TOPIC["SPORT_MOD"], {"api_id": SPORT_CMD["StandDown"]})

    def color(self, color: VUI_COLOR = VUI_COLOR.RED, colortime: int = 60) -> bool:
        """Set robot LED color."""
        return self.publish_request(
            RTC_TOPIC["VUI"],
            {
                "api_id": 1001,
                "parameter": {
                    "color": color,
                    "time": colortime,
                },
            },
        )

    # Compatibility methods for ROS-like interface
    def get_odometry(self) -> Optional[Odometry]:
        """Get current odometry data."""
        return self._odom_data

    def get_robot_mode(self):
        """Get current robot mode."""
        return self._mode
