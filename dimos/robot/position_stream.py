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
Position stream provider for ROS-based robots.

This module creates a reactive stream of position updates from ROS odometry or pose topics.
"""

import logging
import time

from geometry_msgs.msg import PoseStamped  # type: ignore[attr-defined, import-untyped]
from nav_msgs.msg import Odometry  # type: ignore[attr-defined, import-untyped]
from rclpy.node import Node  # type: ignore[import-untyped]
from reactivex import Observable, Subject, operators as ops

from dimos.utils.logging_config import setup_logger

logger = setup_logger(level=logging.INFO)


class PositionStreamProvider:
    """
    A provider for streaming position updates from ROS.

    This class creates an Observable stream of position updates by subscribing
    to ROS odometry or pose topics.
    """

    def __init__(
        self,
        ros_node: Node,
        odometry_topic: str = "/odom",
        pose_topic: str | None = None,
        use_odometry: bool = True,
    ) -> None:
        """
        Initialize the position stream provider.

        Args:
            ros_node: ROS node to use for subscriptions
            odometry_topic: Name of the odometry topic (if use_odometry is True)
            pose_topic: Name of the pose topic (if use_odometry is False)
            use_odometry: Whether to use odometry (True) or pose (False) for position
        """
        self.ros_node = ros_node
        self.odometry_topic = odometry_topic
        self.pose_topic = pose_topic
        self.use_odometry = use_odometry

        self._subject = Subject()  # type: ignore[var-annotated]

        self.last_position = None
        self.last_update_time = None

        self._create_subscription()  # type: ignore[no-untyped-call]

        logger.info(
            f"PositionStreamProvider initialized with "
            f"{'odometry topic' if use_odometry else 'pose topic'}: "
            f"{odometry_topic if use_odometry else pose_topic}"
        )

    def _create_subscription(self):  # type: ignore[no-untyped-def]
        """Create the appropriate ROS subscription based on configuration."""
        if self.use_odometry:
            self.subscription = self.ros_node.create_subscription(
                Odometry, self.odometry_topic, self._odometry_callback, 10
            )
            logger.info(f"Subscribed to odometry topic: {self.odometry_topic}")
        else:
            if not self.pose_topic:
                raise ValueError("Pose topic must be specified when use_odometry is False")

            self.subscription = self.ros_node.create_subscription(
                PoseStamped, self.pose_topic, self._pose_callback, 10
            )
            logger.info(f"Subscribed to pose topic: {self.pose_topic}")

    def _odometry_callback(self, msg: Odometry) -> None:
        """
        Process odometry messages and extract position.

        Args:
            msg: Odometry message from ROS
        """
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        self._update_position(x, y)

    def _pose_callback(self, msg: PoseStamped) -> None:
        """
        Process pose messages and extract position.

        Args:
            msg: PoseStamped message from ROS
        """
        x = msg.pose.position.x
        y = msg.pose.position.y

        self._update_position(x, y)

    def _update_position(self, x: float, y: float) -> None:
        """
        Update the current position and emit to subscribers.

        Args:
            x: X coordinate
            y: Y coordinate
        """
        current_time = time.time()
        position = (x, y)

        if self.last_update_time:
            update_rate = 1.0 / (current_time - self.last_update_time)
            logger.debug(f"Position update rate: {update_rate:.1f} Hz")

        self.last_position = position  # type: ignore[assignment]
        self.last_update_time = current_time  # type: ignore[assignment]

        self._subject.on_next(position)
        logger.debug(f"Position updated: ({x:.2f}, {y:.2f})")

    def get_position_stream(self) -> Observable:  # type: ignore[type-arg]
        """
        Get an Observable stream of position updates.

        Returns:
            Observable that emits (x, y) tuples
        """
        return self._subject.pipe(
            ops.share()  # Share the stream among multiple subscribers
        )

    def get_current_position(self) -> tuple[float, float] | None:
        """
        Get the most recent position.

        Returns:
            Tuple of (x, y) coordinates, or None if no position has been received
        """
        return self.last_position

    def cleanup(self) -> None:
        """Clean up resources."""
        if hasattr(self, "subscription") and self.subscription:
            self.ros_node.destroy_subscription(self.subscription)
            logger.info("Position subscription destroyed")
