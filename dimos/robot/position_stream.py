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
import numpy as np
from typing import Tuple, Optional, Callable, Dict, Any
import time
from reactivex import Subject, Observable
from reactivex import operators as ops
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.robot.position_stream", level=logging.INFO)

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
        pose_topic: Optional[str] = None,
        use_odometry: bool = True
    ):
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
        
        self._subject = Subject()
        
        self.last_position = None
        self.last_update_time = None
        
        self._create_subscription()
        
        logger.info(f"PositionStreamProvider initialized with "
                   f"{'odometry topic' if use_odometry else 'pose topic'}: "
                   f"{odometry_topic if use_odometry else pose_topic}")
    
    def _create_subscription(self):
        """Create the appropriate ROS subscription based on configuration."""
        if self.use_odometry:
            self.subscription = self.ros_node.create_subscription(
                Odometry,
                self.odometry_topic,
                self._odometry_callback,
                10
            )
            logger.info(f"Subscribed to odometry topic: {self.odometry_topic}")
        else:
            if not self.pose_topic:
                raise ValueError("Pose topic must be specified when use_odometry is False")
            
            self.subscription = self.ros_node.create_subscription(
                PoseStamped,
                self.pose_topic,
                self._pose_callback,
                10
            )
            logger.info(f"Subscribed to pose topic: {self.pose_topic}")
    
    
    def _odometry_callback(self, msg: Odometry):
        """
        Process odometry messages and extract position.
        
        Args:
            msg: Odometry message from ROS
        """
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        self._update_position(x, y)
    
    def _pose_callback(self, msg: PoseStamped):
        """
        Process pose messages and extract position.
        
        Args:
            msg: PoseStamped message from ROS
        """
        x = msg.pose.position.x
        y = msg.pose.position.y
        
        self._update_position(x, y)
    
    def _update_position(self, x: float, y: float):
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
        
        self.last_position = position
        self.last_update_time = current_time
        
        self._subject.on_next(position)
        logger.debug(f"Position updated: ({x:.2f}, {y:.2f})")
    
    def get_position_stream(self) -> Observable:
        """
        Get an Observable stream of position updates.
        
        Returns:
            Observable that emits (x, y) tuples
        """
        return self._subject.pipe(
            ops.share()  # Share the stream among multiple subscribers
        )
    
    def get_current_position(self) -> Optional[Tuple[float, float]]:
        """
        Get the most recent position.
        
        Returns:
            Tuple of (x, y) coordinates, or None if no position has been received
        """
        return self.last_position
    
    def cleanup(self):
        """Clean up resources."""
        if hasattr(self, 'subscription') and self.subscription:
            self.ros_node.destroy_subscription(self.subscription)
            logger.info("Position subscription destroyed")
