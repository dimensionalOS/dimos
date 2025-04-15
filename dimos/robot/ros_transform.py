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

import rclpy
from rclpy.node import Node
from typing import Optional
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener
import tf2_ros
from dimos.utils.logging_config import setup_logger
from reactivex import Observable, create
from reactivex.disposable import Disposable

logger = setup_logger("dimos.robot.ros_transform")

__all__ = ["ROSTransform"]


class ROSTransformAbility:
    """Base class for handling ROS transforms between coordinate frames"""

    def get_transform_stream(
        self,
        child_frame: str,
        parent_frame: str = "base_link",
        timeout: float = 1.0,
    ) -> Observable:
        """
        Creates a simple pull-based Observable stream of transforms between coordinate frames.
        Transforms are looked up directly from ROS when subscribers request data.

        Args:
            child_frame: Child/target coordinate frame
            parent_frame: Parent/source coordinate frame
            timeout: How long to wait for the transform to become available (seconds)

        Returns:
            Observable: A pull-based stream of TransformStamped messages
        """

        def lookup_on_request(observer, scheduler):
            # fetch the transform directly from ROS when requested
            observer.on_next(self.get_transform(parent_frame, child_frame, timeout))
            return Disposable()

        # Create a cold observable that fetches from ROS each time it's subscribed to
        return create(lookup_on_request)

    def get_transform(
        self, child_frame: str, parent_frame: str = "base_link", timeout: float = 1.0
    ) -> Optional[TransformStamped]:
        """
        Read transform data between two coordinate frames

        Args:
            child_frame: Child/target coordinate frame
            parent_frame: Parent/source coordinate frame
            timeout: How long to wait for the transform to become available (seconds)

        Returns:
            TransformStamped: The transform data or None if not available
        """
        try:
            # Look up transform
            transform = self._tf_buffer.lookup_transform(
                parent_frame,
                child_frame,
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=timeout),
            )
            return transform
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            logger.error(f"Transform lookup failed: {e}")
            return None
