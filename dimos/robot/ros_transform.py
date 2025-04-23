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
from typing import Optional
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer
import tf2_ros
from dimos.utils.logging_config import setup_logger
from dimos.types.vector import Vector
from scipy.spatial.transform import Rotation as R


logger = setup_logger("dimos.robot.ros_transform")

__all__ = ["ROSTransformAbility"]


def transform_to_euler(msg: TransformStamped) -> [Vector, Vector]:
    q = msg.transform.rotation
    rotation = R.from_quat([q.x, q.y, q.z, q.w])
    return [Vector(msg.transform.translation).to_2d(), Vector(rotation.as_euler("xyz", degrees=False))]


class ROSTransformAbility:
    """Mixin class for handling ROS transforms between coordinate frames"""

    @property
    def tf_buffer(self) -> Buffer:
        if not hasattr(self, "_tf_buffer"):
            self._tf_buffer = tf2_ros.Buffer()
            self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self._node)
            logger.info("Transform listener initialized")

        return self._tf_buffer

    def transform_euler(self, child_frame: str, parent_frame: str = "map", timeout: float = 1.0):
        return transform_to_euler(self.transform(child_frame, parent_frame, timeout))

    def transform(
        self, child_frame: str, parent_frame: str = "map", timeout: float = 1.0
    ) -> Optional[TransformStamped]:
        try:
            transform = self.tf_buffer.lookup_transform(
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
