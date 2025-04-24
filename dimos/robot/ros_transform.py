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
from tf2_geometry_msgs import PointStamped
from dimos.utils.logging_config import setup_logger
from dimos.types.vector import Vector
from dimos.types.path import Path
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
        
    def transform_point(self, point: Vector, source_frame: str, target_frame: str = "map", timeout: float = 1.0):
        """Transform a point from child_frame to parent_frame.
        
        Args:
            point: The point to transform (x, y, z)
            source_frame: The source frame of the point
            target_frame: The target frame to transform to
            timeout: Time to wait for the transform to become available (seconds)
            
        Returns:
            The transformed point as a Vector, or None if the transform failed
        """
        try:
            # Wait for transform to become available
            self.tf_buffer.can_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=timeout)
            )
            
            # Create a PointStamped message
            ps = PointStamped()
            ps.header.frame_id = source_frame
            ps.header.stamp = rclpy.time.Time().to_msg()  # Latest available transform
            ps.point.x = point[0]
            ps.point.y = point[1]
            ps.point.z = point[2] if len(point) > 2 else 0.0
            
            # Transform point
            transformed_ps = self.tf_buffer.transform(
                ps, 
                target_frame,
                rclpy.duration.Duration(seconds=timeout)
            )
            
            # Return as Vector type
            if len(point) > 2:
                return Vector(transformed_ps.point.x, transformed_ps.point.y, transformed_ps.point.z)
            else:
                return Vector(transformed_ps.point.x, transformed_ps.point.y)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
            logger.error(f"Transform from {source_frame} to {target_frame} failed: {e}")
            return None
        
    def transform_path(self, path: Path, source_frame: str, target_frame: str = "map", timeout: float = 1.0):
        """Transform a path from source_frame to target_frame.
        
        Args:
            path: The path to transform
            source_frame: The source frame of the path  
            target_frame: The target frame to transform to
            timeout: Time to wait for the transform to become available (seconds)
            
        Returns:
            The transformed path as a Path, or None if the transform failed
        """
        transformed_path = Path()
        for point in path:
            transformed_point = self.transform_point(point, source_frame, target_frame, timeout)
            if transformed_point is not None:
                transformed_path.append(transformed_point)
        return transformed_path
