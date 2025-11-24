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


def to_euler_rot(msg: TransformStamped) -> [Vector, Vector]:
    q = msg.transform.rotation
    rotation = R.from_quat([q.x, q.y, q.z, q.w])
    return Vector(rotation.as_euler("xyz", degrees=False))


def to_euler_pos(msg: TransformStamped) -> [Vector, Vector]:
    return Vector(msg.transform.translation).to_2d()

def to_euler(msg: TransformStamped) -> [Vector, Vector]:
    return [to_euler_pos(msg), to_euler_rot(msg)]


class ROSTransformAbility:
    """Mixin class for handling ROS transforms between coordinate frames"""

    @property
    def tf_buffer(self) -> Buffer:
        if not hasattr(self, "_tf_buffer"):
            self._tf_buffer = tf2_ros.Buffer()
            self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self._node)
            logger.info("Transform listener initialized")

        return self._tf_buffer

    def transform_euler_pos(self, source_frame: str, target_frame: str = "map", timeout: float = 1.0):
        return to_euler_pos(self.transform(source_frame, target_frame, timeout))

    def transform_euler_rot(self, source_frame: str, target_frame: str = "map", timeout: float = 1.0):
        return to_euler_rot(self.transform(source_frame, target_frame, timeout))

    def transform_euler(self, source_frame: str, target_frame: str = "map", timeout: float = 1.0):
        res = self.transform(source_frame, target_frame, timeout)
        return to_euler(res)

    def transform(
        self, source_frame: str, target_frame: str = "map", timeout: float = 1.0
    ) -> Optional[TransformStamped]:
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
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
        """Transform a point from source_frame to target_frame.

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
                target_frame, source_frame, rclpy.time.Time(), rclpy.duration.Duration(seconds=timeout)
            )

            # Create a PointStamped message
            ps = PointStamped()
            ps.header.frame_id = source_frame
            ps.header.stamp = rclpy.time.Time().to_msg()  # Latest available transform
            ps.point.x = point[0]
            ps.point.y = point[1]
            ps.point.z = point[2] if len(point) > 2 else 0.0

            # Transform point
            transformed_ps = self.tf_buffer.transform(ps, target_frame, rclpy.duration.Duration(seconds=timeout))

            # Return as Vector type
            if len(point) > 2:
                return Vector(transformed_ps.point.x, transformed_ps.point.y, transformed_ps.point.z)
            else:
                return Vector(transformed_ps.point.x, transformed_ps.point.y)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
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

    def transform_rot(self, rotation: Vector, source_frame: str, target_frame: str = "map", timeout: float = 1.0):
        """Transform a rotation from source_frame to target_frame.

        Args:
            rotation: The rotation to transform as Euler angles (x, y, z) in radians
            source_frame: The source frame of the rotation
            target_frame: The target frame to transform to
            timeout: Time to wait for the transform to become available (seconds)

        Returns:
            The transformed rotation as a Vector of Euler angles (x, y, z), or None if the transform failed
        """
        try:
            # Wait for transform to become available
            self.tf_buffer.can_transform(
                target_frame, source_frame, rclpy.time.Time(), rclpy.duration.Duration(seconds=timeout)
            )
            
            # Create a rotation matrix from the input Euler angles
            input_rotation = R.from_euler('xyz', rotation, degrees=False)
            
            # Get the transform from source to target frame
            transform = self.transform(source_frame, target_frame, timeout)
            if transform is None:
                return None
                
            # Extract the rotation from the transform
            q = transform.transform.rotation
            transform_rotation = R.from_quat([q.x, q.y, q.z, q.w])
            
            # Compose the rotations
            # The resulting rotation is the composition of the transform rotation and input rotation
            result_rotation = transform_rotation * input_rotation
            
            # Convert back to Euler angles
            euler_angles = result_rotation.as_euler('xyz', degrees=False)
            
            # Return as Vector type
            return Vector(euler_angles)
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            logger.error(f"Transform rotation from {source_frame} to {target_frame} failed: {e}")
            return None

    def transform_pose(self, position: Vector, rotation: Vector, source_frame: str, target_frame: str = "map", timeout: float = 1.0):
        """Transform a pose from source_frame to target_frame.

        Args:
            position: The position to transform
            rotation: The rotation to transform
            source_frame: The source frame of the pose
            target_frame: The target frame to transform to
            timeout: Time to wait for the transform to become available (seconds)

        Returns:
            Tuple of (transformed_position, transformed_rotation) as Vectors, 
            or (None, None) if either transform failed
        """
        # Transform position
        transformed_position = self.transform_point(position, source_frame, target_frame, timeout)
        
        # Transform rotation
        transformed_rotation = self.transform_rot(rotation, source_frame, target_frame, timeout)
        
        # Return results (both might be None if transforms failed)
        return transformed_position, transformed_rotation