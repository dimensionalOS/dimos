# Copyright 2026 Dimensional Inc.
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

"""Geometry operations on generated ROS value types."""

import math

from dimos_generated.geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from dimos_generated.nav_msgs.msg import Odometry


def yaw(rotation: Quaternion) -> float:
    """Return rotation about Z in radians (ROS XYZW quaternion convention)."""
    return math.atan2(
        2 * (rotation.w * rotation.z + rotation.x * rotation.y),
        1 - 2 * (rotation.y * rotation.y + rotation.z * rotation.z),
    )


def transform_from_odometry(message: Odometry) -> TransformStamped:
    """Copy the pose and declared frames into TF, preserving the exact source stamp."""
    position = message.pose.pose.position
    return TransformStamped(
        header=message.header,
        child_frame_id=message.child_frame_id,
        transform=Transform(
            translation=Vector3(x=position.x, y=position.y, z=position.z),
            rotation=message.pose.pose.orientation,
        ),
    )
