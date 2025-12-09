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

from __future__ import annotations

from dimos_lcm.geometry_msgs import Twist as LCMTwist  # type: ignore[import-untyped]
from plum import dispatch

try:
    from geometry_msgs.msg import (  # type: ignore[attr-defined, import-untyped]
        Twist as ROSTwist,
        Vector3 as ROSVector3,
    )
except ImportError:
    ROSTwist = None  # type: ignore[assignment, misc]
    ROSVector3 = None  # type: ignore[assignment, misc]

# Import Quaternion at runtime for beartype compatibility
# (beartype needs to resolve forward references at runtime)
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3, VectorLike


class Twist(LCMTwist):  # type: ignore[misc]
    linear: Vector3
    angular: Vector3
    msg_name = "geometry_msgs.Twist"

    @dispatch
    def __init__(self) -> None:
        """Initialize a zero twist (no linear or angular velocity)."""
        self.linear = Vector3()
        self.angular = Vector3()

    @dispatch  # type: ignore[no-redef]
    def __init__(self, linear: VectorLike, angular: VectorLike) -> None:
        """Initialize a twist from linear and angular velocities."""

        self.linear = Vector3(linear)
        self.angular = Vector3(angular)

    @dispatch  # type: ignore[no-redef]
    def __init__(self, linear: VectorLike, angular: Quaternion) -> None:
        """Initialize a twist from linear velocity and angular as quaternion (converted to euler)."""
        self.linear = Vector3(linear)
        self.angular = angular.to_euler()

    @dispatch  # type: ignore[no-redef]
    def __init__(self, twist: Twist) -> None:
        """Initialize from another Twist (copy constructor)."""
        self.linear = Vector3(twist.linear)
        self.angular = Vector3(twist.angular)

    @dispatch  # type: ignore[no-redef]
    def __init__(self, lcm_twist: LCMTwist) -> None:
        """Initialize from an LCM Twist."""
        self.linear = Vector3(lcm_twist.linear)
        self.angular = Vector3(lcm_twist.angular)

    @dispatch  # type: ignore[no-redef]
    def __init__(self, **kwargs) -> None:
        """Handle keyword arguments for LCM compatibility."""
        linear = kwargs.get("linear", Vector3())
        angular = kwargs.get("angular", Vector3())

        self.__init__(linear, angular)

    def __repr__(self) -> str:
        return f"Twist(linear={self.linear!r}, angular={self.angular!r})"

    def __str__(self) -> str:
        return f"Twist:\n  Linear: {self.linear}\n  Angular: {self.angular}"

    def __eq__(self, other) -> bool:  # type: ignore[no-untyped-def]
        """Check if two twists are equal."""
        if not isinstance(other, Twist):
            return False
        return self.linear == other.linear and self.angular == other.angular

    @classmethod
    def zero(cls) -> Twist:
        """Create a zero twist (no motion)."""
        return cls()

    def is_zero(self) -> bool:
        """Check if this is a zero twist (no linear or angular velocity)."""
        return self.linear.is_zero() and self.angular.is_zero()

    def __bool__(self) -> bool:
        """Boolean conversion for Twist.

        A Twist is considered False if it's a zero twist (no motion),
        and True otherwise.

        Returns:
            False if twist is zero, True otherwise
        """
        return not self.is_zero()

    @classmethod
    def from_ros_msg(cls, ros_msg: ROSTwist) -> Twist:
        """Create a Twist from a ROS geometry_msgs/Twist message.

        Args:
            ros_msg: ROS Twist message

        Returns:
            Twist instance
        """

        linear = Vector3(ros_msg.linear.x, ros_msg.linear.y, ros_msg.linear.z)
        angular = Vector3(ros_msg.angular.x, ros_msg.angular.y, ros_msg.angular.z)
        return cls(linear, angular)

    def to_ros_msg(self) -> ROSTwist:
        """Convert to a ROS geometry_msgs/Twist message.

        Returns:
            ROS Twist message
        """

        ros_msg = ROSTwist()  # type: ignore[no-untyped-call]
        ros_msg.linear = ROSVector3(x=self.linear.x, y=self.linear.y, z=self.linear.z)  # type: ignore[no-untyped-call]
        ros_msg.angular = ROSVector3(x=self.angular.x, y=self.angular.y, z=self.angular.z)  # type: ignore[no-untyped-call]
        return ros_msg


__all__ = ["Quaternion", "Twist"]
