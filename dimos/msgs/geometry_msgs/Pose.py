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

from typing import TypeAlias

from dimos_lcm.geometry_msgs import (  # type: ignore[import-untyped]
    Pose as LCMPose,
    Transform as LCMTransform,
)

try:
    from geometry_msgs.msg import (  # type: ignore[attr-defined, import-untyped]
        Point as ROSPoint,
        Pose as ROSPose,
        Quaternion as ROSQuaternion,
    )
except ImportError:
    ROSPose = None  # type: ignore[assignment, misc]
    ROSPoint = None  # type: ignore[assignment, misc]
    ROSQuaternion = None  # type: ignore[assignment, misc]

from plum import dispatch

from dimos.msgs.geometry_msgs.Quaternion import Quaternion, QuaternionConvertable
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3, VectorConvertable

# Types that can be converted to/from Pose
PoseConvertable: TypeAlias = (
    tuple[VectorConvertable, QuaternionConvertable]
    | LCMPose
    | Vector3
    | dict[str, VectorConvertable | QuaternionConvertable]
)


class Pose(LCMPose):  # type: ignore[misc]
    position: Vector3
    orientation: Quaternion
    msg_name = "geometry_msgs.Pose"

    @dispatch
    def __init__(self) -> None:
        """Initialize a pose at origin with identity orientation."""
        self.position = Vector3(0.0, 0.0, 0.0)
        self.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)

    @dispatch  # type: ignore[no-redef]
    def __init__(self, x: int | float, y: int | float, z: int | float) -> None:
        """Initialize a pose with position and identity orientation."""
        self.position = Vector3(x, y, z)
        self.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)

    @dispatch  # type: ignore[no-redef]
    def __init__(
        self,
        x: int | float,
        y: int | float,
        z: int | float,
        qx: int | float,
        qy: int | float,
        qz: int | float,
        qw: int | float,
    ) -> None:
        """Initialize a pose with position and orientation."""
        self.position = Vector3(x, y, z)
        self.orientation = Quaternion(qx, qy, qz, qw)

    @dispatch  # type: ignore[no-redef]
    def __init__(
        self,
        position: VectorConvertable | Vector3 | None = None,
        orientation: QuaternionConvertable | Quaternion | None = None,
    ) -> None:
        """Initialize a pose with position and orientation."""
        if orientation is None:
            orientation = [0, 0, 0, 1]
        if position is None:
            position = [0, 0, 0]
        self.position = Vector3(position)
        self.orientation = Quaternion(orientation)

    @dispatch  # type: ignore[no-redef]
    def __init__(self, pose_tuple: tuple[VectorConvertable, QuaternionConvertable]) -> None:
        """Initialize from a tuple of (position, orientation)."""
        self.position = Vector3(pose_tuple[0])
        self.orientation = Quaternion(pose_tuple[1])

    @dispatch  # type: ignore[no-redef]
    def __init__(self, pose_dict: dict[str, VectorConvertable | QuaternionConvertable]) -> None:
        """Initialize from a dictionary with 'position' and 'orientation' keys."""
        self.position = Vector3(pose_dict["position"])
        self.orientation = Quaternion(pose_dict["orientation"])

    @dispatch  # type: ignore[no-redef]
    def __init__(self, pose: Pose) -> None:
        """Initialize from another Pose (copy constructor)."""
        self.position = Vector3(pose.position)
        self.orientation = Quaternion(pose.orientation)

    @dispatch  # type: ignore[no-redef]
    def __init__(self, lcm_pose: LCMPose) -> None:
        """Initialize from an LCM Pose."""
        self.position = Vector3(lcm_pose.position.x, lcm_pose.position.y, lcm_pose.position.z)
        self.orientation = Quaternion(
            lcm_pose.orientation.x,
            lcm_pose.orientation.y,
            lcm_pose.orientation.z,
            lcm_pose.orientation.w,
        )

    @property
    def x(self) -> float:
        """X coordinate of position."""
        return self.position.x

    @property
    def y(self) -> float:
        """Y coordinate of position."""
        return self.position.y

    @property
    def z(self) -> float:
        """Z coordinate of position."""
        return self.position.z

    @property
    def roll(self) -> float:
        """Roll angle in radians."""
        return self.orientation.to_euler().roll

    @property
    def pitch(self) -> float:
        """Pitch angle in radians."""
        return self.orientation.to_euler().pitch

    @property
    def yaw(self) -> float:
        """Yaw angle in radians."""
        return self.orientation.to_euler().yaw

    def __repr__(self) -> str:
        return f"Pose(position={self.position!r}, orientation={self.orientation!r})"

    def __str__(self) -> str:
        return (
            f"Pose(pos=[{self.x:.3f}, {self.y:.3f}, {self.z:.3f}], "
            f"euler=[{self.roll:.3f}, {self.pitch:.3f}, {self.yaw:.3f}]), "
            f"quaternion=[{self.orientation}])"
        )

    def __eq__(self, other) -> bool:  # type: ignore[no-untyped-def]
        """Check if two poses are equal."""
        if not isinstance(other, Pose):
            return False
        return self.position == other.position and self.orientation == other.orientation

    def __matmul__(self, transform: LCMTransform | Transform) -> Pose:
        return self + transform

    def __add__(self, other: Pose | PoseConvertable | LCMTransform | Transform) -> Pose:
        """Compose two poses or apply a transform (transform composition).

        The operation self + other represents applying transformation 'other'
        in the coordinate frame defined by 'self'. This is equivalent to:
        - First apply transformation 'self' (from world to self's frame)
        - Then apply transformation 'other' (from self's frame to other's frame)

        This matches ROS tf convention where:
        T_world_to_other = T_world_to_self * T_self_to_other

        Args:
            other: The pose or transform to compose with this one

        Returns:
            A new Pose representing the composed transformation

        Example:
            robot_pose = Pose(1, 0, 0)  # Robot at (1,0,0) facing forward
            object_in_robot = Pose(2, 0, 0)  # Object 2m in front of robot
            object_in_world = robot_pose + object_in_robot  # Object at (3,0,0) in world

            # Or with a Transform:
            transform = Transform()
            transform.translation = Vector3(2, 0, 0)
            transform.rotation = Quaternion(0, 0, 0, 1)
            new_pose = pose + transform
        """
        # Handle Transform objects
        if isinstance(other, LCMTransform | Transform):
            # Convert Transform to Pose using its translation and rotation
            other_position = Vector3(other.translation)
            other_orientation = Quaternion(other.rotation)
        elif isinstance(other, Pose):
            other_position = other.position
            other_orientation = other.orientation
        else:
            # Convert to Pose if it's a convertible type
            other_pose = Pose(other)
            other_position = other_pose.position
            other_orientation = other_pose.orientation

        # Compose orientations: self.orientation * other.orientation
        new_orientation = self.orientation * other_orientation

        # Transform other's position by self's orientation, then add to self's position
        rotated_position = self.orientation.rotate_vector(other_position)
        new_position = self.position + rotated_position

        return Pose(new_position, new_orientation)

    @classmethod
    def from_ros_msg(cls, ros_msg: ROSPose) -> Pose:
        """Create a Pose from a ROS geometry_msgs/Pose message.

        Args:
            ros_msg: ROS Pose message

        Returns:
            Pose instance
        """
        position = Vector3(ros_msg.position.x, ros_msg.position.y, ros_msg.position.z)
        orientation = Quaternion(
            ros_msg.orientation.x,
            ros_msg.orientation.y,
            ros_msg.orientation.z,
            ros_msg.orientation.w,
        )
        return cls(position, orientation)

    def to_ros_msg(self) -> ROSPose:
        """Convert to a ROS geometry_msgs/Pose message.

        Returns:
            ROS Pose message
        """
        ros_msg = ROSPose()  # type: ignore[no-untyped-call]
        ros_msg.position = ROSPoint(  # type: ignore[no-untyped-call]
            x=float(self.position.x), y=float(self.position.y), z=float(self.position.z)
        )
        ros_msg.orientation = ROSQuaternion(  # type: ignore[no-untyped-call]
            x=float(self.orientation.x),
            y=float(self.orientation.y),
            z=float(self.orientation.z),
            w=float(self.orientation.w),
        )
        return ros_msg


@dispatch
def to_pose(value: Pose) -> Pose:
    """Pass through Pose objects."""
    return value


@dispatch  # type: ignore[no-redef]
def to_pose(value: PoseConvertable) -> Pose:
    """Convert a pose-compatible value to a Pose object."""
    return Pose(value)


PoseLike: TypeAlias = PoseConvertable | Pose
