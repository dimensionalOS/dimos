# Copyright 2025-2026 Dimensional Inc.
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

from typing import Any, TypeAlias, overload

from dimos_lcm.geometry_msgs import (
    Pose as LCMPose,
    Transform as LCMTransform,
)
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


def _is_position_orientation_pair(value: Any) -> bool:
    """True for a 2-element (position, orientation) pair, false for a 2D position."""
    return (
        isinstance(value, tuple | list)
        and len(value) == 2
        and not isinstance(value[0], int | float)
        and not isinstance(value[1], int | float)
    )


class Pose(LCMPose):  # type: ignore[misc]
    position: Vector3
    orientation: Quaternion
    msg_name = "geometry_msgs.Pose"

    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(self, x: float, y: float, z: float) -> None: ...

    @overload
    def __init__(
        self,
        x: float,
        y: float,
        z: float,
        qx: float,
        qy: float,
        qz: float,
        qw: float,
    ) -> None: ...

    @overload
    def __init__(
        self,
        position: VectorConvertable | Vector3 | None = ...,
        orientation: QuaternionConvertable | Quaternion | None = ...,
    ) -> None: ...

    @overload
    def __init__(self, value: PoseConvertable | Pose, /) -> None: ...

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        """Initialize a pose.

        Supported forms:
            Pose()                                  # origin, identity orientation
            Pose(x, y, z)
            Pose(x, y, z, qx, qy, qz, qw)
            Pose(position)                          # any Vector3-convertable
            Pose(position, orientation)
            Pose(position=..., orientation=...)     # keyword args
            Pose((position, orientation))           # pair
            Pose({"position": ..., "orientation": ...})
            Pose(other_pose)                        # copy constructor
            Pose(lcm_pose)                          # from LCM message
        """
        position: Any = None
        orientation: Any = None

        if len(args) == 3:
            position = args
        elif len(args) == 7:
            position, orientation = args[:3], args[3:]
        elif len(args) == 2:
            position, orientation = args
        elif len(args) == 1:
            value = args[0]
            # Pose before LCMPose (it is a subclass); dict and pair before the
            # generic "it is a position" fallback.
            if isinstance(value, Pose):
                position, orientation = value.position, value.orientation
            elif isinstance(value, LCMPose):
                position = (value.position.x, value.position.y, value.position.z)
                orientation = (
                    value.orientation.x,
                    value.orientation.y,
                    value.orientation.z,
                    value.orientation.w,
                )
            elif isinstance(value, dict):
                position, orientation = value["position"], value["orientation"]
            elif _is_position_orientation_pair(value):
                position, orientation = value
            else:
                position = value
        elif args:
            raise TypeError(f"Pose takes 1, 2, 3 or 7 positional arguments ({len(args)} given)")

        if kwargs:
            position = kwargs.pop("position", position)
            orientation = kwargs.pop("orientation", orientation)
            if kwargs:
                raise TypeError(f"Pose got unexpected keyword arguments {sorted(kwargs)}")

        self.position = Vector3(0.0, 0.0, 0.0) if position is None else Vector3(position)
        self.orientation = (
            Quaternion(0.0, 0.0, 0.0, 1.0) if orientation is None else Quaternion(orientation)
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

    def __sub__(self, other: Pose) -> Pose:
        """Compute the delta pose: self - other.

        For position: simple subtraction.
        For orientation: delta_quat = self.orientation * inverse(other.orientation)

        Returns:
            A new Pose representing the delta transformation
        """
        delta_position = self.position - other.position
        delta_orientation = self.orientation * other.orientation.inverse()
        return Pose(delta_position, delta_orientation)


@dispatch
def to_pose(value: Pose) -> Pose:
    """Pass through Pose objects."""
    return value


@dispatch  # type: ignore[no-redef]
def to_pose(value: PoseConvertable) -> Pose:
    """Convert a pose-compatible value to a Pose object."""
    return Pose(value)


PoseLike: TypeAlias = PoseConvertable | Pose
