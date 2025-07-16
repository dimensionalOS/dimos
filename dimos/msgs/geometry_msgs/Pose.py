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

import struct
import traceback
from io import BytesIO
from typing import BinaryIO, TypeAlias

from dimos_lcm.geometry_msgs import Pose as LCMPose
from plum import dispatch

from dimos.msgs.geometry_msgs.Quaternion import Quaternion, QuaternionConvertable
from dimos.msgs.geometry_msgs.Vector3 import Vector3, VectorConvertable

# Types that can be converted to/from Pose
PoseConvertable: TypeAlias = (
    tuple[VectorConvertable, QuaternionConvertable]
    | LCMPose
    | dict[str, VectorConvertable | QuaternionConvertable]
)


class Pose(LCMPose):
    position: Vector3
    orientation: Quaternion
    msg_name = "geometry_msgs.Pose"

    @classmethod
    def lcm_decode(cls, data: bytes | BinaryIO):
        if not hasattr(data, "read"):
            data = BytesIO(data)
        if data.read(8) != cls._get_packed_fingerprint():
            traceback.print_exc()
            raise ValueError("Decode error")
        return cls._lcm_decode_one(data)

    @classmethod
    def _lcm_decode_one(cls, buf):
        return cls(Vector3._decode_one(buf), Quaternion._decode_one(buf))

    def lcm_encode(self) -> bytes:
        return super().encode()

    @dispatch
    def __init__(self) -> None:
        """Initialize a pose at origin with identity orientation."""
        self.position = Vector3(0.0, 0.0, 0.0)
        self.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)

    @dispatch
    def __init__(self, x: int | float, y: int | float, z: int | float) -> None:
        """Initialize a pose with position and identity orientation."""
        self.position = Vector3(x, y, z)
        self.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)

    @dispatch
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

    @dispatch
    def __init__(
        self,
        position: VectorConvertable | Vector3 = [0, 0, 0],
        orientation: QuaternionConvertable | Quaternion = [0, 0, 0, 1],
    ) -> None:
        """Initialize a pose with position and orientation."""
        self.position = Vector3(position)
        self.orientation = Quaternion(orientation)

    @dispatch
    def __init__(self, pose_tuple: tuple[VectorConvertable, QuaternionConvertable]) -> None:
        """Initialize from a tuple of (position, orientation)."""
        self.position = Vector3(pose_tuple[0])
        self.orientation = Quaternion(pose_tuple[1])

    @dispatch
    def __init__(self, pose_dict: dict[str, VectorConvertable | QuaternionConvertable]) -> None:
        """Initialize from a dictionary with 'position' and 'orientation' keys."""
        self.position = Vector3(pose_dict["position"])
        self.orientation = Quaternion(pose_dict["orientation"])

    @dispatch
    def __init__(self, pose: Pose) -> None:
        """Initialize from another Pose (copy constructor)."""
        self.position = Vector3(pose.position)
        self.orientation = Quaternion(pose.orientation)

    @dispatch
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
            f"euler=[{self.roll:.3f}, {self.pitch:.3f}, {self.yaw:.3f}])"
        )

    def __eq__(self, other) -> bool:
        """Check if two poses are equal."""
        if not isinstance(other, Pose):
            return False
        return self.position == other.position and self.orientation == other.orientation


@dispatch
def to_pose(value: "Pose") -> Pose:
    """Pass through Pose objects."""
    return value


@dispatch
def to_pose(value: PoseConvertable | Pose) -> Pose:
    """Convert a pose-compatible value to a Pose object."""
    return Pose(value)


PoseLike: TypeAlias = PoseConvertable | Pose
