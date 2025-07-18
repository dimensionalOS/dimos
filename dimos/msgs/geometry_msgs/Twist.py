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
from io import BytesIO
from typing import BinaryIO

from dimos_lcm.geometry_msgs import Twist as LCMTwist
from plum import dispatch

from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3, VectorLike


class Twist(LCMTwist):
    linear: Vector3
    angular: Vector3
    msg_name = "geometry_msgs.Twist"

    @dispatch
    def __init__(self) -> None:
        """Initialize a zero twist (no linear or angular velocity)."""
        self.linear = Vector3()
        self.angular = Vector3()

    @dispatch
    def __init__(self, linear: VectorLike, angular: VectorLike) -> None:
        """Initialize a twist from linear and angular velocities."""

        self.linear = Vector3(linear)
        self.angular = Vector3(angular)

    @dispatch
    def __init__(self, linear: VectorLike, angular: Quaternion) -> None:
        """Initialize a twist from linear velocity and angular as quaternion (converted to euler)."""
        self.linear = Vector3(linear)
        self.angular = angular.to_euler()

    @dispatch
    def __init__(self, twist: "Twist") -> None:
        """Initialize from another Twist (copy constructor)."""
        self.linear = Vector3(twist.linear)
        self.angular = Vector3(twist.angular)

    @dispatch
    def __init__(self, lcm_twist: LCMTwist) -> None:
        """Initialize from an LCM Twist."""
        self.linear = Vector3(lcm_twist.linear)
        self.angular = Vector3(lcm_twist.angular)

    @dispatch
    def __init__(self, **kwargs):
        """Handle keyword arguments for LCM compatibility."""
        linear = kwargs.get("linear", Vector3())
        angular = kwargs.get("angular", Vector3())

        self.__init__(linear, angular)

    @classmethod
    def lcm_decode(cls, data: bytes | BinaryIO):
        if not hasattr(data, "read"):
            data = BytesIO(data)
        if data.read(8) != cls._get_packed_fingerprint():
            raise ValueError("Decode error")
        return cls._lcm_decode_one(data)

    @classmethod
    def _lcm_decode_one(cls, buf):
        linear = Vector3._lcm_decode_one(buf)
        angular = Vector3._lcm_decode_one(buf)
        return cls(linear=linear, angular=angular)

    def lcm_encode(self) -> bytes:
        return super().encode()

    def __repr__(self) -> str:
        return f"Twist(linear={self.linear!r}, angular={self.angular!r})"

    def __str__(self) -> str:
        return f"Twist:\n  Linear: {self.linear}\n  Angular: {self.angular}"

    def __eq__(self, other) -> bool:
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
