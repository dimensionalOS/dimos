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
from collections.abc import Sequence
from io import BytesIO
from typing import BinaryIO, TypeAlias

import numpy as np
from dimos_lcm.geometry_msgs import Quaternion as LCMQuaternion
from plum import dispatch

from dimos.msgs.geometry_msgs.Vector3 import Vector3

# Types that can be converted to/from Quaternion
QuaternionConvertable: TypeAlias = Sequence[int | float] | LCMQuaternion | np.ndarray


class Quaternion(LCMQuaternion):
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 1.0
    name = "geometry_msgs.Quaternion"

    @classmethod
    def lcm_decode(cls, data: bytes | BinaryIO):
        if not hasattr(data, "read"):
            data = BytesIO(data)
        if data.read(8) != cls._get_packed_fingerprint():
            raise ValueError("Decode error")
        return cls._lcm_decode_one(data)

    @classmethod
    def _lcm_decode_one(cls, buf):
        return cls(struct.unpack(">dddd", buf.read(32)))

    def lcm_encode(self):
        return super().encode()

    @dispatch
    def __init__(self) -> None: ...

    @dispatch
    def __init__(self, x: int | float, y: int | float, z: int | float, w: int | float) -> None:
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)
        self.w = float(w)

    @dispatch
    def __init__(self, sequence: Sequence[int | float] | np.ndarray) -> None:
        if isinstance(sequence, np.ndarray):
            if sequence.size != 4:
                raise ValueError("Quaternion requires exactly 4 components [x, y, z, w]")
        else:
            if len(sequence) != 4:
                raise ValueError("Quaternion requires exactly 4 components [x, y, z, w]")

        self.x = sequence[0]
        self.y = sequence[1]
        self.z = sequence[2]
        self.w = sequence[3]

    @dispatch
    def __init__(self, quaternion: "Quaternion") -> None:
        """Initialize from another Quaternion (copy constructor)."""
        self.x, self.y, self.z, self.w = quaternion.x, quaternion.y, quaternion.z, quaternion.w

    @dispatch
    def __init__(self, lcm_quaternion: LCMQuaternion) -> None:
        """Initialize from an LCM Quaternion."""
        self.x, self.y, self.z, self.w = (
            lcm_quaternion.x,
            lcm_quaternion.y,
            lcm_quaternion.z,
            lcm_quaternion.w,
        )

    def to_tuple(self) -> tuple[float, float, float, float]:
        """Tuple representation of the quaternion (x, y, z, w)."""
        return (self.x, self.y, self.z, self.w)

    def to_list(self) -> list[float]:
        """List representation of the quaternion (x, y, z, w)."""
        return [self.x, self.y, self.z, self.w]

    def to_numpy(self) -> np.ndarray:
        """Numpy array representation of the quaternion (x, y, z, w)."""
        return np.array([self.x, self.y, self.z, self.w])

    @property
    def euler(self) -> Vector3:
        return self.to_euler()

    @property
    def radians(self) -> Vector3:
        return self.to_euler()

    def to_radians(self) -> Vector3:
        """Radians representation of the quaternion (x, y, z, w)."""
        return self.to_euler()

    def to_euler(self) -> Vector3:
        """Convert quaternion to Euler angles (roll, pitch, yaw) in radians.

        Returns:
            Vector3: Euler angles as (roll, pitch, yaw) in radians
        """
        # Convert quaternion to Euler angles using ZYX convention (yaw, pitch, roll)
        # Source: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles

        # Roll (x-axis rotation)
        sinr_cosp = 2 * (self.w * self.x + self.y * self.z)
        cosr_cosp = 1 - 2 * (self.x * self.x + self.y * self.y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (self.w * self.y - self.z * self.x)
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = np.arcsin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (self.w * self.z + self.x * self.y)
        cosy_cosp = 1 - 2 * (self.y * self.y + self.z * self.z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return Vector3(roll, pitch, yaw)

    def __getitem__(self, idx: int) -> float:
        """Allow indexing into quaternion components: 0=x, 1=y, 2=z, 3=w."""
        if idx == 0:
            return self.x
        elif idx == 1:
            return self.y
        elif idx == 2:
            return self.z
        elif idx == 3:
            return self.w
        else:
            raise IndexError(f"Quaternion index {idx} out of range [0-3]")

    def __repr__(self) -> str:
        return f"Quaternion({self.x:.6f}, {self.y:.6f}, {self.z:.6f}, {self.w:.6f})"

    def __str__(self) -> str:
        return self.__repr__()

    def __eq__(self, other) -> bool:
        if not isinstance(other, Quaternion):
            return False
        return self.x == other.x and self.y == other.y and self.z == other.z and self.w == other.w
