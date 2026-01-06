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

from collections.abc import Sequence
from io import BytesIO
import struct
from typing import BinaryIO, TypeAlias

from dimos_lcm.geometry_msgs import Quaternion as LCMQuaternion  # type: ignore[import-untyped]
import numpy as np
from plum import dispatch
from scipy.spatial.transform import Rotation as R  # type: ignore[import-untyped]

from dimos.msgs.geometry_msgs.Vector3 import Vector3

# Types that can be converted to/from Quaternion
QuaternionConvertable: TypeAlias = Sequence[int | float] | LCMQuaternion | np.ndarray  # type: ignore[type-arg]


class Quaternion(LCMQuaternion):  # type: ignore[misc]
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 1.0
    msg_name = "geometry_msgs.Quaternion"

    @classmethod
    def lcm_decode(cls, data: bytes | BinaryIO):  # type: ignore[no-untyped-def]
        if not hasattr(data, "read"):
            data = BytesIO(data)
        if data.read(8) != cls._get_packed_fingerprint():
            raise ValueError("Decode error")
        return cls._lcm_decode_one(data)  # type: ignore[no-untyped-call]

    @classmethod
    def _lcm_decode_one(cls, buf):  # type: ignore[no-untyped-def]
        return cls(struct.unpack(">dddd", buf.read(32)))

    @dispatch
    def __init__(self) -> None: ...

    @dispatch  # type: ignore[no-redef]
    def __init__(self, x: int | float, y: int | float, z: int | float, w: int | float) -> None:
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)
        self.w = float(w)

    @dispatch  # type: ignore[no-redef]
    def __init__(self, sequence: Sequence[int | float] | np.ndarray) -> None:  # type: ignore[type-arg]
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

    @dispatch  # type: ignore[no-redef]
    def __init__(self, quaternion: Quaternion) -> None:
        """Initialize from another Quaternion (copy constructor)."""
        self.x, self.y, self.z, self.w = quaternion.x, quaternion.y, quaternion.z, quaternion.w

    @dispatch  # type: ignore[no-redef]
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

    def to_numpy(self) -> np.ndarray:  # type: ignore[type-arg]
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

    @classmethod
    def from_euler(cls, vector: Vector3) -> Quaternion:
        """Convert Euler angles (roll, pitch, yaw) in radians to quaternion.

        Args:
            vector: Vector3 containing (roll, pitch, yaw) in radians

        Returns:
            Quaternion representation
        """

        # Calculate quaternion components
        cy = np.cos(vector.yaw * 0.5)
        sy = np.sin(vector.yaw * 0.5)
        cp = np.cos(vector.pitch * 0.5)
        sp = np.sin(vector.pitch * 0.5)
        cr = np.cos(vector.roll * 0.5)
        sr = np.sin(vector.roll * 0.5)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy

        return cls(x, y, z, w)

    def to_euler(self) -> Vector3:
        """Convert quaternion to Euler angles (roll, pitch, yaw) in radians.

        Returns:
            Vector3: Euler angles as (roll, pitch, yaw) in radians
        """
        # Use scipy for accurate quaternion to euler conversion
        quat = [self.x, self.y, self.z, self.w]
        rotation = R.from_quat(quat)
        euler_angles = rotation.as_euler("xyz")  # roll, pitch, yaw

        return Vector3(euler_angles[0], euler_angles[1], euler_angles[2])

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

    def __eq__(self, other) -> bool:  # type: ignore[no-untyped-def]
        if not isinstance(other, Quaternion):
            return False
        return self.x == other.x and self.y == other.y and self.z == other.z and self.w == other.w

    def __mul__(self, other: Quaternion) -> Quaternion:
        """Multiply two quaternions (Hamilton product).

        The result represents the composition of rotations:
        q1 * q2 represents rotating by q2 first, then by q1.
        """
        if not isinstance(other, Quaternion):
            raise TypeError(f"Cannot multiply Quaternion with {type(other)}")

        # Hamilton product formula
        w = self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z
        x = self.w * other.x + self.x * other.w + self.y * other.z - self.z * other.y
        y = self.w * other.y - self.x * other.z + self.y * other.w + self.z * other.x
        z = self.w * other.z + self.x * other.y - self.y * other.x + self.z * other.w

        return Quaternion(x, y, z, w)

    def conjugate(self) -> Quaternion:
        """Return the conjugate of the quaternion.

        For unit quaternions, the conjugate represents the inverse rotation.
        """
        return Quaternion(-self.x, -self.y, -self.z, self.w)

    def inverse(self) -> Quaternion:
        """Return the inverse of the quaternion.

        For unit quaternions, this is equivalent to the conjugate.
        For non-unit quaternions, this is conjugate / norm^2.
        """
        norm_sq = self.x**2 + self.y**2 + self.z**2 + self.w**2
        if norm_sq == 0:
            raise ZeroDivisionError("Cannot invert zero quaternion")

        # For unit quaternions (norm_sq ≈ 1), this simplifies to conjugate
        if np.isclose(norm_sq, 1.0):
            return self.conjugate()

        # For non-unit quaternions
        conj = self.conjugate()
        return Quaternion(conj.x / norm_sq, conj.y / norm_sq, conj.z / norm_sq, conj.w / norm_sq)

    def normalize(self) -> Quaternion:
        """Return a normalized (unit) quaternion."""
        norm = np.sqrt(self.x**2 + self.y**2 + self.z**2 + self.w**2)
        if norm == 0:
            raise ZeroDivisionError("Cannot normalize zero quaternion")
        return Quaternion(self.x / norm, self.y / norm, self.z / norm, self.w / norm)

    def rotate_vector(self, vector: Vector3) -> Vector3:
        """Rotate a 3D vector by this quaternion.

        Args:
            vector: The vector to rotate

        Returns:
            The rotated vector
        """
        # For unit quaternions, conjugate equals inverse, so we use conjugate for efficiency
        # The rotation formula is: q * v * q^* where q^* is the conjugate

        # Convert vector to pure quaternion (w=0)
        v_quat = Quaternion(vector.x, vector.y, vector.z, 0)

        # Apply rotation: q * v * q^* (conjugate for unit quaternions)
        rotated = self * v_quat * self.conjugate()

        # Extract vector components
        return Vector3(rotated.x, rotated.y, rotated.z)
