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

from collections.abc import Sequence
from io import BytesIO
import math
import struct
from typing import TYPE_CHECKING, Any, BinaryIO, TypeAlias, overload

if TYPE_CHECKING:
    import rerun as rr

from dimos_lcm.geometry_msgs import Quaternion as LCMQuaternion
import numpy as np

from dimos.msgs.geometry_msgs.Vector3 import Vector3

# Types that can be converted to/from Quaternion
QuaternionConvertable: TypeAlias = Sequence[int | float] | LCMQuaternion | np.ndarray


def _four_components(value: Any) -> tuple[float, float, float, float]:
    """Unpack a length-4 sequence or array into (x, y, z, w)."""
    if not isinstance(value, np.ndarray | Sequence):
        raise TypeError(f"Cannot initialize Quaternion from {type(value)}")
    size = value.size if isinstance(value, np.ndarray) else len(value)
    if size != 4:
        raise ValueError("Quaternion requires exactly 4 components [x, y, z, w]")
    return (float(value[0]), float(value[1]), float(value[2]), float(value[3]))


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

    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(
        self,
        x: float = ...,
        y: float = ...,
        z: float = ...,
        w: float = ...,
    ) -> None: ...

    @overload
    def __init__(self, value: QuaternionConvertable | Quaternion, /) -> None: ...

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        """Initialize a quaternion.

        Supported forms:
            Quaternion()                        # identity (0, 0, 0, 1)
            Quaternion(x, y, z, w)
            Quaternion(x=1, y=2, z=3, w=4)      # keyword args
            Quaternion([x, y, z, w])            # sequence
            Quaternion(np.array([x, y, z, w]))  # numpy array
            Quaternion(other_quaternion)        # copy constructor
            Quaternion(lcm_quaternion)          # from LCM message
        """
        if len(args) == 4:
            self.x = float(args[0])
            self.y = float(args[1])
            self.z = float(args[2])
            self.w = float(args[3])
        elif len(args) == 1:
            value = args[0]
            # Quaternion before LCMQuaternion (it is a subclass) and before the
            # generic sequence branch (a Quaternion is indexable).
            if isinstance(value, (Quaternion, LCMQuaternion)):
                self.x, self.y, self.z, self.w = value.x, value.y, value.z, value.w
            else:
                self.x, self.y, self.z, self.w = _four_components(value)
        elif args:
            raise TypeError(
                f"Quaternion takes 1 sequence or 4 components ({len(args)} positional given)"
            )
        elif kwargs:
            self.x = float(kwargs.pop("x", 0.0))
            self.y = float(kwargs.pop("y", 0.0))
            self.z = float(kwargs.pop("z", 0.0))
            self.w = float(kwargs.pop("w", 1.0))
        # else: no arguments — the class defaults already spell the identity.

        if kwargs:
            raise TypeError(f"Quaternion got unexpected keyword arguments {sorted(kwargs)}")

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

    def to_rerun(self) -> rr.Quaternion:
        import rerun as rr

        return rr.Quaternion(xyzw=[self.x, self.y, self.z, self.w])

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

    @classmethod
    def from_rotation_matrix(cls, matrix: np.ndarray) -> Quaternion:
        """Convert a 3x3 rotation matrix to quaternion.

        Args:
            matrix: 3x3 rotation matrix (numpy array)

        Returns:
            Quaternion representation
        """
        from scipy.spatial.transform import (
            Rotation,  # ~330ms: deferred to avoid startup cost
        )

        rotation = Rotation.from_matrix(matrix)
        quat = rotation.as_quat()  # Returns [x, y, z, w]
        return cls(quat[0], quat[1], quat[2], quat[3])

    def to_rotation_matrix(self) -> np.ndarray:
        """Convert quaternion to a 3x3 rotation matrix."""
        from scipy.spatial.transform import (
            Rotation,  # ~330ms: deferred to avoid startup cost
        )

        return np.asarray(Rotation.from_quat([self.x, self.y, self.z, self.w]).as_matrix())

    def to_euler(self) -> Vector3:
        """Convert quaternion to Euler angles (roll, pitch, yaw) in radians.

        Returns:
            Vector3: Euler angles as (roll, pitch, yaw) in radians
        """
        # Use scipy for accurate quaternion to euler conversion
        from scipy.spatial.transform import (
            Rotation,  # ~330ms: deferred to avoid startup cost
        )

        quat = [self.x, self.y, self.z, self.w]
        rotation = Rotation.from_quat(quat)
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

    def is_zero(self) -> bool:
        """All components are zero — i.e. an uninitialized placeholder, not a valid rotation."""
        return self.x == 0.0 and self.y == 0.0 and self.z == 0.0 and self.w == 0.0

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

    def dot(self, other: Quaternion) -> float:
        return float(self.x * other.x + self.y * other.y + self.z * other.z + self.w * other.w)

    def angle_to(self, other: Quaternion) -> float:
        """Smallest rotation angle (radians) between two unit quaternions.

        ``abs(self.dot(other))`` collapses the double-cover sign ambiguity;
        the ``min(1.0, ...)`` clamps against numerical drift past 1.
        """
        return 2.0 * math.acos(min(1.0, abs(self.dot(other))))

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
