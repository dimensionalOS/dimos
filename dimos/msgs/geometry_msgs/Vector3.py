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
from typing import Any, TypeAlias

from dimos_lcm.geometry_msgs import Vector3 as LCMVector3
import numpy as np

# Types that can be converted to/from Vector
VectorConvertable: TypeAlias = Sequence[int | float] | LCMVector3 | np.ndarray  # type: ignore[type-arg]


def _ensure_3d(data: np.ndarray) -> np.ndarray:  # type: ignore[type-arg]
    """Ensure the data array is exactly 3D by padding with zeros or raising an exception if too long."""
    if len(data) == 3:
        return data
    elif len(data) < 3:
        padded = np.zeros(3, dtype=float)
        padded[: len(data)] = data
        return padded
    else:
        raise ValueError(
            f"Vector3 cannot be initialized with more than 3 components. Got {len(data)} components."
        )


class Vector3(LCMVector3):  # type: ignore[misc]
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        """Initialize a 3D vector.

        Supported forms:
            Vector3()                       # zero vector
            Vector3(x)                      # (x, 0, 0)
            Vector3(x, y)                   # (x, y, 0)
            Vector3(x, y, z)                # (x, y, z)
            Vector3(x=1, y=2, z=3)          # keyword args
            Vector3([x, y, z])              # sequence
            Vector3(np.array([x, y, z]))    # numpy array
            Vector3(other_vector3)          # copy constructor
            Vector3(lcm_vector3)            # from LCM message
        """
        if kwargs and not args:
            # Keyword arguments: Vector3(x=1, y=2, z=3)
            self.x = float(kwargs.get("x", 0.0))
            self.y = float(kwargs.get("y", 0.0))
            self.z = float(kwargs.get("z", 0.0))
        elif not args:
            # No arguments: zero vector
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0
        elif len(args) == 1:
            arg = args[0]
            if isinstance(arg, Vector3):
                # Copy constructor
                self.x = arg.x
                self.y = arg.y
                self.z = arg.z
            elif isinstance(arg, LCMVector3):
                # From LCM Vector3
                self.x = float(arg.x)
                self.y = float(arg.y)
                self.z = float(arg.z)
            elif isinstance(arg, np.ndarray):
                # From numpy array
                data = _ensure_3d(np.array(arg, dtype=float))
                self.x = float(data[0])
                self.y = float(data[1])
                self.z = float(data[2])
            elif isinstance(arg, (list, tuple)):
                # From sequence
                data = _ensure_3d(np.array(arg, dtype=float))
                self.x = float(data[0])
                self.y = float(data[1])
                self.z = float(data[2])
            elif isinstance(arg, (int, float)):
                # Single numeric value: (x, 0, 0)
                self.x = float(arg)
                self.y = 0.0
                self.z = 0.0
            else:
                raise TypeError(f"Cannot initialize Vector3 from {type(arg)}")
        elif len(args) == 2:
            # Two numeric values: (x, y, 0)
            self.x = float(args[0])
            self.y = float(args[1])
            self.z = 0.0
        elif len(args) == 3:
            # Three numeric values: (x, y, z)
            self.x = float(args[0])
            self.y = float(args[1])
            self.z = float(args[2])
        else:
            raise TypeError(f"Vector3 takes at most 3 positional arguments ({len(args)} given)")

    @property
    def as_tuple(self) -> tuple[float, float, float]:
        return (self.x, self.y, self.z)

    @property
    def yaw(self) -> float:
        return self.z

    @property
    def pitch(self) -> float:
        return self.y

    @property
    def roll(self) -> float:
        return self.x

    @property
    def data(self) -> np.ndarray:  # type: ignore[type-arg]
        """Get the underlying numpy array."""
        return np.array([self.x, self.y, self.z], dtype=float)

    def __getitem__(self, idx: int) -> float:
        if idx == 0:
            return self.x
        elif idx == 1:
            return self.y
        elif idx == 2:
            return self.z
        else:
            raise IndexError(f"Vector3 index {idx} out of range [0-2]")

    def __repr__(self) -> str:
        return f"Vector({self.data})"

    def __str__(self) -> str:
        def getArrow() -> str:
            repr = ["←", "↖", "↑", "↗", "→", "↘", "↓", "↙"]

            if self.x == 0 and self.y == 0:
                return "·"

            # Calculate angle in radians and convert to directional index
            angle = np.arctan2(self.y, self.x)
            # Map angle to 0-7 index (8 directions) with proper orientation
            dir_index = int(((angle + np.pi) * 4 / np.pi) % 8)
            # Get directional arrow symbol
            return repr[dir_index]

        return f"{getArrow()} Vector {self.__repr__()}"

    def agent_encode(self) -> dict[str, float]:
        """Encode the vector for agent communication."""
        return {"x": self.x, "y": self.y, "z": self.z}

    def serialize(self) -> dict[str, Any]:
        """Serialize the vector to a tuple."""
        return {"type": "vector", "c": (self.x, self.y, self.z)}

    def __eq__(self, other: object) -> bool:
        """Check if two vectors are equal using numpy's allclose for floating point comparison."""
        if not isinstance(other, Vector3):
            return False
        return bool(np.allclose([self.x, self.y, self.z], [other.x, other.y, other.z]))

    def __add__(self, other: VectorConvertable | Vector3) -> Vector3:
        other_vector: Vector3 = to_vector(other)
        return self.__class__(
            self.x + other_vector.x, self.y + other_vector.y, self.z + other_vector.z
        )

    def __sub__(self, other: VectorConvertable | Vector3) -> Vector3:
        other_vector = to_vector(other)
        return self.__class__(
            self.x - other_vector.x, self.y - other_vector.y, self.z - other_vector.z
        )

    def __mul__(self, scalar: float) -> Vector3:
        return self.__class__(self.x * scalar, self.y * scalar, self.z * scalar)

    def __rmul__(self, scalar: float) -> Vector3:
        return self.__mul__(scalar)

    def __truediv__(self, scalar: float) -> Vector3:
        return self.__class__(self.x / scalar, self.y / scalar, self.z / scalar)

    def __neg__(self) -> Vector3:
        return self.__class__(-self.x, -self.y, -self.z)

    def dot(self, other: VectorConvertable | Vector3) -> float:
        """Compute dot product."""
        other_vector = to_vector(other)
        return float(self.x * other_vector.x + self.y * other_vector.y + self.z * other_vector.z)

    def cross(self, other: VectorConvertable | Vector3) -> Vector3:
        """Compute cross product (3D vectors only)."""
        other_vector = to_vector(other)
        return self.__class__(
            self.y * other_vector.z - self.z * other_vector.y,
            self.z * other_vector.x - self.x * other_vector.z,
            self.x * other_vector.y - self.y * other_vector.x,
        )

    def magnitude(self) -> float:
        """Alias for length()."""
        return self.length()

    def length(self) -> float:
        """Compute the Euclidean length (magnitude) of the vector."""
        return float(np.sqrt(self.x * self.x + self.y * self.y + self.z * self.z))

    def length_squared(self) -> float:
        """Compute the squared length of the vector (faster than length())."""
        return float(self.x * self.x + self.y * self.y + self.z * self.z)

    def normalize(self) -> Vector3:
        """Return a normalized unit vector in the same direction."""
        length = self.length()
        if length < 1e-10:  # Avoid division by near-zero
            return self.__class__(0.0, 0.0, 0.0)
        return self.__class__(self.x / length, self.y / length, self.z / length)

    def to_2d(self) -> Vector3:
        """Convert a vector to a 2D vector by taking only the x and y components (z=0)."""
        return self.__class__(self.x, self.y, 0.0)

    def distance(self, other: VectorConvertable | Vector3) -> float:
        """Compute Euclidean distance to another vector."""
        other_vector = to_vector(other)
        dx = self.x - other_vector.x
        dy = self.y - other_vector.y
        dz = self.z - other_vector.z
        return float(np.sqrt(dx * dx + dy * dy + dz * dz))

    def distance_squared(self, other: VectorConvertable | Vector3) -> float:
        """Compute squared Euclidean distance to another vector (faster than distance())."""
        other_vector = to_vector(other)
        dx = self.x - other_vector.x
        dy = self.y - other_vector.y
        dz = self.z - other_vector.z
        return float(dx * dx + dy * dy + dz * dz)

    def angle(self, other: VectorConvertable | Vector3) -> float:
        """Compute the angle (in radians) between this vector and another."""
        other_vector = to_vector(other)
        this_length = self.length()
        other_length = other_vector.length()

        if this_length < 1e-10 or other_length < 1e-10:
            return 0.0

        cos_angle = np.clip(
            self.dot(other_vector) / (this_length * other_length),
            -1.0,
            1.0,
        )
        return float(np.arccos(cos_angle))

    def project(self, onto: VectorConvertable | Vector3) -> Vector3:
        """Project this vector onto another vector."""
        onto_vector = to_vector(onto)
        onto_length_sq = (
            onto_vector.x * onto_vector.x
            + onto_vector.y * onto_vector.y
            + onto_vector.z * onto_vector.z
        )
        if onto_length_sq < 1e-10:
            return self.__class__(0.0, 0.0, 0.0)

        scalar_projection = self.dot(onto_vector) / onto_length_sq
        return self.__class__(
            scalar_projection * onto_vector.x,
            scalar_projection * onto_vector.y,
            scalar_projection * onto_vector.z,
        )

    @classmethod
    def zeros(cls) -> Vector3:
        """Create a zero 3D vector."""
        return cls()

    @classmethod
    def ones(cls) -> Vector3:
        """Create a 3D vector of ones."""
        return cls(1.0, 1.0, 1.0)

    @classmethod
    def unit_x(cls) -> Vector3:
        """Create a unit vector in the x direction."""
        return cls(1.0, 0.0, 0.0)

    @classmethod
    def unit_y(cls) -> Vector3:
        """Create a unit vector in the y direction."""
        return cls(0.0, 1.0, 0.0)

    @classmethod
    def unit_z(cls) -> Vector3:
        """Create a unit vector in the z direction."""
        return cls(0.0, 0.0, 1.0)

    def to_list(self) -> list[float]:
        """Convert the vector to a list."""
        return [self.x, self.y, self.z]

    def to_tuple(self) -> tuple[float, float, float]:
        """Convert the vector to a tuple."""
        return (self.x, self.y, self.z)

    def to_numpy(self) -> np.ndarray:  # type: ignore[type-arg]
        """Convert the vector to a numpy array."""
        return np.array([self.x, self.y, self.z], dtype=float)

    def is_zero(self) -> bool:
        """Check if this is a zero vector (all components are zero).

        Returns:
            True if all components are zero, False otherwise
        """
        return bool(np.allclose([self.x, self.y, self.z], 0.0))

    @property
    def quaternion(self) -> Quaternion:  # type: ignore[name-defined]
        return self.to_quaternion()

    def to_quaternion(self) -> Quaternion:  # type: ignore[name-defined]
        """Convert Vector3 representing Euler angles (roll, pitch, yaw) to a Quaternion.

        Assumes this Vector3 contains Euler angles in radians:
        - x component: roll (rotation around x-axis)
        - y component: pitch (rotation around y-axis)
        - z component: yaw (rotation around z-axis)

        Returns:
            Quaternion: The equivalent quaternion representation
        """
        # Import here to avoid circular imports
        from dimos.msgs.geometry_msgs.Quaternion import Quaternion

        # Extract Euler angles
        roll = self.x
        pitch = self.y
        yaw = self.z

        # Convert Euler angles to quaternion using ZYX convention
        # Source: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles

        # Compute half angles
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)

        # Compute quaternion components
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy

        return Quaternion(x, y, z, w)

    def __bool__(self) -> bool:
        """Boolean conversion for Vector.

        A Vector is considered False if it's a zero vector (all components are zero),
        and True otherwise.

        Returns:
            False if vector is zero, True otherwise
        """
        return not self.is_zero()


def to_numpy(value: Vector3 | np.ndarray | Sequence[int | float]) -> np.ndarray:  # type: ignore[type-arg]
    """Convert a value to a numpy array."""
    if isinstance(value, Vector3):
        return value.to_numpy()
    elif isinstance(value, np.ndarray):
        return value
    else:
        return np.array(value, dtype=float)


def to_vector(value: VectorConvertable | Vector3) -> Vector3:
    """Convert a vector-compatible value to a Vector3 object."""
    if isinstance(value, Vector3):
        return value
    return Vector3(value)


def to_tuple(value: Vector3 | np.ndarray | Sequence[int | float]) -> tuple[float, ...]:  # type: ignore[type-arg]
    """Convert a value to a tuple."""
    if isinstance(value, Vector3):
        return value.to_tuple()
    elif isinstance(value, np.ndarray):
        return tuple(value.tolist())
    elif isinstance(value, tuple):
        return value
    else:
        return tuple(value)


def to_list(value: Vector3 | np.ndarray | Sequence[int | float]) -> list[float]:  # type: ignore[type-arg]
    """Convert a value to a list."""
    if isinstance(value, Vector3):
        return value.to_list()
    elif isinstance(value, np.ndarray):
        result: list[float] = value.tolist()
        return result
    elif isinstance(value, list):
        return value
    else:
        return list(value)


VectorLike: TypeAlias = VectorConvertable | Vector3


def make_vector3(x: float, y: float, z: float) -> Vector3:
    return Vector3(x, y, z)
