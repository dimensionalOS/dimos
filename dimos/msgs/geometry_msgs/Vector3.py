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
from typing import ForwardRef, overload

import numpy as np
from lcm_msgs.geometry_msgs import Vector3 as LCMVector3
from plum import dispatch

# Vector-like types that can be converted to/from Vector
VectorLike = Sequence[int | float] | LCMVector3 | ForwardRef("Vector3") | np.ndarray


class Vector3(LCMVector3):
    name = "geometry_msgs.Vector3"
    _data: np.ndarray

    @dispatch
    def __init__(self) -> None:
        """Initialize an empty vector."""
        self._data = np.array([], dtype=float)

    @dispatch
    def __init__(self, x: int | float) -> None:
        """Initialize a 1D vector from a single numeric value."""
        self._data = np.array([float(x)], dtype=float)

    @dispatch
    def __init__(self, x: int | float, y: int | float) -> None:
        """Initialize a 2D vector from x, y components."""
        self._data = np.array([float(x), float(y)], dtype=float)

    @dispatch
    def __init__(self, x: int | float, y: int | float, z: int | float) -> None:
        """Initialize a 3D vector from x, y, z components."""
        self._data = np.array([float(x), float(y), float(z)], dtype=float)

    @dispatch
    def __init__(self, sequence: Sequence[int | float]) -> None:
        """Initialize from a sequence (list, tuple) of numbers."""
        self._data = np.array(sequence, dtype=float)

    @dispatch
    def __init__(self, array: np.ndarray) -> None:
        """Initialize from a numpy array."""
        self._data = np.array(array, dtype=float)

    @dispatch
    def __init__(self, vector: "Vector3") -> None:
        """Initialize from another Vector3 (copy constructor)."""
        self._data = np.array([vector.x, vector.y, vector.z], dtype=float)

    @dispatch
    def __init__(self, lcm_vector: LCMVector3) -> None:
        """Initialize from an LCM Vector3."""
        self._data = np.array([lcm_vector.x, lcm_vector.y, lcm_vector.z], dtype=float)

    @property
    def yaw(self) -> float:
        return self.x

    @property
    def as_tuple(self) -> tuple[float, ...]:
        """Tuple representation of the vector."""
        return tuple(self._data)

    @property
    def x(self) -> float:
        """X component of the vector."""
        return self._data[0] if len(self._data) > 0 else 0.0

    @property
    def y(self) -> float:
        """Y component of the vector."""
        return self._data[1] if len(self._data) > 1 else 0.0

    @property
    def z(self) -> float:
        """Z component of the vector."""
        return self._data[2] if len(self._data) > 2 else 0.0

    @property
    def dim(self) -> int:
        """Dimensionality of the vector."""
        return len(self._data)

    @property
    def data(self) -> np.ndarray:
        """Get the underlying numpy array."""
        return self._data

    def __getitem__(self, idx):
        return self._data[idx]

    def __repr__(self) -> str:
        return f"Vector({self.data})"

    def __str__(self) -> str:
        if self.dim < 2:
            return self.__repr__()

        def getArrow():
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

    def serialize(self) -> dict:
        """Serialize the vector to a tuple."""
        return {"type": "vector", "c": tuple(self._data.tolist())}

    def __eq__(self, other) -> bool:
        """Check if two vectors are equal using numpy's allclose for floating point comparison."""
        if not isinstance(other, Vector3):
            return False
        if len(self._data) != len(other._data):
            return False
        return np.allclose(self._data, other._data)

    def __add__(self: Vector3, other: VectorLike) -> Vector3:
        other: Vector3 = to_vector(other)
        if self.dim != other.dim:
            max_dim = max(self.dim, other.dim)
            return self.pad(max_dim) + other.pad(max_dim)
        return self.__class__(self._data + other._data)

    def __sub__(self, other: VectorLike) -> Vector3:
        other = to_vector(other)
        if self.dim != other.dim:
            max_dim = max(self.dim, other.dim)
            return self.pad(max_dim) - other.pad(max_dim)
        return self.__class__(self._data - other._data)

    def __mul__(self, scalar: float) -> Vector3:
        return self.__class__(self._data * scalar)

    def __rmul__(self, scalar: float) -> Vector3:
        return self.__mul__(scalar)

    def __truediv__(self, scalar: float) -> Vector3:
        return self.__class__(self._data / scalar)

    def __neg__(self) -> Vector3:
        return self.__class__(-self._data)

    def dot(self, other: VectorLike) -> float:
        """Compute dot product."""
        other = to_vector(other)
        return float(np.dot(self._data, other._data))

    def cross(self, other: VectorLike) -> Vector3:
        """Compute cross product (3D vectors only)."""
        if self.dim != 3:
            raise ValueError("Cross product is only defined for 3D vectors")

        other = to_vector(other)
        if other.dim != 3:
            raise ValueError("Cross product requires two 3D vectors")

        return self.__class__(np.cross(self._data, other._data))

    def length(self) -> float:
        """Compute the Euclidean length (magnitude) of the vector."""
        return float(np.linalg.norm(self._data))

    def length_squared(self) -> float:
        """Compute the squared length of the vector (faster than length())."""
        return float(np.sum(self._data * self._data))

    def normalize(self) -> Vector3:
        """Return a normalized unit vector in the same direction."""
        length = self.length()
        if length < 1e-10:  # Avoid division by near-zero
            return self.__class__(np.zeros_like(self._data))
        return self.__class__(self._data / length)

    def to_2d(self) -> Vector3:
        """Convert a vector to a 2D vector by taking only the x and y components."""
        return self.__class__(self._data[:2])

    def pad(self, dim: int) -> Vector3:
        """Pad a vector with zeros to reach the specified dimension.

        If vector already has dimension >= dim, it is returned unchanged.
        """
        if self.dim >= dim:
            return self

        padded = np.zeros(dim, dtype=float)
        padded[: len(self._data)] = self._data
        return self.__class__(padded)

    def distance(self, other: VectorLike) -> float:
        """Compute Euclidean distance to another vector."""
        other = to_vector(other)
        return float(np.linalg.norm(self._data - other._data))

    def distance_squared(self, other: VectorLike) -> float:
        """Compute squared Euclidean distance to another vector (faster than distance())."""
        other = to_vector(other)
        diff = self._data - other._data
        return float(np.sum(diff * diff))

    def angle(self, other: VectorLike) -> float:
        """Compute the angle (in radians) between this vector and another."""
        other = to_vector(other)
        if self.length() < 1e-10 or other.length() < 1e-10:
            return 0.0

        cos_angle = np.clip(
            np.dot(self._data, other._data)
            / (np.linalg.norm(self._data) * np.linalg.norm(other._data)),
            -1.0,
            1.0,
        )
        return float(np.arccos(cos_angle))

    def project(self, onto: VectorLike) -> Vector3:
        """Project this vector onto another vector."""
        onto = to_vector(onto)
        onto_length_sq = np.sum(onto._data * onto._data)
        if onto_length_sq < 1e-10:
            return self.__class__(np.zeros_like(self._data))

        scalar_projection = np.dot(self._data, onto._data) / onto_length_sq
        return self.__class__(scalar_projection * onto._data)

    # this is here to test ros_observable_topic
    # doesn't happen irl afaik that we want a vector from ros message
    @classmethod
    def from_msg(cls, msg) -> Vector3:
        return cls(*msg)

    @classmethod
    def zeros(cls, dim: int) -> Vector3:
        """Create a zero vector of given dimension."""
        return cls(np.zeros(dim))

    @classmethod
    def ones(cls, dim: int) -> Vector3:
        """Create a vector of ones with given dimension."""
        return cls(np.ones(dim))

    @classmethod
    def unit_x(cls, dim: int = 3) -> Vector3:
        """Create a unit vector in the x direction."""
        v = np.zeros(dim)
        v[0] = 1.0
        return cls(v)

    @classmethod
    def unit_y(cls, dim: int = 3) -> Vector3:
        """Create a unit vector in the y direction."""
        v = np.zeros(dim)
        v[1] = 1.0
        return cls(v)

    @classmethod
    def unit_z(cls, dim: int = 3) -> Vector3:
        """Create a unit vector in the z direction."""
        v = np.zeros(dim)
        if dim > 2:
            v[2] = 1.0
        return cls(v)

    def to_list(self) -> list[float]:
        """Convert the vector to a list."""
        return self._data.tolist()

    def to_tuple(self) -> tuple[float, ...]:
        """Convert the vector to a tuple."""
        return tuple(self._data)

    def to_numpy(self) -> np.ndarray:
        """Convert the vector to a numpy array."""
        return self._data

    def is_zero(self) -> bool:
        """Check if this is a zero vector (all components are zero).

        Returns:
            True if all components are zero, False otherwise
        """
        return np.allclose(self._data, 0.0)

    def __bool__(self) -> bool:
        """Boolean conversion for Vector.

        A Vector is considered False if it's a zero vector (all components are zero),
        and True otherwise.

        Returns:
            False if vector is zero, True otherwise
        """
        return not self.is_zero()

    def __iter__(self):
        """Make Vector3 iterable so it can be converted to tuple/list."""
        return iter(self._data)


@dispatch
def to_numpy(value: "Vector3") -> np.ndarray:
    """Convert a Vector3 to a numpy array."""
    return value.data


@dispatch
def to_numpy(value: np.ndarray) -> np.ndarray:
    """Pass through numpy arrays."""
    return value


@dispatch
def to_numpy(value: Sequence[int | float]) -> np.ndarray:
    """Convert a sequence to a numpy array."""
    return np.array(value, dtype=float)


@dispatch
def to_vector(value: "Vector3") -> Vector3:
    """Pass through Vector3 objects."""
    return value


@dispatch
def to_vector(value: VectorLike) -> Vector3:
    """Convert a vector-compatible value to a Vector3 object."""
    return Vector3(value)


@dispatch
def to_tuple(value: Vector3) -> tuple[float, ...]:
    """Convert a Vector3 to a tuple."""
    return tuple(value.data)


@dispatch
def to_tuple(value: np.ndarray) -> tuple[float, ...]:
    """Convert a numpy array to a tuple."""
    return tuple(value.tolist())


@dispatch
def to_tuple(value: Sequence[int | float]) -> tuple[float, ...]:
    """Convert a sequence to a tuple."""
    if isinstance(value, tuple):
        return value
    else:
        return tuple(value)


@dispatch
def to_list(value: Vector3) -> list[float]:
    """Convert a Vector3 to a list."""
    return value.data.tolist()


@dispatch
def to_list(value: np.ndarray) -> list[float]:
    """Convert a numpy array to a list."""
    return value.tolist()


@dispatch
def to_list(value: Sequence[int | float]) -> list[float]:
    """Convert a sequence to a list."""
    if isinstance(value, list):
        return value
    else:
        return list(value)
