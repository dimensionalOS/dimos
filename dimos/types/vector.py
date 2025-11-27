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

import builtins
from collections.abc import Sequence
from typing import TypeVar, Union

import numpy as np

from dimos.types.ros_polyfill import Vector3

T = TypeVar("T", bound="Vector")

# Vector-like types that can be converted to/from Vector
VectorLike = Union[Sequence[int | float], Vector3, "Vector", np.ndarray]  # type: ignore[type-arg]


class Vector:
    """A wrapper around numpy arrays for vector operations with intuitive syntax."""

    def __init__(self, *args: VectorLike) -> None:
        """Initialize a vector from components or another iterable.

        Examples:
            Vector(1, 2)       # 2D vector
            Vector(1, 2, 3)    # 3D vector
            Vector([1, 2, 3])  # From list
            Vector(np.array([1, 2, 3])) # From numpy array
        """
        if len(args) == 1 and hasattr(args[0], "__iter__"):
            self._data = np.array(args[0], dtype=float)

        elif len(args) == 1:
            self._data = np.array([args[0].x, args[0].y, args[0].z], dtype=float)  # type: ignore[union-attr]

        else:
            self._data = np.array(args, dtype=float)

    @property
    def yaw(self) -> float:
        return self.x

    @property
    def tuple(self) -> tuple[float, ...]:
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
    def data(self) -> np.ndarray:  # type: ignore[type-arg]
        """Get the underlying numpy array."""
        return self._data

    def __getitem__(self, idx: int):  # type: ignore[no-untyped-def]
        return self._data[idx]

    def __repr__(self) -> str:
        return f"Vector({self.data})"

    def __str__(self) -> str:
        if self.dim < 2:
            return self.__repr__()

        def getArrow():  # type: ignore[no-untyped-def]
            repr = ["←", "↖", "↑", "↗", "→", "↘", "↓", "↙"]

            if self.x == 0 and self.y == 0:
                return "·"

            # Calculate angle in radians and convert to directional index
            angle = np.arctan2(self.y, self.x)
            # Map angle to 0-7 index (8 directions) with proper orientation
            dir_index = int(((angle + np.pi) * 4 / np.pi) % 8)
            # Get directional arrow symbol
            return repr[dir_index]

        return f"{getArrow()} Vector {self.__repr__()}"  # type: ignore[no-untyped-call]

    def serialize(self) -> builtins.tuple:  # type: ignore[type-arg]
        """Serialize the vector to a tuple."""
        return {"type": "vector", "c": self._data.tolist()}  # type: ignore[return-value]

    def __eq__(self, other) -> bool:  # type: ignore[no-untyped-def]
        """Check if two vectors are equal using numpy's allclose for floating point comparison."""
        if not isinstance(other, Vector):
            return False
        if len(self._data) != len(other._data):
            return False
        return np.allclose(self._data, other._data)

    def __add__(self: T, other: VectorLike) -> T:
        other = to_vector(other)
        if self.dim != other.dim:
            max_dim = max(self.dim, other.dim)
            return self.pad(max_dim) + other.pad(max_dim)
        return self.__class__(self._data + other._data)

    def __sub__(self: T, other: VectorLike) -> T:
        other = to_vector(other)
        if self.dim != other.dim:
            max_dim = max(self.dim, other.dim)
            return self.pad(max_dim) - other.pad(max_dim)
        return self.__class__(self._data - other._data)

    def __mul__(self: T, scalar: float) -> T:
        return self.__class__(self._data * scalar)

    def __rmul__(self: T, scalar: float) -> T:
        return self.__mul__(scalar)

    def __truediv__(self: T, scalar: float) -> T:
        return self.__class__(self._data / scalar)

    def __neg__(self: T) -> T:
        return self.__class__(-self._data)

    def dot(self, other: VectorLike) -> float:
        """Compute dot product."""
        other = to_vector(other)
        return float(np.dot(self._data, other._data))

    def cross(self: T, other: VectorLike) -> T:
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

    def normalize(self: T) -> T:
        """Return a normalized unit vector in the same direction."""
        length = self.length()
        if length < 1e-10:  # Avoid division by near-zero
            return self.__class__(np.zeros_like(self._data))
        return self.__class__(self._data / length)

    def to_2d(self: T) -> T:
        """Convert a vector to a 2D vector by taking only the x and y components."""
        return self.__class__(self._data[:2])

    def pad(self: T, dim: int) -> T:
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

    def project(self: T, onto: VectorLike) -> T:
        """Project this vector onto another vector."""
        onto = to_vector(onto)
        onto_length_sq = np.sum(onto._data * onto._data)
        if onto_length_sq < 1e-10:
            return self.__class__(np.zeros_like(self._data))

        scalar_projection = np.dot(self._data, onto._data) / onto_length_sq
        return self.__class__(scalar_projection * onto._data)

    @classmethod
    def zeros(cls: type[T], dim: int) -> T:
        """Create a zero vector of given dimension."""
        return cls(np.zeros(dim))

    @classmethod
    def ones(cls: type[T], dim: int) -> T:
        """Create a vector of ones with given dimension."""
        return cls(np.ones(dim))

    @classmethod
    def unit_x(cls: type[T], dim: int = 3) -> T:
        """Create a unit vector in the x direction."""
        v = np.zeros(dim)
        v[0] = 1.0
        return cls(v)

    @classmethod
    def unit_y(cls: type[T], dim: int = 3) -> T:
        """Create a unit vector in the y direction."""
        v = np.zeros(dim)
        v[1] = 1.0
        return cls(v)

    @classmethod
    def unit_z(cls: type[T], dim: int = 3) -> T:
        """Create a unit vector in the z direction."""
        v = np.zeros(dim)
        if dim > 2:
            v[2] = 1.0
        return cls(v)

    def to_list(self) -> list[float]:
        """Convert the vector to a list."""
        return self._data.tolist()  # type: ignore[no-any-return]

    def to_tuple(self) -> builtins.tuple[float, ...]:
        """Convert the vector to a tuple."""
        return tuple(self._data)

    def to_numpy(self) -> np.ndarray:  # type: ignore[type-arg]
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


def to_numpy(value: VectorLike) -> np.ndarray:  # type: ignore[type-arg]
    """Convert a vector-compatible value to a numpy array.

    Args:
        value: Any vector-like object (Vector, numpy array, tuple, list)

    Returns:
        Numpy array representation
    """
    if isinstance(value, Vector3):
        return np.array([value.x, value.y, value.z], dtype=float)
    if isinstance(value, Vector):
        return value.data
    elif isinstance(value, np.ndarray):
        return value
    else:
        return np.array(value, dtype=float)


def to_vector(value: VectorLike) -> Vector:
    """Convert a vector-compatible value to a Vector object.

    Args:
        value: Any vector-like object (Vector, numpy array, tuple, list)

    Returns:
        Vector object
    """
    if isinstance(value, Vector):
        return value
    else:
        return Vector(value)


def to_tuple(value: VectorLike) -> tuple[float, ...]:
    """Convert a vector-compatible value to a tuple.

    Args:
        value: Any vector-like object (Vector, numpy array, tuple, list)

    Returns:
        Tuple of floats
    """
    if isinstance(value, Vector3):
        return tuple([value.x, value.y, value.z])
    if isinstance(value, Vector):
        return tuple(value.data)
    elif isinstance(value, np.ndarray):
        return tuple(value.tolist())
    elif isinstance(value, tuple):
        return value
    else:
        return tuple(value)


def to_list(value: VectorLike) -> list[float]:
    """Convert a vector-compatible value to a list.

    Args:
        value: Any vector-like object (Vector, numpy array, tuple, list)

    Returns:
        List of floats
    """
    if isinstance(value, Vector):
        return value.data.tolist()  # type: ignore[no-any-return]
    elif isinstance(value, np.ndarray):
        return value.tolist()  # type: ignore[no-any-return]
    elif isinstance(value, list):
        return value
    else:
        return list(value)  # type: ignore[arg-type]


# Helper functions to check dimensionality
def is_2d(value: VectorLike) -> bool:
    """Check if a vector-compatible value is 2D.

    Args:
        value: Any vector-like object (Vector, numpy array, tuple, list)

    Returns:
        True if the value is 2D
    """
    if isinstance(value, Vector3):
        return False
    elif isinstance(value, Vector):
        return len(value) == 2  # type: ignore[arg-type]
    elif isinstance(value, np.ndarray):
        return value.shape[-1] == 2 or value.size == 2
    else:
        return len(value) == 2


def is_3d(value: VectorLike) -> bool:
    """Check if a vector-compatible value is 3D.

    Args:
        value: Any vector-like object (Vector, numpy array, tuple, list)

    Returns:
        True if the value is 3D
    """
    if isinstance(value, Vector):
        return len(value) == 3  # type: ignore[arg-type]
    elif isinstance(value, Vector3):
        return True
    elif isinstance(value, np.ndarray):
        return value.shape[-1] == 3 or value.size == 3
    else:
        return len(value) == 3


# Extraction functions for XYZ components
def x(value: VectorLike) -> float:
    """Get the X component of a vector-compatible value.

    Args:
        value: Any vector-like object (Vector, numpy array, tuple, list)

    Returns:
        X component as a float
    """
    if isinstance(value, Vector):
        return value.x
    elif isinstance(value, Vector3):
        return value.x  # type: ignore[no-any-return]
    else:
        return float(to_numpy(value)[0])


def y(value: VectorLike) -> float:
    """Get the Y component of a vector-compatible value.

    Args:
        value: Any vector-like object (Vector, numpy array, tuple, list)

    Returns:
        Y component as a float
    """
    if isinstance(value, Vector):
        return value.y
    elif isinstance(value, Vector3):
        return value.y  # type: ignore[no-any-return]
    else:
        arr = to_numpy(value)
        return float(arr[1]) if len(arr) > 1 else 0.0


def z(value: VectorLike) -> float:
    """Get the Z component of a vector-compatible value.

    Args:
        value: Any vector-like object (Vector, numpy array, tuple, list)

    Returns:
        Z component as a float
    """
    if isinstance(value, Vector):
        return value.z
    elif isinstance(value, Vector3):
        return value.z  # type: ignore[no-any-return]
    else:
        arr = to_numpy(value)
        return float(arr[2]) if len(arr) > 2 else 0.0
