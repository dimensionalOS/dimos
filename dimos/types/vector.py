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

import numpy as np
from geometry_msgs.msg import Vector3
from typing import (
    Tuple,
    List,
    TypeVar,
    Protocol,
    runtime_checkable,
)

T = TypeVar("T", bound="Vector")


class Vector:
    """A wrapper around numpy arrays for vector operations with intuitive syntax."""

    def __init__(self, *args):
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
            self._data = np.array([args[0].x, args[0].y, args[0].z], dtype=float)

        else:
            self._data = np.array(args, dtype=float)

    @property
    def yaw(self) -> float:
        return self.x

    @property
    def tuple(self) -> Tuple[float, ...]:
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

    def __len__(self) -> int:
        return len(self._data)

    def __getitem__(self, idx):
        return self._data[idx]

    def __iter__(self):
        return iter(self._data)

    def __repr__(self) -> str:
        components = ",".join(f"{x:.6g}" for x in self._data)
        return f"({components})"

    def __str__(self) -> str:
        if self.dim < 2:
            return self.__repr__()

        def getArrow():
            repr = ["←", "↖", "↑", "↗", "→", "↘", "↓", "↙"]

            if self.y == 0 and self.x == 0:
                return "·"

            # Calculate angle in radians and convert to directional index
            angle = np.arctan2(self.y, self.x)
            # Map angle to 0-7 index (8 directions) with proper orientation
            dir_index = int(((angle + np.pi) * 4 / np.pi) % 8)
            # Get directional arrow symbol
            return repr[dir_index]

        return f"{getArrow()} Vector {self.__repr__()}"

    def serialize(self) -> Tuple:
        """Serialize the vector to a tuple."""
        return {"type": "vector", "c": self._data.tolist()}

    def __eq__(self, other) -> bool:
        if isinstance(other, Vector):
            return np.array_equal(self._data, other._data)
        return np.array_equal(self._data, np.array(other, dtype=float))

    def __add__(self: T, other) -> T:
        if isinstance(other, Vector):
            return self.__class__(self._data + other._data)
        return self.__class__(self._data + np.array(other, dtype=float))

    def __sub__(self: T, other) -> T:
        if isinstance(other, Vector):
            return self.__class__(self._data - other._data)
        return self.__class__(self._data - np.array(other, dtype=float))

    def __mul__(self: T, scalar: float) -> T:
        return self.__class__(self._data * scalar)

    def __rmul__(self: T, scalar: float) -> T:
        return self.__mul__(scalar)

    def __truediv__(self: T, scalar: float) -> T:
        return self.__class__(self._data / scalar)

    def __neg__(self: T) -> T:
        return self.__class__(-self._data)

    def dot(self, other) -> float:
        """Compute dot product."""
        if isinstance(other, Vector):
            return float(np.dot(self._data, other._data))
        return float(np.dot(self._data, np.array(other, dtype=float)))

    def cross(self: T, other) -> T:
        """Compute cross product (3D vectors only)."""
        if self.dim != 3:
            raise ValueError("Cross product is only defined for 3D vectors")

        if isinstance(other, Vector):
            other_data = other._data
        else:
            other_data = np.array(other, dtype=float)

        if len(other_data) != 3:
            raise ValueError("Cross product requires two 3D vectors")

        return self.__class__(np.cross(self._data, other_data))

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

    def distance(self, other) -> float:
        """Compute Euclidean distance to another vector."""
        if isinstance(other, Vector):
            return float(np.linalg.norm(self._data - other._data))
        return float(np.linalg.norm(self._data - np.array(other, dtype=float)))

    def distance_squared(self, other) -> float:
        """Compute squared Euclidean distance to another vector (faster than distance())."""
        if isinstance(other, Vector):
            diff = self._data - other._data
        else:
            diff = self._data - np.array(other, dtype=float)
        return float(np.sum(diff * diff))

    def angle(self, other) -> float:
        """Compute the angle (in radians) between this vector and another."""
        if self.length() < 1e-10 or (isinstance(other, Vector) and other.length() < 1e-10):
            return 0.0

        if isinstance(other, Vector):
            other_data = other._data
        else:
            other_data = np.array(other, dtype=float)

        cos_angle = np.clip(
            np.dot(self._data, other_data) / (np.linalg.norm(self._data) * np.linalg.norm(other_data)),
            -1.0,
            1.0,
        )
        return float(np.arccos(cos_angle))

    def project(self: T, onto) -> T:
        """Project this vector onto another vector."""
        if isinstance(onto, Vector):
            onto_data = onto._data
        else:
            onto_data = np.array(onto, dtype=float)

        onto_length_sq = np.sum(onto_data * onto_data)
        if onto_length_sq < 1e-10:
            return self.__class__(np.zeros_like(self._data))

        scalar_projection = np.dot(self._data, onto_data) / onto_length_sq
        return self.__class__(scalar_projection * onto_data)

    # this is here to test ros_observable_topic
    # doesn't happen irl afaik that we want a vector from ros message
    @classmethod
    def from_msg(cls: type[T], msg) -> T:
        return cls(*msg)

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

    def to_list(self) -> List[float]:
        """Convert the vector to a list."""
        return self._data.tolist()

    def to_tuple(self) -> Tuple[float, ...]:
        """Convert the vector to a tuple."""
        return tuple(self._data)

    def to_numpy(self) -> np.ndarray:
        """Convert the vector to a numpy array."""
        return self._data


# Protocol approach for static type checking
@runtime_checkable
class VectorLike(Protocol):
    """Protocol for types that can be treated as vectors."""

    def __getitem__(self, key: int) -> float: ...
    def __len__(self) -> int: ...


def to_numpy(value: VectorLike) -> np.ndarray:
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


def to_tuple(value: VectorLike) -> Tuple[float, ...]:
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


def to_list(value: VectorLike) -> List[float]:
    """Convert a vector-compatible value to a list.

    Args:
        value: Any vector-like object (Vector, numpy array, tuple, list)

    Returns:
        List of floats
    """
    if isinstance(value, Vector):
        return value.data.tolist()
    elif isinstance(value, np.ndarray):
        return value.tolist()
    elif isinstance(value, list):
        return value
    else:
        return list(value)


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
        return len(value) == 2
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
        return len(value) == 3
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
        return value.x
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
        return value.y
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
        return value.z
    else:
        arr = to_numpy(value)
        return float(arr[2]) if len(arr) > 2 else 0.0


if __name__ == "__main__":
    # Test vectors in various directions
    test_vectors = [
        Vector(1, 0),  # Right
        Vector(1, 1),  # Up-Right
        Vector(0, 1),  # Up
        Vector(-1, 1),  # Up-Left
        Vector(-1, 0),  # Left
        Vector(-1, -1),  # Down-Left
        Vector(0, -1),  # Down
        Vector(1, -1),  # Down-Right
        Vector(0.5, 0.5),  # Up-Right (shorter)
        Vector(-3, 4),  # Up-Left (longer)
        Vector(Vector3(x=2.0, y=3.0, z=4.0)),
    ]

    for v in test_vectors:
        print(str(v))

    # Test the vector compatibility functions
    print("Testing vectortypes.py conversion functions\n")

    # Create test vectors in different formats
    vector_obj = Vector(1.0, 2.0, 3.0)
    numpy_arr = np.array([4.0, 5.0, 6.0])
    tuple_vec = (7.0, 8.0, 9.0)
    list_vec = [10.0, 11.0, 12.0]

    print("Original values:")
    print(f"Vector:     {vector_obj}")
    print(f"NumPy:      {numpy_arr}")
    print(f"Tuple:      {tuple_vec}")
    print(f"List:       {list_vec}")
    print()

    # Test to_numpy
    print("to_numpy() conversions:")
    print(f"Vector → NumPy:  {to_numpy(vector_obj)}")
    print(f"NumPy → NumPy:   {to_numpy(numpy_arr)}")
    print(f"Tuple → NumPy:   {to_numpy(tuple_vec)}")
    print(f"List → NumPy:    {to_numpy(list_vec)}")
    print()

    # Test to_vector
    print("to_vector() conversions:")
    print(f"Vector → Vector:  {to_vector(vector_obj)}")
    print(f"NumPy → Vector:   {to_vector(numpy_arr)}")
    print(f"Tuple → Vector:   {to_vector(tuple_vec)}")
    print(f"List → Vector:    {to_vector(list_vec)}")
    print()

    # Test to_tuple
    print("to_tuple() conversions:")
    print(f"Vector → Tuple:  {to_tuple(vector_obj)}")
    print(f"NumPy → Tuple:   {to_tuple(numpy_arr)}")
    print(f"Tuple → Tuple:   {to_tuple(tuple_vec)}")
    print(f"List → Tuple:    {to_tuple(list_vec)}")
    print()

    # Test to_list
    print("to_list() conversions:")
    print(f"Vector → List:  {to_list(vector_obj)}")
    print(f"NumPy → List:   {to_list(numpy_arr)}")
    print(f"Tuple → List:   {to_list(tuple_vec)}")
    print(f"List → List:    {to_list(list_vec)}")
    print()

    # Test component extraction
    print("Component extraction:")
    print("x() function:")
    print(f"x(Vector):  {x(vector_obj)}")
    print(f"x(NumPy):   {x(numpy_arr)}")
    print(f"x(Tuple):   {x(tuple_vec)}")
    print(f"x(List):    {x(list_vec)}")
    print()

    print("y() function:")
    print(f"y(Vector):  {y(vector_obj)}")
    print(f"y(NumPy):   {y(numpy_arr)}")
    print(f"y(Tuple):   {y(tuple_vec)}")
    print(f"y(List):    {y(list_vec)}")
    print()

    print("z() function:")
    print(f"z(Vector):  {z(vector_obj)}")
    print(f"z(NumPy):   {z(numpy_arr)}")
    print(f"z(Tuple):   {z(tuple_vec)}")
    print(f"z(List):    {z(list_vec)}")
    print()

    # Test dimension checking
    print("Dimension checking:")
    vec2d = Vector(1.0, 2.0)
    vec3d = Vector(1.0, 2.0, 3.0)
    arr2d = np.array([1.0, 2.0])
    arr3d = np.array([1.0, 2.0, 3.0])

    print(f"is_2d(Vector(1,2)):       {is_2d(vec2d)}")
    print(f"is_2d(Vector(1,2,3)):     {is_2d(vec3d)}")
    print(f"is_2d(np.array([1,2])):   {is_2d(arr2d)}")
    print(f"is_2d(np.array([1,2,3])): {is_2d(arr3d)}")
    print(f"is_2d((1,2)):             {is_2d((1.0, 2.0))}")
    print(f"is_2d((1,2,3)):           {is_2d((1.0, 2.0, 3.0))}")
    print()

    print(f"is_3d(Vector(1,2)):       {is_3d(vec2d)}")
    print(f"is_3d(Vector(1,2,3)):     {is_3d(vec3d)}")
    print(f"is_3d(np.array([1,2])):   {is_3d(arr2d)}")
    print(f"is_3d(np.array([1,2,3])): {is_3d(arr3d)}")
    print(f"is_3d((1,2)):             {is_3d((1.0, 2.0))}")
    print(f"is_3d((1,2,3)):           {is_3d((1.0, 2.0, 3.0))}")
    print()

    # Test the Protocol interface
    print("Testing VectorLike Protocol:")
    print(f"isinstance(Vector(1,2), VectorLike):      {isinstance(vec2d, VectorLike)}")
    print(f"isinstance(np.array([1,2]), VectorLike):  {isinstance(arr2d, VectorLike)}")
    print(f"isinstance((1,2), VectorLike):            {isinstance((1.0, 2.0), VectorLike)}")
    print(f"isinstance([1,2], VectorLike):            {isinstance([1.0, 2.0], VectorLike)}")
    print()

    # Test mixed operations using different vector types
    # These functions aren't defined in vectortypes, but demonstrate the concept
    def distance(a, b):
        a_np = to_numpy(a)
        b_np = to_numpy(b)
        diff = a_np - b_np
        return np.sqrt(np.sum(diff * diff))

    def midpoint(a, b):
        a_np = to_numpy(a)
        b_np = to_numpy(b)
        return (a_np + b_np) / 2

    print("Mixed operations between different vector types:")
    print(f"distance(Vector(1,2,3), [4,5,6]):           {distance(vec3d, [4.0, 5.0, 6.0])}")
    print(f"distance(np.array([1,2,3]), (4,5,6)):       {distance(arr3d, (4.0, 5.0, 6.0))}")
    print(f"midpoint(Vector(1,2,3), np.array([4,5,6])): {midpoint(vec3d, numpy_arr)}")
