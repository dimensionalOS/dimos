import numpy as np
from typing import Union, Tuple, List, TypeVar, Generic, overload
from geometry_msgs.msg import Vector3

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
        if self.length() < 1e-10 or (
            isinstance(other, Vector) and other.length() < 1e-10
        ):
            return 0.0

        if isinstance(other, Vector):
            other_data = other._data
        else:
            other_data = np.array(other, dtype=float)

        cos_angle = np.clip(
            np.dot(self._data, other_data)
            / (np.linalg.norm(self._data) * np.linalg.norm(other_data)),
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
