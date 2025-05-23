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

from typing import List, Tuple, TypeVar, Union, Sequence
import numpy as np
from plum import dispatch

from dimos.types.vector import Vector, to_vector, to_numpy, VectorLike


T = TypeVar("T", bound="Position")


class Position(Vector):
    """A position in 3D space, consisting of a position vector and a rotation vector.

    Position inherits from Vector and behaves like a vector for the position component.
    The rotation vector is stored separately and can be accessed via the rot property.
    """

    _rot: Vector = None

    @dispatch
    def __init__(self, pos: VectorLike):
        """Initialize Position with a Vector or a tuple of vectors.

        Args:
            pos: A Vector or a tuple of (pos, rot) vectors.
        """
        self._data = to_numpy(pos)
        self._rot = None

    @dispatch
    def __init__(self, pos: VectorLike, rot: VectorLike):
        """Initialize Position with position and rotation vectors.

        Args:
            pos: The position vector.
            rot: The rotation vector.
        """
        self._data = to_numpy(pos)
        self._rot = to_vector(rot)

    def __repr__(self) -> str:
        return f"Position({self.pos.__repr__()}, {self.rot.__repr__()})"

    def __str__(self) -> str:
        return self.__repr__()

    def is_zero(self) -> bool:
        """Check if both position and rotation vectors are zero."""
        return super().is_zero() and self.rot.is_zero()

    def __bool__(self) -> bool:
        """Boolean conversion for Position.

        A Position is considered False if both position and rotation are zero vectors,
        and True otherwise.

        Returns:
            False if position is zero, True otherwise
        """
        return not self.is_zero()

    def serialize(self):
        """Serialize the position to a dictionary."""
        return {"type": "position", "pos": self.to_list(), "rot": self.rot.to_list()}

    def __eq__(self, other) -> bool:
        """Check if two positions are equal using numpy's allclose for floating point comparison."""
        if not isinstance(other, Position):
            return False
        return np.allclose(self.pos._data, other.pos._data) and np.allclose(self.rot._data, other.rot._data)

    @property
    def rot(self) -> Vector:
        if self._rot:
            return self._rot
        else:
            return Vector(0, 0, 0)

    @property
    def pos(self) -> Vector:
        """Get the position vector (self)."""
        return to_vector(self._data)

    def __add__(self: T, other) -> T:
        """Override Vector's __add__ to handle Position objects specially.

        When adding two Position objects, both position and rotation components are added.
        """
        if isinstance(other, Position):
            # Add both position and rotation components
            result = super().__add__(other)
            result._rot = self.rot + other.rot
            return result
        else:
            # For other types, just use Vector's addition
            return Position(super().__add__(other), self.rot)

    def __sub__(self: T, other) -> T:
        """Override Vector's __sub__ to handle Position objects specially.

        When subtracting two Position objects, both position and rotation components are subtracted.
        """
        if isinstance(other, Position):
            # Subtract both position and rotation components
            result = super().__sub__(other)
            result._rot = self.rot - other.rot
            return result
        else:
            # For other types, just use Vector's subtraction
            return super().__sub__(other)

    def __mul__(self: T, scalar: float) -> T:
        """Override Vector's __mul__ to handle scalar multiplication.

        When multiplying a Position by a scalar, only the position component is multiplied.
        The rotation component remains unchanged.
        """
        # Use Vector's multiplication for the position component
        return Position(self.pos * scalar, self.rot)
