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

from typing import TypeVar, Optional, Union
import numpy as np
from dimos.types.vector import Vector, VectorLike, to_vector

T = TypeVar('T', bound='Position')


class Position(Vector):
    """A position in 3D space, consisting of a position vector and a rotation vector.
    
    Position inherits from Vector and behaves like a vector for the position component.
    The rotation vector is stored separately and can be accessed via the rot property.
    """

    def __init__(self, *args, **kwargs):
        """Initialize a position with optional position and rotation vectors.
        
        Examples:
            Position()                         # Zero position and rotation
            Position(Vector(1,2,3))            # Custom position, zero rotation
            Position([1,2,3])                  # From list, zero rotation
            Position(rot=Vector(4,5,6))        # Zero position, custom rotation
            Position(Vector(1,2,3), Vector(4,5,6))  # Custom position and rotation
            Position([1,2,3], [4,5,6])         # From lists
            Position(np.array([1,2,3]), np.array([4,5,6]))  # From numpy arrays
        """
        pos = None
        rot = kwargs.get('rot', None)
        
        # Handle different initialization patterns
        if 'pos' in kwargs:
            pos = kwargs['pos']
        elif len(args) == 1:
            pos = args[0]
        elif len(args) >= 2:
            pos, rot = args[0], args[1]
        
        # Initialize the position component using Vector's initialization
        if pos is None:
            super().__init__(Vector.zeros(3))
        else:
            super().__init__(pos)
        
        # Initialize rotation vector
        self.rot = to_vector(rot) if rot is not None else Vector.zeros(3)

    def __repr__(self) -> str:
        """String representation of the position."""
        return f"Position(pos={super().__repr__()}, rot={self.rot})"

    def __str__(self) -> str:
        """String representation of the position."""
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
        return {
            "type": "position",
            "pos": self.to_list(),
            "rot": self.rot.to_list()
        }
    
    def __eq__(self, other) -> bool:
        """Check if two positions are equal."""
        if not isinstance(other, Position):
            return False
        return super().__eq__(other) and self.rot == other.rot
        
    @property
    def pos(self) -> Vector:
        """Get the position vector (self)."""
        return to_vector(self.data)
        
    def __add__(self: T, other) -> T:
        """Override Vector's __add__ to handle Position objects specially.
        
        When adding two Position objects, both position and rotation components are added.
        """
        if isinstance(other, Position):
            # Add both position and rotation components
            result = super().__add__(other)
            result.rot = self.rot + other.rot
            return result
        else:
            # For other types, just use Vector's addition
            return Position(pos=super().__add__(other), rot=self.rot)
            
    def __sub__(self: T, other) -> T:
        """Override Vector's __sub__ to handle Position objects specially.
        
        When subtracting two Position objects, both position and rotation components are subtracted.
        """
        if isinstance(other, Position):
            # Subtract both position and rotation components
            result = super().__sub__(other)
            result.rot = self.rot - other.rot
            return result
        else:
            # For other types, just use Vector's subtraction
            return super().__sub__(other)
            
    def __mul__(self: T, scalar: float) -> T:
        """Override Vector's __mul__ to handle scalar multiplication.
        
        When multiplying a Position by a scalar, only the position component is multiplied.
        The rotation component remains unchanged.
        """
        print("RET", "ROT IS", self.rot, "POS IS", self.pos, "RES IS",self.pos * scalar)
        # Use Vector's multiplication for the position component
        return Position(pos=self.pos * scalar, rot=self.rot)
