from dataclasses import dataclass, field, InitVar
from typing import Any, Union, overload, TypeVar, Optional
from dimos.types.vector import Vector, VectorLike, to_vector

T = TypeVar('T', bound='Position')


@dataclass
class Position:
    """A position in 3D space with both position and rotation vectors.
    
    Both position and rotation components can be initialized with any VectorLike object
    (Vector, numpy array, tuple, list) and will be converted to Vector objects internally.
    """
    # The fields are always Vector types internally
    pos: Vector = field(default_factory=Vector)
    rot: Vector = field(default_factory=Vector)
    
    def __post_init__(self):
        # Convert pos and rot to Vector if they are VectorLike
        if not isinstance(self.pos, Vector):
            self.pos = to_vector(self.pos)
        
        if not isinstance(self.rot, Vector):
            self.rot = to_vector(self.rot)

    def __repr__(self) -> str:
        return f"pos({self.pos}), rot({self.rot})"

    def __str__(self) -> str:
        return self.__repr__()

    def is_zero(self) -> bool:
        """Check if both position and rotation vectors are zero.

        Returns:
            True if both pos and rot are zero vectors, False otherwise
        """
        return self.pos.is_zero() and self.rot.is_zero()

    def __bool__(self) -> bool:
        """Boolean conversion for Position.

        A Position is considered False if it's a zero position (both vectors are zero),
        and True otherwise.

        Returns:
            False if position is zero, True otherwise
        """
        return not self.is_zero()


    @property
    def x(self):
        return self.pos.x

    @property
    def y(self):
        return self.pos.x

    @property
    def z(self):
        return self.pos.x
