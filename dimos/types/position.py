from dataclasses import dataclass, field, InitVar
from typing import Any, Union, overload, TypeVar, Optional
from dimos.types.vector import Vector, VectorLike, to_vector

T = TypeVar('T', bound='Position')


class Position(Vector):
    def __init__(self, *args, **kwargs):
        """Initialize pos

        Examples:
            (Vector(1,2,3), Vector(4,5,6))
            ([1,2,3], [4,5,6])
            (np.array([1,2,3]), Vector(4,5,6))

            basically,

            (VectorLike, VectorLike)
            [VectorLike, VectorLike]

            can also omit the rotation,
            VectorLike
            [1,2,3]
            np.array(1,2,3)
            [VectorLike]
            [np.array(1,2,3)]
        """

    def __str__(self) -> str:
        return self.__repr__()

    def __bool__(self) -> bool:
        return not self.is_zero()
