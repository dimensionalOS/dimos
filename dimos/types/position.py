from dataclasses import dataclass
from dimos.types.vector import Vector


@dataclass
class Position:
    pos: Vector
    rot: Vector

    def __repr__(self) -> str:
        return f"pos({self.pos}), rot({self.rot})"

    def __str__(self) -> str:
        return self.__repr__()
