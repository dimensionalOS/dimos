# from dataclasses import dataclass
from dimos.types.vector import Vector


class Position:
    def __init__(self, pos: Vector, rot: Vector):
        self.pos = pos
        self.rot = rot

    @property
    def theta(self) -> float:
        return self.rot.z

    def __repr__(self) -> str:
        return f"pos({self.pos}), rot({self.rot})"

    def __str__(self) -> str:
        return self.__repr__()
