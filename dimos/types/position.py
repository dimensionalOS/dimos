from dimos.types.vector import Vector


class Position:
    def __init__(self, coords: Vector, rot: Vector):
        self.coords = coords
        self.rot = rot

    @property
    def theta(self) -> float:
        return self.rot.z

    def __repr__(self) -> str:
        return f"coords({self.coords}), rot({self.rot})"

    def __str__(self) -> str:
        return self.__repr__()
