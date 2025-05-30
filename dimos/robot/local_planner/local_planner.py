from abc import abstractmethod
from typing import Optional

from reactivex import Observable

from dimos.types.vector import Vector, VectorLike
from dimos.web.websocket_vis.helpers import Visualizable


class LocalPlanner(Visualizable):
    @abstractmethod
    def set_goal(self, goal: VectorLike) -> bool: ...

    @abstractmethod
    def get_move_stream(self, frequency: Optional[float]) -> Observable[Vector]: ...
