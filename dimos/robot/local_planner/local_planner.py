from abc import abstractmethod
from typing import Optional

from reactivex import Observable

from dimos.types.vector import Vector, VectorLike
from dimos.types.path import Path
from dimos.web.websocket_vis.helpers import Visualizable


class LocalPlanner(Visualizable):
    @abstractmethod
    def set_path(self, path: Path) -> bool: ...

    @abstractmethod
    def get_move_stream(self, frequency: Optional[float]) -> Observable[Vector]: ...
