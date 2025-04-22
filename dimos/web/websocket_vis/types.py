from typing import Union, Iterable, Tuple, TypedDict, List
from abc import ABC, abstractmethod

from dimos.types.vector import Vector
from dimos.types.path import Path
from dimos.types.costmap import Costmap
from reactivex.observable import Observable


class VectorDrawConfig(TypedDict, total=False):
    color: str
    width: float
    style: str  # "solid", "dashed", etc.


class PathDrawConfig(TypedDict, total=False):
    color: str
    width: float
    style: str
    fill: bool


class CostmapDrawConfig(TypedDict, total=False):
    colormap: str
    opacity: float
    scale: float


Drawable = Union[
    Vector,
    Path,
    Costmap,
    Tuple[Vector, VectorDrawConfig],
    Tuple[Path, PathDrawConfig],
    Tuple[Costmap, CostmapDrawConfig],
]
Drawables = Iterable[Drawable]


class Visualizable(ABC):
    """
    Abstract base class for objects that can provide visualization data.
    """

    @abstractmethod
    def vis_stream(self) -> Observable[List[Drawable]]:
        """
        Returns an Observable stream of Drawable objects that can be
        visualized in the websocket visualization system.

        Returns:
            Observable[List[Drawable]]: An observable stream of lists of drawable objects
        """
        pass

    @abstractmethod
    def vis(self, name: str, drawable: Drawable) -> None:
        """
        Visualizes the provided drawables.

        Args:
            drawables (List[Drawable]): A list of drawable objects to visualize.
        """
        pass
