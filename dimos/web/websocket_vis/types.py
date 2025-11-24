from typing import Union, Iterable, Tuple, TypedDict

from dimos.types.vector import Vector
from dimos.types.path import Path
from dimos.types.costmap import Costmap


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
