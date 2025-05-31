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
