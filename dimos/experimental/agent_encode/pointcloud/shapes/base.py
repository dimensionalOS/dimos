# Copyright 2026 Dimensional Inc.
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

from __future__ import annotations

from abc import ABC, abstractmethod

import numpy as np
from numpy.typing import NDArray

from dimos.experimental.agent_encode.pointcloud.image.lib.canvas import Canvas, Drawable


class Shape(Drawable, ABC):
    """A solid that returns are tested against; drawn as its wireframe."""

    @abstractmethod
    def contains(self, points: NDArray[np.float32] | NDArray[np.float64]) -> NDArray[np.bool_]:
        """Whether each (N, 3) point lies inside."""

    @abstractmethod
    def distance(self, points: NDArray[np.float32] | NDArray[np.float64]) -> NDArray[np.float64]:
        """Distance from the surface to each (N, 3) point; 0 inside."""

    @abstractmethod
    def chord(self, direction: NDArray[np.float64]) -> float:
        """Length of the longest segment along the unit ``direction`` inside the shape."""

    @abstractmethod
    def anchor(self, z_extent: tuple[float, float]) -> NDArray[np.float64]:
        """The shape's centre; ``z_extent`` closes unbounded ends."""

    @abstractmethod
    def wireframe(self, z_extent: tuple[float, float]) -> list[NDArray[np.float64]]:
        """World-coordinate outline paths; ``z_extent`` closes unbounded ends."""

    def draw(self, canvas: Canvas) -> None:
        for path in self.wireframe(canvas.z_extent):
            canvas.path(path)
