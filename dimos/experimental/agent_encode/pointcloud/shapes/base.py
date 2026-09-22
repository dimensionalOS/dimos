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

from abc import abstractmethod

import numpy as np

from dimos.experimental.agent_encode.pointcloud.render.overlays import Canvas, Overlay


class Shape(Overlay):
    """A solid that queries test returns against."""

    @abstractmethod
    def contains(self, points: np.ndarray) -> np.ndarray:
        """Whether each point lies inside."""

    @abstractmethod
    def distance(self, points: np.ndarray) -> np.ndarray:
        """Distance from the surface to each point; 0 inside."""

    @abstractmethod
    def chord(self, direction: np.ndarray) -> float:
        """Length of the longest segment along the unit ``direction`` inside the shape."""

    @abstractmethod
    def anchor(self, z_extent: tuple[float, float]) -> np.ndarray:
        """The shape's centre; ``z_extent`` closes unbounded ends."""

    @abstractmethod
    def wireframe(self, z_extent: tuple[float, float]) -> list[np.ndarray]:
        """World-coordinate outline paths; ``z_extent`` closes unbounded ends."""

    def draw(self, canvas: Canvas) -> list[str]:
        for path in self.wireframe(canvas.z_extent):
            canvas.path(path)
        return ["shape"]
