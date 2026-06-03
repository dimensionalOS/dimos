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

"""Space: 2D spatial rendering canvas (world frame).

Space.add() is a smart dispatcher: it accepts element types directly
(explicit rendering mode), raw dimos msgs (auto-wrapped into default
element), or observations (smart dispatch based on data type).
"""

from __future__ import annotations

from typing import Any

from dimos.memory2.type.observation import EmbeddedObservation, Observation
from dimos.memory2.vis.color import ColorRange, resolve_deferred
from dimos.memory2.vis.space.elements import (
    Arrow,
    Box3D,
    Camera,
    Point,
    Polyline,
    Pose,
    SpaceElement,
    Text,
)
from dimos.msgs.geometry_msgs.Point import Point as GeoPoint
from dimos.msgs.geometry_msgs.Pose import Pose as GeoPose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.nav_msgs.Path import Path as NavPath
from dimos.msgs.protocol import DimosMsg
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.vision_msgs.Detection3D import Detection3D


def _autocolor_value(item: Any) -> float | None:
    """Extract the scalar to colormap for an item, or None if not auto-colorable."""
    if isinstance(item, EmbeddedObservation):
        return float(item.similarity or 0.0)
    if isinstance(item, Observation):
        if item.data_type in (float, int):
            return float(item.data)
        return float(item.ts)
    if isinstance(item, (int, float)):
        return float(item)
    return None


class Space:
    """2D spatial rendering canvas (world frame).

    Accumulates elements for spatial visualization. Elements can be added as:
    - Element types directly: ``s.add(Pose(posestamped, color="red"))``
    - Raw dimos msgs with style kwargs: ``s.add(posestamped, color="red")``
    - Observations (smart dispatch): ``s.add(observation)``
    - Lists of EmbeddedObservations: ``s.add(results)`` → similarity heatmap
    - Streams / iterables: ``s.add(stream)`` → materializes and adds each obs.data
    """

    def __init__(self) -> None:
        self._elements: list[SpaceElement] = []
        self._autocolor_ranges: list[ColorRange] = []

    def add(self, element: Any, **kwargs: Any) -> Space:
        """Add an element with smart dispatch.

        Element types (Pose, Arrow, Point, etc.) are stored as-is.
        Raw dimos msgs are auto-wrapped into their default element,
        with ``**kwargs`` forwarded as style (color, label, etc.).
        """

        if isinstance(element, (Pose, Arrow, Point, Box3D, Camera, Polyline, Text)):
            self._elements.append(element)
        elif isinstance(element, DimosMsg):
            self.add_dimos_msg(element, **kwargs)
        elif isinstance(element, EmbeddedObservation):
            self.add_embedded_observation(element, **kwargs)
        elif isinstance(element, Observation):
            self.add_observation(element, **kwargs)
        elif hasattr(element, "__iter__"):
            cmap = kwargs.pop("cmap", "turbo")
            color_range = ColorRange(cmap=cmap)
            self._autocolor_ranges.append(color_range)
            for item in element:
                v = _autocolor_value(item)
                if v is not None and isinstance(item, Observation):
                    self._elements.append(Arrow(msg=item.pose_stamped, color=color_range(v)))
                else:
                    self.add(item, **kwargs)
        else:
            raise TypeError(
                f"Space.add() does not know how to handle {type(element).__name__}. "
                f"Pass an element type (Pose, Arrow, Point, ...) or a dimos msg."
            )

        return self

    def add_dimos_msg(self, msg: DimosMsg, **kwargs: Any) -> None:
        """Dispatch a DimosMsg to its default element type."""
        if isinstance(msg, PoseStamped):
            self._elements.append(Pose(msg=msg, **kwargs))
        elif isinstance(msg, GeoPose):
            self._elements.append(Pose(msg=msg, **kwargs))
        elif isinstance(msg, GeoPoint):
            self._elements.append(Point(msg=msg, **kwargs))
        elif isinstance(msg, NavPath):
            self._elements.append(Polyline(msg=msg, **kwargs))
        elif isinstance(msg, OccupancyGrid):
            self._elements.append(msg)
        elif isinstance(msg, PointCloud2):
            self._elements.append(msg)
        elif isinstance(msg, Detection3D):
            self._elements.append(
                Box3D(
                    center=msg.bbox.center,
                    size=msg.bbox.size,
                    label=getattr(msg, "id", None),
                    **kwargs,
                )
            )
        else:
            raise TypeError(
                f"No default element for {type(msg).__name__}. "
                f"Wrap it explicitly (e.g. Pose(msg), Arrow(msg))."
            )

    def add_embedded_observation(self, obs: EmbeddedObservation[Any], **kwargs: Any) -> None:
        """Pass through to renderer like a regular Observation."""
        self._elements.append(obs)

    def add_observation(self, obs: Observation[Any], **kwargs: Any) -> None:
        """Store the observation directly; renderers decide how to display it."""
        self._elements.append(obs)

    def base_map(self, grid: OccupancyGrid) -> Space:
        """Add an OccupancyGrid as the background map."""
        return self.add(grid)

    def to_svg(self, path: str | None = None) -> str:
        """Render to SVG string. Optionally write to file."""
        from dimos.memory2.vis.space.svg import render

        resolve_deferred(self._elements)
        svg = render(self)
        if path is not None:
            with open(path, "w") as f:
                f.write(svg)
        return svg

    def to_rerun(self, app_id: str = "space", spawn: bool = True) -> None:
        """Render to Rerun viewer."""
        from dimos.memory2.vis.space.rerun import render

        resolve_deferred(self._elements)
        render(self, app_id=app_id, spawn=spawn)

    def _repr_svg_(self) -> str:
        """Jupyter inline display."""
        return self.to_svg()

    @property
    def elements(self) -> list[SpaceElement]:
        """Read-only access to accumulated elements."""
        return list(self._elements)

    def __len__(self) -> int:
        return len(self._elements)

    def __repr__(self) -> str:
        counts: dict[str, int] = {}
        for el in self._elements:
            name = type(el).__name__
            counts[name] = counts.get(name, 0) + 1
        parts = [f"{n}={c}" for n, c in sorted(counts.items())]
        return f"Space({', '.join(parts)})"
