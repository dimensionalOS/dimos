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

"""Drawing2D builder for memory2 visualization.

Drawing2D.add() is a smart dispatcher: it accepts vis types directly (explicit
rendering mode), raw dimos msgs (auto-wrapped into default vis type), or
observations (smart dispatch based on data type).
"""

from __future__ import annotations

from typing import Any

from dimos.memory2.type.observation import EmbeddedObservation, Observation
from dimos.memory2.vis.type import (
    Arrow,
    Box3D,
    Camera,
    Point,
    Polyline,
    Pose,
    SceneElement,
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


class Drawing2D:
    """Accumulates scene elements for spatial 2D visualization.

    Elements can be added as:
    - Vis types directly: ``d.add(Pose(posestamped, color="red"))``
    - Raw dimos msgs with style kwargs: ``d.add(posestamped, color="red")``
    - Observations (smart dispatch): ``d.add(observation)``
    - Lists of EmbeddedObservations: ``d.add(results)`` → similarity heatmap
    - Streams / iterables: ``d.add(stream)`` → materializes and adds each obs.data
    """

    def __init__(self) -> None:
        self._elements: list[SceneElement] = []

    def add(self, element: Any, **kwargs: Any) -> Drawing2D:
        """Add a scene element with smart dispatch.

        Vis types (Pose, Arrow, Point, etc.) are stored as-is.
        Raw dimos msgs are auto-wrapped into their default vis type,
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
            for item in element:
                self.add(item, **kwargs)
        else:
            raise TypeError(
                f"Drawing2D.add() does not know how to handle {type(element).__name__}. "
                f"Pass a vis type (Pose, Arrow, Point, ...) or a dimos msg."
            )

        return self

    def add_dimos_msg(self, msg: DimosMsg, **kwargs: Any) -> None:
        """Dispatch a DimosMsg to its default vis type."""
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
                f"No default vis type for {type(msg).__name__}. "
                f"Wrap it explicitly (e.g. Pose(msg), Arrow(msg))."
            )

    def add_embedded_observation(self, obs: EmbeddedObservation[Any], **kwargs: Any) -> None:
        """Store embedded observation directly — each renderer decides presentation."""
        self._elements.append(obs)

    def add_observation(self, obs: Observation[Any], **kwargs: Any) -> None:
        """Smart dispatch: inspect observation data type to pick vis type."""
        from dimos.msgs.sensor_msgs.Image import Image

        data = obs.data
        if isinstance(data, Image):
            self._elements.append(
                Camera(
                    pose=obs.pose_stamped,
                    image=data,
                    **kwargs,
                )
            )
        else:
            self._elements.append(Pose(msg=obs.pose_stamped, **kwargs))

    def base_map(self, grid: OccupancyGrid) -> Drawing2D:
        """Add an OccupancyGrid as the background map."""
        return self.add(grid)

    def to_svg(self, path: str | None = None) -> str:
        """Render to SVG string. Optionally write to file."""
        from dimos.memory2.vis.drawing2d.svg import render

        svg = render(self)
        if path is not None:
            with open(path, "w") as f:
                f.write(svg)
        return svg

    def to_rerun(self, app_id: str = "drawing2d", spawn: bool = True) -> None:
        """Render to Rerun viewer."""
        from dimos.memory2.vis.drawing2d.rerun import render

        render(self, app_id=app_id, spawn=spawn)

    def _repr_svg_(self) -> str:
        """Jupyter inline display."""
        return self.to_svg()

    @property
    def elements(self) -> list[SceneElement]:
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
        return f"Drawing2D({', '.join(parts)})"


# Backwards compatibility alias
Drawing = Drawing2D
