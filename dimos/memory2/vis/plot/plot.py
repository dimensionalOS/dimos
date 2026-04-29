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

"""Plot: 2D chart builder for memory2 visualization."""

from __future__ import annotations

from enum import Enum
from typing import Any

from dimos.memory2.vis.plot.elements import HLine, Markers, PlotElement, Series, VLine


class TimeAxis(str, Enum):
    """How the x-axis is formatted.

    - ``raw``: unix timestamps as-is (matplotlib's default numeric formatter).
    - ``relative``: seconds since the first sample, e.g. ``"0s"``, ``"60s"``.
    - ``absolute``: wall-clock time from the system timezone, e.g. ``"11:09:11"``.
    """

    raw = "raw"
    relative = "relative"
    absolute = "absolute"


class Plot:
    """2D chart. Today treats X as time; generalized axes are follow-up work.

    Elements can be added as:
    - Series/Markers/HLine directly
    - Stream[float] → materializes, extracts obs.ts/obs.data into Series
    - list[Observation[float]] → extracts obs.ts/obs.data into Series
    """

    def __init__(self, time_axis: TimeAxis = TimeAxis.relative) -> None:
        self._elements: list[PlotElement] = []
        self.time_axis = time_axis

    def add(self, element: Any, **kwargs: Any) -> Plot:
        """Add a plot element with smart dispatch."""
        from dimos.memory2.stream import Stream
        from dimos.memory2.type.observation import Observation

        if isinstance(element, (Series, Markers, HLine, VLine)):
            self._elements.append(element)
        elif isinstance(element, Stream):
            self._add_from_observations(element.to_list(), **kwargs)
        elif isinstance(element, list) and element and isinstance(element[0], Observation):
            self._add_from_observations(element, **kwargs)
        elif hasattr(element, "__iter__"):
            # Try as iterable of observations
            items = list(element)
            if items and isinstance(items[0], Observation):
                self._add_from_observations(items, **kwargs)
            else:
                raise TypeError(f"Plot.add() cannot handle iterable of {type(items[0]).__name__}.")
        else:
            raise TypeError(
                f"Plot.add() does not know how to handle {type(element).__name__}. "
                f"Pass Series, Markers, HLine, VLine, a Stream, or a list of Observations."
            )

        return self

    def _add_from_observations(self, obs_list: list[Any], **kwargs: Any) -> None:
        """Convert observations to a Series (ts → x, data → y)."""
        ts = [obs.ts for obs in obs_list]
        values = [float(obs.data) for obs in obs_list]
        self._elements.append(Series(ts=ts, values=values, **kwargs))

    def to_svg(self, path: str | None = None) -> str:
        """Render to SVG string. Optionally write to file."""
        from dimos.memory2.vis.color import resolve_deferred
        from dimos.memory2.vis.plot.svg import render

        resolve_deferred(self._elements)
        svg = render(self)
        if path is not None:
            with open(path, "w") as f:
                f.write(svg)
        return svg

    def to_rerun(self, app_id: str = "plot", spawn: bool = True) -> None:
        """Render to Rerun viewer (placeholder — currently a no-op)."""
        from dimos.memory2.vis.color import resolve_deferred
        from dimos.memory2.vis.plot.rerun import render

        resolve_deferred(self._elements)
        render(self, app_id=app_id, spawn=spawn)

    def _repr_svg_(self) -> str:
        """Jupyter inline display."""
        return self.to_svg()

    @property
    def elements(self) -> list[PlotElement]:
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
        return f"Plot({', '.join(parts)})"
