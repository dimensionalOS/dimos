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

"""Element types for Plot (2D charts)."""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum
from typing import Union


class Style(StrEnum):
    """Line style for Series and HLine elements.

    Values match matplotlib's `linestyle` names so they pass through directly
    to the renderer without translation.
    """

    solid = "solid"
    dashed = "dashed"
    dotted = "dotted"


@dataclass
class Series:
    """Line connecting (t, y) points.

    ``connect`` is the maximum gap (in x-axis units, typically seconds) over
    which the renderer will draw a connecting line. Samples whose neighbors
    are further apart get visually separated — useful when a stream has
    holes (e.g. an embedding stream that skipped dark frames). Set to
    ``None`` to always connect regardless of gap size.

    ``gap_fill`` controls what happens at a gap. ``None`` (default) breaks
    the line entirely. A float value drops the line to that value across
    the gap, producing a "valley" — set ``gap_fill=0.0`` to render holes as
    drops to zero.
    """

    ts: list[float]
    values: list[float]
    color: str | None = None
    width: float = 1.5
    label: str | None = None
    axis: str | None = None
    opacity: float = 1.0
    style: Style = Style.solid
    connect: float | None = 2.0
    gap_fill: float | None = None


@dataclass
class Markers:
    """Scatter dots at (t, y) points."""

    ts: list[float]
    values: list[float]
    color: str | None = None
    radius: float = 0.5
    label: str | None = None
    axis: str | None = None
    opacity: float = 1.0


@dataclass
class HLine:
    """Horizontal reference line."""

    y: float
    color: str = "#888888"
    style: Style = Style.dashed
    label: str | None = None
    axis: str | None = None
    opacity: float = 1.0


@dataclass
class VLine:
    """Vertical reference line.

    Always draws on the primary x-axis — twin axes all share the same x, so
    there's no need for an ``axis`` field: the line spans the full y range
    regardless of which axes owns it.
    """

    x: float
    color: str = "#888888"
    style: Style = Style.dashed
    label: str | None = None
    opacity: float = 1.0


PlotElement = Union[Series, Markers, HLine, VLine]
