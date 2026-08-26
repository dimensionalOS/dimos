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

"""Backend-neutral, display-only manipulation visualization layers."""

from __future__ import annotations

from dataclasses import dataclass
import math
import re
from typing import TypeAlias

import numpy as np
from numpy.typing import NDArray

_ID_SEGMENT = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]*$")


def _validate_id(value: str, *, hierarchical: bool) -> str:
    if not isinstance(value, str) or not value:
        raise ValueError("visualization ID must be a nonempty string")
    segments = value.split("/") if hierarchical else [value]
    if any(not _ID_SEGMENT.fullmatch(segment) for segment in segments):
        kind = "layer" if hierarchical else "element"
        raise ValueError(f"{kind} ID contains an invalid segment: {value!r}")
    return value


def _snapshot_positions(value: NDArray[np.generic], *, name: str) -> NDArray[np.float32]:
    result = np.array(value, dtype=np.float32, copy=True)
    if result.ndim != 2 or result.shape[1:] != (3,):
        raise ValueError(f"{name} must have shape (N, 3)")
    if not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain only finite values")
    result.setflags(write=False)
    return result


def _snapshot_colors(
    value: NDArray[np.generic] | None,
    *,
    count: int,
    allow_uniform: bool,
) -> NDArray[np.uint8] | None:
    if value is None:
        return None
    source = np.asarray(value)
    valid_shapes: set[tuple[int, ...]] = {(count, 3)}
    if allow_uniform:
        valid_shapes.add((3,))
    if source.shape not in valid_shapes:
        expected = "(3,) or (N, 3)" if allow_uniform else "(N, 3)"
        raise ValueError(f"colors must have shape {expected}")
    if not np.issubdtype(source.dtype, np.number):
        raise ValueError("colors must be numeric RGB values")
    numeric = np.asarray(source, dtype=np.float64)
    if not np.all(np.isfinite(numeric)):
        raise ValueError("colors must contain only finite values")
    if np.issubdtype(source.dtype, np.floating) and np.all((numeric >= 0.0) & (numeric <= 1.0)):
        numeric = np.rint(numeric * 255.0)
    if np.any(numeric < 0.0) or np.any(numeric > 255.0):
        raise ValueError("colors must be in [0, 1] or [0, 255]")
    if not np.all(numeric == np.rint(numeric)):
        raise ValueError("colors above 1 must be integer RGB values")
    result = np.array(numeric, dtype=np.uint8, copy=True)
    result.setflags(write=False)
    return result


def _validate_size(value: float | None, *, name: str) -> float | None:
    if value is None:
        return None
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f"{name} must be finite and positive")
    return result


def _snapshot_triangles(value: NDArray[np.generic], *, vertex_count: int) -> NDArray[np.int32]:
    source = np.asarray(value)
    if source.ndim != 2 or source.shape[1:] != (3,):
        raise ValueError("triangles must have shape (M, 3)")
    if not np.issubdtype(source.dtype, np.number):
        raise ValueError("triangles must contain integer indices")
    numeric = np.asarray(source, dtype=np.float64)
    if not np.all(np.isfinite(numeric)) or not np.all(numeric == np.rint(numeric)):
        raise ValueError("triangles must contain finite integer indices")
    if np.any(numeric < 0) or (numeric.size and np.any(numeric >= vertex_count)):
        raise ValueError("triangles contain an out-of-range vertex index")
    result = np.array(numeric, dtype=np.int32, copy=True)
    result.setflags(write=False)
    return result


def _validate_opacity(value: float) -> float:
    result = float(value)
    if not math.isfinite(result) or not 0.0 < result <= 1.0:
        raise ValueError("opacity must be finite and in (0, 1]")
    return result


@dataclass(frozen=True)
class PointCloudElement:
    """A generic colored point cloud with no planning authority."""

    id: str
    points: NDArray[np.generic]
    colors: NDArray[np.generic] | None = None
    point_size: float | None = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "id", _validate_id(self.id, hierarchical=False))
        points = _snapshot_positions(self.points, name="points")
        object.__setattr__(self, "points", points)
        object.__setattr__(
            self,
            "colors",
            _snapshot_colors(self.colors, count=len(points), allow_uniform=False),
        )
        object.__setattr__(self, "point_size", _validate_size(self.point_size, name="point_size"))


@dataclass(frozen=True)
class LineSetElement:
    """Indexed line geometry with optional uniform or per-line RGB."""

    id: str
    vertices: NDArray[np.generic]
    edges: NDArray[np.generic]
    colors: NDArray[np.generic] | None = None
    line_width: float | None = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "id", _validate_id(self.id, hierarchical=False))
        vertices = _snapshot_positions(self.vertices, name="vertices")
        object.__setattr__(self, "vertices", vertices)

        source_edges = np.asarray(self.edges)
        if source_edges.ndim != 2 or source_edges.shape[1:] != (2,):
            raise ValueError("edges must have shape (M, 2)")
        if not np.issubdtype(source_edges.dtype, np.number):
            raise ValueError("edges must contain integer indices")
        numeric_edges = np.asarray(source_edges, dtype=np.float64)
        if not np.all(np.isfinite(numeric_edges)) or not np.all(
            numeric_edges == np.rint(numeric_edges)
        ):
            raise ValueError("edges must contain finite integer indices")
        if np.any(numeric_edges < 0) or (
            numeric_edges.size and np.any(numeric_edges >= len(vertices))
        ):
            raise ValueError("edges contain an out-of-range vertex index")
        edges = np.array(numeric_edges, dtype=np.int32, copy=True)
        edges.setflags(write=False)
        object.__setattr__(self, "edges", edges)
        object.__setattr__(
            self,
            "colors",
            _snapshot_colors(self.colors, count=len(edges), allow_uniform=True),
        )
        object.__setattr__(self, "line_width", _validate_size(self.line_width, name="line_width"))


@dataclass(frozen=True)
class MeshElement:
    """Indexed triangle mesh with a uniform RGB color and opacity."""

    id: str
    vertices: NDArray[np.generic]
    triangles: NDArray[np.generic]
    color: NDArray[np.generic]
    opacity: float = 1.0

    def __post_init__(self) -> None:
        object.__setattr__(self, "id", _validate_id(self.id, hierarchical=False))
        vertices = _snapshot_positions(self.vertices, name="vertices")
        object.__setattr__(self, "vertices", vertices)
        object.__setattr__(
            self,
            "triangles",
            _snapshot_triangles(self.triangles, vertex_count=len(vertices)),
        )
        color = _snapshot_colors(self.color, count=1, allow_uniform=True)
        if color is None or color.ndim != 1:
            raise ValueError("color must have shape (3,)")
        object.__setattr__(self, "color", color)
        object.__setattr__(self, "opacity", _validate_opacity(self.opacity))


VisualizationElement: TypeAlias = PointCloudElement | LineSetElement | MeshElement


@dataclass(frozen=True)
class VisualizationLayer:
    """A complete, owner-scoped set of display-only visual elements."""

    id: str
    frame_id: str
    elements: tuple[VisualizationElement, ...]
    default_visible: bool = True

    def __post_init__(self) -> None:
        object.__setattr__(self, "id", _validate_id(self.id, hierarchical=True))
        if not isinstance(self.frame_id, str) or not self.frame_id.strip():
            raise ValueError("frame_id must be a nonempty string")
        object.__setattr__(self, "frame_id", self.frame_id.strip())
        elements = tuple(self.elements)
        if any(
            not isinstance(item, (PointCloudElement, LineSetElement, MeshElement))
            for item in elements
        ):
            raise TypeError("elements must be point-cloud, line-set, or mesh elements")
        ids = [item.id for item in elements]
        if len(ids) != len(set(ids)):
            raise ValueError("element IDs must be unique within a layer")
        object.__setattr__(self, "elements", elements)
