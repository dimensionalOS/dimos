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

"""Validated 2-D geometry for generation-time objectivity gates."""

from __future__ import annotations

from shapely.geometry import MultiPolygon, Point, Polygon
from shapely.geometry.base import BaseGeometry
from shapely.ops import unary_union

from dimos.benchmark.dimsim.models import NavigationGeometry, Point2, Polygon2


class GeometryGateError(ValueError):
    """Raised when authoritative geometry cannot support a stable predicate."""


def polygon(value: Polygon2) -> Polygon:
    result = Polygon(value)
    if not result.is_valid or result.is_empty or result.area <= 0:
        raise GeometryGateError("footprint must be a valid non-empty polygon")
    return result


def surface_distance(first: Polygon2, second: Polygon2) -> float:
    return float(polygon(first).distance(polygon(second)))


def outer_edge_distance(point: Point2, target: Polygon2) -> float:
    target_polygon = polygon(target)
    return float(Point(point).distance(target_polygon.boundary))


def stopping_band(
    target: Polygon2,
    threshold_m: float,
    footprint_radius_m: float = 0.0,
) -> BaseGeometry:
    if threshold_m <= 0:
        raise GeometryGateError("stopping threshold must be positive")
    if footprint_radius_m < 0:
        raise GeometryGateError("footprint radius must be non-negative")
    shape = polygon(target)
    return shape.buffer(threshold_m + footprint_radius_m).difference(shape)


def feasible_stopping_region(
    target: Polygon2,
    navigation: NavigationGeometry,
    spawn: Point2,
    threshold_m: float,
    footprint_radius_m: float,
    clearance_m: float,
) -> BaseGeometry:
    required_clearance = footprint_radius_m + clearance_m
    remaining_clearance = max(0.0, required_clearance - navigation.clearance_radius_m)
    navigable = unary_union([polygon(item) for item in navigation.navigable])
    if remaining_clearance:
        navigable = navigable.buffer(-remaining_clearance)
    blocked = unary_union([polygon(item).buffer(required_clearance) for item in navigation.blocked])
    free = navigable.difference(blocked)
    feasible = stopping_band(target, threshold_m, footprint_radius_m).intersection(free)
    if feasible.is_empty:
        raise GeometryGateError("no collision-free stopping region")

    spawn_point = Point(spawn)
    components: list[BaseGeometry] = list(free.geoms) if isinstance(free, MultiPolygon) else [free]
    reachable = next(
        (component for component in components if component.covers(spawn_point)),
        None,
    )
    if reachable is None or reachable.intersection(feasible).is_empty:
        raise GeometryGateError("stopping region is unreachable from canonical spawn")
    return reachable.intersection(feasible)
