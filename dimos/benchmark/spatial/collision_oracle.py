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

"""Conservative, deterministic planar collision and geometry-candidate oracles.

The implementation intentionally uses only NumPy and exact segment predicates.
It does not depend on optional geometry packages, so corpus labels are portable
across generation environments.  Polygon-boundary contact is always treated as
collision.  Translation evaluates exact convex swept segments; rotation samples
at the configured angular bound and expands each sample enough to contain the
intervening continuous rotation.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from enum import Enum
import math

import numpy as np
from shapely.geometry import LineString, Point, Polygon
from shapely.ops import unary_union

from dimos.benchmark.spatial.config import GeometryToleranceConfig, SquareFootprintConfig
from dimos.benchmark.spatial.models import BarrierSegment, Point2D, Polygon2D, Pose2D

_EPSILON = 1e-12


class CandidateDisposition(str, Enum):
    """Outcome for an oracle evaluation or candidate-stability check."""

    CLEAR = "clear"
    COLLISION = "collision"
    REJECTED_UNCERTAIN = "rejected-uncertain"


@dataclass(frozen=True)
class OracleEvaluation:
    """A deterministic oracle result with a generator-actionable explanation."""

    disposition: CandidateDisposition
    reason: str

    @property
    def is_collision(self) -> bool:
        """Whether the nominal geometry collides (including boundary contact)."""

        return self.disposition is CandidateDisposition.COLLISION

    @property
    def is_retained(self) -> bool:
        """Whether this geometry is stable enough to retain as a corpus candidate."""

        return self.disposition is not CandidateDisposition.REJECTED_UNCERTAIN


class GeometryCandidateRejectedError(ValueError):
    """Raised when a boolean-only caller attempts to use an uncertain candidate."""


@dataclass(frozen=True)
class GeometryToleranceValidator:
    """Reject geometry whose classification changes inside configured tolerance bands."""

    tolerances: GeometryToleranceConfig

    def validate_opening_candidate(
        self,
        opening: Polygon2D,
        blocking_regions: tuple[Polygon2D, ...],
        *,
        parent_wall_regions: tuple[Polygon2D, ...] = (),
    ) -> OracleEvaluation:
        """Reject an opening whose boundary is too close to blocking geometry.

        Opening extraction supplies only a candidate polygon and the wall/blocked
        polygons it was derived from.  Contact or a distance within the configured
        opening band is deliberately ambiguous and must not become an edge label.
        """

        # An aperture intentionally contacts its parent wall.  Callers provide
        # that wall separately so only unrelated blockers affect clearance.
        parent_regions = tuple(_as_shapely(p) for p in parent_wall_regions)
        blockers = tuple(
            _as_shapely(region)
            for region in blocking_regions
            if not any(_as_shapely(region).equals(parent) for parent in parent_regions)
        )
        opening_shape = _as_shapely(opening)
        distance = min((opening_shape.distance(blocker) for blocker in blockers), default=math.inf)
        if distance <= float(self.tolerances.opening_uncertainty_margin_m) + _EPSILON:
            return OracleEvaluation(
                CandidateDisposition.REJECTED_UNCERTAIN,
                "opening candidate lies within opening_uncertainty_margin_m of blocking geometry",
            )
        return OracleEvaluation(
            CandidateDisposition.CLEAR, "opening candidate is outside uncertainty band"
        )

    def validate_room_boundary_candidate(
        self, point: Point2D, room_boundaries: tuple[Polygon2D, ...]
    ) -> OracleEvaluation:
        """Reject a point candidate close enough to a room boundary to change membership."""

        point_shape = Point(point.x_m, point.y_m)
        distance = min(
            (point_shape.distance(_as_shapely(boundary).boundary) for boundary in room_boundaries),
            default=math.inf,
        )
        if distance <= float(self.tolerances.room_boundary_uncertainty_margin_m) + _EPSILON:
            return OracleEvaluation(
                CandidateDisposition.REJECTED_UNCERTAIN,
                "room candidate lies within room_boundary_uncertainty_margin_m of a boundary",
            )
        return OracleEvaluation(
            CandidateDisposition.CLEAR, "room candidate is outside uncertainty band"
        )


@dataclass(frozen=True)
class SquareFootprintCollisionOracle:
    """Continuous 2D collision oracle for a square robot against authoritative polygons.

    ``floor_regions`` are navigable closed polygons and ``blocked_regions`` are
    closed obstacle polygons.  A footprint must remain strictly inside at least
    one floor region and strictly disjoint from every blocked region.  This
    conservative per-region rule intentionally rejects ambiguous seams between
    separately supplied floor polygons rather than manufacturing a label.
    """

    floor_regions: tuple[Polygon2D, ...]
    blocked_regions: tuple[Polygon2D, ...]
    footprint: SquareFootprintConfig
    tolerances: GeometryToleranceConfig
    barriers: tuple[BarrierSegment, ...] = ()

    def __post_init__(self) -> None:
        if not self.floor_regions:
            raise ValueError("floor_regions must contain at least one navigable polygon")
        _validate_polygon_collection(self.floor_regions, "floor_regions")
        _validate_polygon_collection(self.blocked_regions, "blocked_regions")

    def evaluate_pose(self, pose: Pose2D) -> OracleEvaluation:
        """Evaluate a pose and reject it if tolerance perturbations change its label."""

        self._validate_pose(pose, "pose")
        return self._evaluate_sweep("pose", lambda margin: (self._square_at(pose, margin),))

    def evaluate_translation(self, start_pose: Pose2D, distance_m: float) -> OracleEvaluation:
        """Evaluate the complete fixed-yaw straight translation, not just endpoints."""

        if not math.isfinite(distance_m):
            raise ValueError("distance_m must be finite")
        self._validate_pose(start_pose, "start_pose")
        steps = max(1, math.ceil(abs(distance_m) / float(self.tolerances.translation_sweep_step_m)))
        direction = np.array([math.cos(start_pose.yaw_rad), math.sin(start_pose.yaw_rad)])

        def swept_polygons(margin: float) -> tuple[np.ndarray, ...]:
            polygons: list[np.ndarray] = []
            for index in range(steps):
                first = self._translated_pose(start_pose, direction, distance_m * index / steps)
                second = self._translated_pose(
                    start_pose, direction, distance_m * (index + 1) / steps
                )
                polygons.append(
                    _convex_hull(
                        np.vstack((self._square_at(first, margin), self._square_at(second, margin)))
                    )
                )
            return tuple(polygons)

        return self._evaluate_sweep("translation", swept_polygons)

    def evaluate_rotation(self, pose: Pose2D, yaw_delta_rad: float) -> OracleEvaluation:
        """Evaluate continuous rotation with adaptive actual/outer interval bounds."""

        if not math.isfinite(yaw_delta_rad):
            raise ValueError("yaw_delta_rad must be finite")
        self._validate_pose(pose, "pose")
        uncertainty = float(self.tolerances.collision_uncertainty_margin_m)
        outcomes = tuple(
            self._rotation_interval_result(pose, yaw_delta_rad, margin)
            for margin in (0.0, uncertainty, -uncertainty)
        )
        if CandidateDisposition.REJECTED_UNCERTAIN in outcomes:
            return OracleEvaluation(
                CandidateDisposition.REJECTED_UNCERTAIN,
                "rotation refinement limit cannot prove occupancy",
            )
        if len(set(outcomes)) != 1:
            return OracleEvaluation(
                CandidateDisposition.REJECTED_UNCERTAIN,
                "rotation changes state inside collision_uncertainty_margin_m",
            )
        disposition = outcomes[0]
        return OracleEvaluation(
            disposition,
            "rotation intersects geometry"
            if disposition is CandidateDisposition.COLLISION
            else "rotation is collision-free",
        )

    def _rotation_interval_result(
        self, pose: Pose2D, delta: float, margin: float
    ) -> CandidateDisposition:
        def visit(start: float, end: float, depth: int) -> CandidateDisposition:
            first = self._square_at(Pose2D(x_m=pose.x_m, y_m=pose.y_m, yaw_rad=start), margin)
            second = self._square_at(Pose2D(x_m=pose.x_m, y_m=pose.y_m, yaw_rad=end), margin)
            # Actual squares witness collision.  The expanded hull is only an
            # outer cover and can prove clearance, never collision.
            if self._collides((first, second)):
                return CandidateDisposition.COLLISION
            corner_radius = (self._effective_side + 2.0 * margin) / math.sqrt(2.0)
            arc_expansion = corner_radius * 2.0 * math.sin(abs(end - start) / 4.0)
            outer = _convex_hull(
                np.vstack(
                    (
                        self._square_at(
                            Pose2D(x_m=pose.x_m, y_m=pose.y_m, yaw_rad=start),
                            margin + arc_expansion,
                        ),
                        self._square_at(
                            Pose2D(x_m=pose.x_m, y_m=pose.y_m, yaw_rad=end), margin + arc_expansion
                        ),
                    )
                )
            )
            if not self._collides((outer,)):
                return CandidateDisposition.CLEAR
            if depth >= self.tolerances.rotation_refinement_limit:
                return CandidateDisposition.REJECTED_UNCERTAIN
            middle = (start + end) / 2.0
            left, right = visit(start, middle, depth + 1), visit(middle, end, depth + 1)
            if CandidateDisposition.COLLISION in (left, right):
                return CandidateDisposition.COLLISION
            if CandidateDisposition.REJECTED_UNCERTAIN in (left, right):
                return CandidateDisposition.REJECTED_UNCERTAIN
            return CandidateDisposition.CLEAR

        intervals = max(1, math.ceil(abs(delta) / float(self.tolerances.rotation_sweep_step_rad)))
        start = pose.yaw_rad
        results = tuple(
            visit(start + delta * index / intervals, start + delta * (index + 1) / intervals, 0)
            for index in range(intervals)
        )
        if CandidateDisposition.COLLISION in results:
            return CandidateDisposition.COLLISION
        if CandidateDisposition.REJECTED_UNCERTAIN in results:
            return CandidateDisposition.REJECTED_UNCERTAIN
        return CandidateDisposition.CLEAR

    def pose_is_collision_free(self, pose: Pose2D) -> bool:
        """Return the stable occupancy answer or raise for an uncertain candidate."""

        return self._require_stable(self.evaluate_pose(pose))

    def translation_is_collision_free(self, start_pose: Pose2D, distance_m: float) -> bool:
        """Return the stable translation answer or raise for an uncertain candidate."""

        return self._require_stable(self.evaluate_translation(start_pose, distance_m))

    def rotation_is_collision_free(self, pose: Pose2D, yaw_delta_rad: float) -> bool:
        """Return the stable rotation answer or raise for an uncertain candidate."""

        return self._require_stable(self.evaluate_rotation(pose, yaw_delta_rad))

    @property
    def _effective_side(self) -> float:
        return float(self.footprint.side_length_m) + 2.0 * float(self.footprint.safety_margin_m)

    def _require_stable(self, evaluation: OracleEvaluation) -> bool:
        if not evaluation.is_retained:
            raise GeometryCandidateRejectedError(evaluation.reason)
        return not evaluation.is_collision

    def _evaluate_sweep(
        self, motion_name: str, sweep: Callable[[float], tuple[np.ndarray, ...]]
    ) -> OracleEvaluation:
        nominal = self._collides(sweep(0.0))
        uncertainty = float(self.tolerances.collision_uncertainty_margin_m)
        expanded = self._collides(sweep(uncertainty))
        contracted = self._collides(sweep(-uncertainty))
        if nominal != expanded or nominal != contracted:
            return OracleEvaluation(
                CandidateDisposition.REJECTED_UNCERTAIN,
                f"{motion_name} changes collision state inside collision_uncertainty_margin_m",
            )
        if nominal:
            return OracleEvaluation(
                CandidateDisposition.COLLISION, f"{motion_name} intersects geometry"
            )
        return OracleEvaluation(CandidateDisposition.CLEAR, f"{motion_name} is collision-free")

    def _collides(self, polygons: tuple[np.ndarray, ...]) -> bool:
        floor_union = unary_union(tuple(_as_shapely(region) for region in self.floor_regions))
        blocked_union = unary_union(tuple(_as_shapely(region) for region in self.blocked_regions))
        for polygon in polygons:
            footprint = Polygon(polygon)
            # covers establishes exact union containment; touching its boundary is collision.
            if not floor_union.covers(footprint) or floor_union.boundary.intersects(footprint):
                return True
            if not blocked_union.is_empty and footprint.intersects(blocked_union):
                return True
            if any(
                footprint.intersects(
                    LineString(
                        ((barrier.start.x_m, barrier.start.y_m), (barrier.end.x_m, barrier.end.y_m))
                    )
                )
                for barrier in self.barriers
            ):
                return True
        return False

    def _square_at(self, pose: Pose2D, extra_margin: float) -> np.ndarray:
        half_side = self._effective_side / 2.0 + extra_margin
        if half_side <= 0.0:
            raise ValueError(
                "collision_uncertainty_margin_m must be smaller than half the footprint side"
            )
        corners = np.array(
            (
                (-half_side, -half_side),
                (half_side, -half_side),
                (half_side, half_side),
                (-half_side, half_side),
            ),
            dtype=np.float64,
        )
        cosine, sine = math.cos(pose.yaw_rad), math.sin(pose.yaw_rad)
        rotation = np.array(((cosine, -sine), (sine, cosine)), dtype=np.float64)
        return corners @ rotation.T + np.array((pose.x_m, pose.y_m), dtype=np.float64)

    @staticmethod
    def _validate_pose(pose: Pose2D, name: str) -> None:
        if not all(math.isfinite(value) for value in (pose.x_m, pose.y_m, pose.yaw_rad)):
            raise ValueError(f"{name} must contain only finite x_m, y_m, and yaw_rad values")

    @staticmethod
    def _translated_pose(pose: Pose2D, direction: np.ndarray, distance_m: float) -> Pose2D:
        return Pose2D(
            x_m=pose.x_m + float(direction[0]) * distance_m,
            y_m=pose.y_m + float(direction[1]) * distance_m,
            yaw_rad=pose.yaw_rad,
        )


def _as_array(polygon: Polygon2D) -> np.ndarray:
    return np.array(tuple((point.x_m, point.y_m) for point in polygon.vertices), dtype=np.float64)


def _as_shapely(polygon: Polygon2D) -> Polygon:
    return Polygon(
        tuple((point.x_m, point.y_m) for point in polygon.vertices),
        tuple(tuple((point.x_m, point.y_m) for point in hole) for hole in polygon.holes),
    )


def _validate_polygon_collection(polygons: tuple[Polygon2D, ...], name: str) -> None:
    for index, polygon in enumerate(polygons):
        vertices = _as_array(polygon)
        if not np.isfinite(vertices).all():
            raise ValueError(f"{name}[{index}] contains a non-finite coordinate")
        if abs(_signed_area(vertices)) <= _EPSILON:
            raise ValueError(f"{name}[{index}] has zero area")
        if not _as_shapely(polygon).is_valid:
            raise ValueError(f"{name}[{index}] is not a valid polygon")


def _signed_area(polygon: np.ndarray) -> float:
    return 0.5 * float(
        np.sum(
            polygon[:, 0] * np.roll(polygon[:, 1], -1) - polygon[:, 1] * np.roll(polygon[:, 0], -1)
        )
    )


def _cross(first: np.ndarray, second: np.ndarray) -> float:
    return float(first[0] * second[1] - first[1] * second[0])


def _on_segment(point: np.ndarray, start: np.ndarray, end: np.ndarray) -> bool:
    return abs(_cross(end - start, point - start)) <= _EPSILON and bool(
        np.dot(point - start, point - end) <= _EPSILON
    )


def _segments_touch_or_intersect(
    first_start: np.ndarray, first_end: np.ndarray, second_start: np.ndarray, second_end: np.ndarray
) -> bool:
    first_cross_start = _cross(first_end - first_start, second_start - first_start)
    first_cross_end = _cross(first_end - first_start, second_end - first_start)
    second_cross_start = _cross(second_end - second_start, first_start - second_start)
    second_cross_end = _cross(second_end - second_start, first_end - second_start)
    if (
        (first_cross_start > _EPSILON and first_cross_end < -_EPSILON)
        or (first_cross_start < -_EPSILON and first_cross_end > _EPSILON)
    ) and (
        (second_cross_start > _EPSILON and second_cross_end < -_EPSILON)
        or (second_cross_start < -_EPSILON and second_cross_end > _EPSILON)
    ):
        return True
    return any(
        (
            abs(first_cross_start) <= _EPSILON
            and _on_segment(second_start, first_start, first_end),
            abs(first_cross_end) <= _EPSILON and _on_segment(second_end, first_start, first_end),
            abs(second_cross_start) <= _EPSILON
            and _on_segment(first_start, second_start, second_end),
            abs(second_cross_end) <= _EPSILON and _on_segment(first_end, second_start, second_end),
        )
    )


def _point_in_polygon(point: np.ndarray, polygon: np.ndarray) -> bool:
    """Return true for points inside or on a simple polygon boundary."""

    inside = False
    for start, end in zip(polygon, np.roll(polygon, -1, axis=0), strict=True):
        if _on_segment(point, start, end):
            return True
        if (start[1] > point[1]) != (end[1] > point[1]):
            crossing_x = (end[0] - start[0]) * (point[1] - start[1]) / (end[1] - start[1]) + start[
                0
            ]
            if point[0] < crossing_x:
                inside = not inside
    return inside


def _strictly_contains_polygon(container: np.ndarray, candidate: np.ndarray) -> bool:
    for start, end in zip(container, np.roll(container, -1, axis=0), strict=True):
        for candidate_start, candidate_end in zip(
            candidate, np.roll(candidate, -1, axis=0), strict=True
        ):
            if _segments_touch_or_intersect(start, end, candidate_start, candidate_end):
                return False
    return all(_point_in_polygon(point, container) for point in candidate)


def _covered_by_floor_union(candidate: np.ndarray, floors: tuple[np.ndarray, ...]) -> bool:
    """Require the footprint boundary to be inside the navigable polygon union.

    Unlike a per-room containment test this admits an internal seam, while a
    point on an exterior boundary has support from only one region and remains a
    collision.  Barriers subsequently close every non-door seam.
    """
    if any(_strictly_contains_polygon(floor, candidate) for floor in floors):
        return True
    for point in candidate:
        containing = sum(_point_in_polygon(point, floor) for floor in floors)
        if containing == 0:
            return False
        boundary_count = sum(
            any(
                _on_segment(point, start, end)
                for start, end in zip(floor, np.roll(floor, -1, axis=0), strict=True)
            )
            for floor in floors
        )
        if boundary_count == 1:
            return False
    for start, end in zip(candidate, np.roll(candidate, -1, axis=0), strict=True):
        midpoint = (start + end) / 2.0
        if not any(_point_in_polygon(midpoint, floor) for floor in floors):
            return False
    return True


def _polygon_touches_segment(polygon: np.ndarray, barrier: BarrierSegment) -> bool:
    start = np.array((barrier.start.x_m, barrier.start.y_m), dtype=np.float64)
    end = np.array((barrier.end.x_m, barrier.end.y_m), dtype=np.float64)
    return (
        any(
            _segments_touch_or_intersect(first, second, start, end)
            for first, second in zip(polygon, np.roll(polygon, -1, axis=0), strict=True)
        )
        or _point_in_polygon(start, polygon)
        or _point_in_polygon(end, polygon)
    )


def _polygons_touch_or_intersect(first: np.ndarray, second: np.ndarray) -> bool:
    for first_start, first_end in zip(first, np.roll(first, -1, axis=0), strict=True):
        for second_start, second_end in zip(second, np.roll(second, -1, axis=0), strict=True):
            if _segments_touch_or_intersect(first_start, first_end, second_start, second_end):
                return True
    return _point_in_polygon(first[0], second) or _point_in_polygon(second[0], first)


def _convex_hull(points: np.ndarray) -> np.ndarray:
    """Return a counter-clockwise convex hull using deterministic monotonic chaining."""

    ordered = sorted((float(point[0]), float(point[1])) for point in points)
    unique = list(dict.fromkeys(ordered))
    if len(unique) < 3:
        raise ValueError("swept footprint must contain at least three distinct points")

    def build_half(candidates: list[tuple[float, float]]) -> list[tuple[float, float]]:
        half: list[tuple[float, float]] = []
        for candidate in candidates:
            while len(half) >= 2:
                start = np.array(half[-2])
                end = np.array(half[-1])
                current = np.array(candidate)
                if _cross(end - start, current - end) > _EPSILON:
                    break
                half.pop()
            half.append(candidate)
        return half

    lower = build_half(unique)
    upper = build_half(list(reversed(unique)))
    return np.array(lower[:-1] + upper[:-1], dtype=np.float64)


def _point_segment_distance(point: np.ndarray, start: np.ndarray, end: np.ndarray) -> float:
    segment = end - start
    length_squared = float(np.dot(segment, segment))
    if length_squared <= _EPSILON:
        return float(np.linalg.norm(point - start))
    fraction = float(np.clip(np.dot(point - start, segment) / length_squared, 0.0, 1.0))
    return float(np.linalg.norm(point - (start + fraction * segment)))


def _point_polygon_set_distance(point: np.ndarray, polygons: tuple[np.ndarray, ...]) -> float:
    if not polygons:
        return math.inf
    return min(
        _point_segment_distance(point, start, end)
        for polygon in polygons
        for start, end in zip(polygon, np.roll(polygon, -1, axis=0), strict=True)
    )


def _polygon_set_distance(polygon: np.ndarray, other_polygons: tuple[np.ndarray, ...]) -> float:
    if not other_polygons:
        return math.inf
    if any(_polygons_touch_or_intersect(polygon, other) for other in other_polygons):
        return 0.0
    distances = [
        _point_segment_distance(point, start, end)
        for other in other_polygons
        for point in polygon
        for start, end in zip(other, np.roll(other, -1, axis=0), strict=True)
    ]
    distances.extend(
        _point_segment_distance(point, start, end)
        for other in other_polygons
        for point in other
        for start, end in zip(polygon, np.roll(polygon, -1, axis=0), strict=True)
    )
    return min(distances)
