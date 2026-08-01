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

# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
"""Strict Structured3D import into one private 2-D free-space model."""

from __future__ import annotations

from math import hypot, isclose
from pathlib import Path

from pydantic import BaseModel, ConfigDict, Field, PositiveFloat, field_validator, model_validator
from shapely.geometry import LineString, MultiLineString, Point, Polygon
from shapely.geometry.base import BaseGeometry
from shapely.ops import unary_union

from dimos.benchmark.spatial.config import SPATIAL_BENCHMARK_V1, GeometryToleranceConfig
from dimos.benchmark.spatial.models import (
    BarrierSegment,
    CoverageConnector,
    FreeSpaceModel,
    Geometry,
    OpeningEdge,
    Point2D,
    Polygon2D,
    Room,
    SourceProvenance,
    SpatialModel,
    Topology,
)
from dimos.benchmark.spatial.utilities import hash_file_sha256, stable_opaque_id

_EPSILON_M = 1e-6
# This is deliberately a policy version rather than a substring blacklist.
ROOM_SEMANTIC_POLICY_VERSION = "structured3d-room-labels-v1"
SUPPORTED_ROOM_SEMANTICS = frozenset(
    {
        "living room",
        "bedroom",
        "kitchen",
        "bathroom",
        "dining room",
        "study",
        "office",
        "hallway",
        "corridor",
    }
)


class Structured3DError(ValueError):
    """A source scene cannot safely provide oracle geometry."""


class Structured3DSourceModel(BaseModel):
    """Strict keys but JSON-compatible containers for documented source artifacts."""

    model_config = ConfigDict(extra="forbid", frozen=True)


class SourceAxisTransform(SpatialModel):
    axes: tuple[
        tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]
    ] = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    scale_m_per_source_unit: PositiveFloat = 0.001

    @model_validator(mode="after")
    def validate_proper_rotation(self) -> SourceAxisTransform:
        first, second, third = self.axes
        determinant = (
            first[0] * (second[1] * third[2] - second[2] * third[1])
            - first[1] * (second[0] * third[2] - second[2] * third[0])
            + first[2] * (second[0] * third[1] - second[1] * third[0])
        )
        values = (
            sum(value * value for value in first),
            sum(value * value for value in second),
            sum(value * value for value in third),
            sum(left * right for left, right in zip(first, second, strict=True)),
            sum(left * right for left, right in zip(first, third, strict=True)),
            sum(left * right for left, right in zip(second, third, strict=True)),
        )
        if (
            not all(isclose(value, 1.0, abs_tol=1e-9) for value in values[:3])
            or not all(isclose(value, 0.0, abs_tol=1e-9) for value in values[3:])
            or not isclose(determinant, 1.0, abs_tol=1e-9)
        ):
            raise ValueError("source axis transform must be a proper orthonormal rotation")
        return self


class Structured3DJunction(Structured3DSourceModel):
    ID: int
    coordinate: tuple[float, float, float]


class Structured3DLine(Structured3DSourceModel):
    ID: int
    point: tuple[float, float, float]
    direction: tuple[float, float, float]


class Structured3DPlane(Structured3DSourceModel):
    ID: int
    type: str
    normal: tuple[float, float, float]
    offset: float


class Structured3DSemantic(Structured3DSourceModel):
    ID: int
    type: str
    plane_ids: tuple[int, ...] = Field(alias="planeID", min_length=1)


class Structured3DCuboid(Structured3DSourceModel):
    ID: int
    plane_ids: tuple[int, ...] = Field(alias="planeID")


class Structured3DManhattan(Structured3DSourceModel):
    ID: int
    plane_ids: tuple[int, ...] = Field(alias="planeID")


class Structured3DAnnotation(Structured3DSourceModel):
    junctions: tuple[Structured3DJunction, ...] = Field(min_length=1)
    lines: tuple[Structured3DLine, ...] = Field(min_length=1)
    planes: tuple[Structured3DPlane, ...] = Field(min_length=1)
    semantics: tuple[Structured3DSemantic, ...] = Field(min_length=1)
    plane_line_matrix: tuple[tuple[int, ...], ...] = Field(alias="planeLineMatrix")
    line_junction_matrix: tuple[tuple[int, ...], ...] = Field(alias="lineJunctionMatrix")
    cuboids: tuple[Structured3DCuboid, ...] = ()
    manhattan: tuple[Structured3DManhattan, ...] = ()

    @field_validator("plane_line_matrix", "line_junction_matrix")
    @classmethod
    def validate_binary_matrix(
        cls, matrix: tuple[tuple[int, ...], ...]
    ) -> tuple[tuple[int, ...], ...]:
        if any(value not in (0, 1) for row in matrix for value in row):
            raise ValueError("incidence matrices must contain only 0 or 1")
        return matrix

    @model_validator(mode="after")
    def validate_indices(self) -> Structured3DAnnotation:
        collections = (
            ("junction", self.junctions),
            ("line", self.lines),
            ("plane", self.planes),
        )
        for name, values in collections:
            if tuple(item.ID for item in values) != tuple(range(len(values))):
                raise ValueError(f"Structured3D {name} IDs must equal array indices")
        semantic_ids = tuple(item.ID for item in self.semantics)
        if len(set(semantic_ids)) != len(semantic_ids):
            raise ValueError("Structured3D semantic IDs must be unique")
        if len(self.plane_line_matrix) != len(self.planes) or any(
            len(row) != len(self.lines) for row in self.plane_line_matrix
        ):
            raise ValueError("planeLineMatrix shape does not match plane and line arrays")
        if len(self.line_junction_matrix) != len(self.lines) or any(
            len(row) != len(self.junctions) for row in self.line_junction_matrix
        ):
            raise ValueError("lineJunctionMatrix shape does not match line and junction arrays")
        if any(
            plane_id < 0 or plane_id >= len(self.planes)
            for semantic in self.semantics
            for plane_id in semantic.plane_ids
        ):
            raise ValueError("semantic planeID references an unknown plane")
        return self


class PlaneGeometry(SpatialModel):
    source_plane_id: int
    semantic_type: str
    contours_m: tuple[tuple[tuple[float, float, float], ...], ...] = Field(min_length=1)

    @property
    def vertices_m(self) -> tuple[tuple[float, float, float], ...]:
        """Compatibility view of the exterior contour."""
        return self.contours_m[0]


class Structured3DImport(SpatialModel):
    source_provenance: SourceProvenance
    source_to_benchmark: SourceAxisTransform
    geometry: Geometry
    topology: Topology
    blocked_geometry: tuple[PlaneGeometry, ...]
    opening_geometry: tuple[PlaneGeometry, ...]
    free_space: FreeSpaceModel


def load_structured3d_scene(
    annotation_path: Path,
    *,
    scene_id: str,
    source_scene_key: str,
    source_revision: str,
    source_dataset: str = "Structured3D",
    source_to_benchmark: SourceAxisTransform | None = None,
    known_invalid_scene_keys: frozenset[str] = frozenset(),
    tolerances: GeometryToleranceConfig | None = None,
) -> Structured3DImport:
    if not annotation_path.is_file():
        raise Structured3DError(f"Structured3D annotation does not exist: {annotation_path}")
    if source_scene_key in known_invalid_scene_keys:
        raise Structured3DError(f"scene {source_scene_key!r} is listed as invalid")
    try:
        annotation = Structured3DAnnotation.model_validate_json(annotation_path.read_bytes())
    except (OSError, ValueError) as error:
        raise Structured3DError(
            f"invalid Structured3D annotation {annotation_path}: {error}"
        ) from error
    transform = source_to_benchmark or SourceAxisTransform()
    policy = tolerances or SPATIAL_BENCHMARK_V1.geometry_tolerances
    contours = tuple(
        _plane_contours(
            annotation,
            index,
            tuple(_transform(item.coordinate, transform) for item in annotation.junctions),
        )
        for index in range(len(annotation.planes))
    )
    semantic_by_plane = _semantic_by_plane(annotation)
    floors: list[tuple[int, Polygon2D]] = []
    floor_holes: list[BarrierSegment] = []
    walls: list[PlaneGeometry] = []
    openings: list[PlaneGeometry] = []
    for index, plane in enumerate(annotation.planes):
        semantic = semantic_by_plane[index]
        label = semantic.type.casefold() if semantic else ""
        if (
            plane.type.casefold() == "floor"
            and semantic is not None
            and label in SUPPORTED_ROOM_SEMANTICS
        ):
            floor = _floor_polygon(plane, contours[index], transform)
            floors.append((index, floor))
            floor_holes.extend(_hole_barriers(floor.holes))
        if plane.type.casefold() == "wall":
            walls.append(
                PlaneGeometry(
                    source_plane_id=plane.ID,
                    semantic_type=label or "wall",
                    contours_m=contours[index],
                )
            )
        if semantic is not None and label in {"door", "window"}:
            openings.append(
                PlaneGeometry(
                    source_plane_id=plane.ID, semantic_type=label, contours_m=contours[index]
                )
            )
    if not floors or not walls:
        raise Structured3DError("scene has no eligible structural floors or walls")
    _validate_single_floor(tuple(contours[index][0][0][2] for index, _ in floors))
    rooms = tuple(
        Room(
            room_id=stable_opaque_id(
                "room", {"scene": scene_id, "floor": annotation.planes[index].ID}
            ),
            boundary=polygon,
        )
        for index, polygon in floors
    )
    _validate_room_regions(rooms)
    internal, opening_polygons, opening_spans = _validated_internal_openings(
        scene_id, rooms, tuple(openings), policy
    )
    wall_barriers = _barriers(tuple(walls), opening_spans)
    _reject_open_plan_subdivisions(rooms, opening_spans, wall_barriers, policy)
    barriers = wall_barriers + tuple(floor_holes)
    free_space = FreeSpaceModel(
        floor_regions=tuple(room.boundary for room in rooms) + opening_polygons,
        barriers=barriers,
        coverage_connectors=_coverage_connectors(
            tuple(room.boundary for room in rooms) + opening_polygons,
            rooms,
            opening_spans,
            barriers,
            policy,
        ),
    )
    topology = Topology(scene_id=scene_id, rooms=rooms, direct_openings=internal)
    _validate_topology(topology)
    provenance = SourceProvenance(
        scene_id=scene_id,
        source_dataset=source_dataset,
        source_scene_key=source_scene_key,
        source_revision=source_revision,
        source_artifact_sha256=hash_file_sha256(annotation_path),
        coordinate_frame_description=f"Structured3D metric transform; axes={transform.axes}; scale_m_per_source_unit={transform.scale_m_per_source_unit}",
    )
    return Structured3DImport(
        source_provenance=provenance,
        source_to_benchmark=transform,
        geometry=Geometry(
            scene_id=scene_id,
            floor_regions=free_space.floor_regions,
            blocked_regions=(),
            openings=opening_polygons,
            barrier_segments=barriers,
        ),
        topology=topology,
        blocked_geometry=tuple(walls),
        opening_geometry=tuple(openings),
        free_space=free_space,
    )


def unique_room_for_marker(
    marker: Point2D, topology: Topology, tolerances: GeometryToleranceConfig | None = None
) -> Room:
    margin = float(
        (tolerances or SPATIAL_BENCHMARK_V1.geometry_tolerances).room_boundary_uncertainty_margin_m
    )
    matches = tuple(
        room
        for room in topology.rooms
        if _contains(room.boundary, marker) and _boundary_distance(marker, room.boundary) > margin
    )
    if len(matches) != 1:
        raise Structured3DError(
            f"marker must belong unambiguously to one eligible room; found {len(matches)}"
        )
    return matches[0]


def _transform(
    point: tuple[float, float, float], transform: SourceAxisTransform
) -> tuple[float, float, float]:
    return (
        transform.scale_m_per_source_unit
        * sum(transform.axes[0][column] * point[column] for column in range(3)),
        transform.scale_m_per_source_unit
        * sum(transform.axes[1][column] * point[column] for column in range(3)),
        transform.scale_m_per_source_unit
        * sum(transform.axes[2][column] * point[column] for column in range(3)),
    )


def _plane_contours(
    annotation: Structured3DAnnotation,
    plane_index: int,
    junctions: tuple[tuple[float, float, float], ...],
) -> tuple[tuple[tuple[float, float, float], ...], ...]:
    selected = tuple(
        index for index, value in enumerate(annotation.plane_line_matrix[plane_index]) if value
    )
    adjacency: dict[int, set[int]] = {}
    for line_index in selected:
        endpoints = tuple(
            index
            for index, value in enumerate(annotation.line_junction_matrix[line_index])
            if value
        )
        if len(endpoints) != 2:
            raise Structured3DError(f"line {line_index} must have exactly two junctions")
        first, second = endpoints
        adjacency.setdefault(first, set()).add(second)
        adjacency.setdefault(second, set()).add(first)
    if not adjacency or any(len(neighbors) != 2 for neighbors in adjacency.values()):
        raise Structured3DError(f"plane {plane_index} boundaries must be closed contours")
    contours: list[tuple[tuple[float, float, float], ...]] = []
    remaining = set(adjacency)
    while remaining:
        start = min(remaining)
        previous = -1
        current = start
        ordered = [start]
        while True:
            following = next(
                neighbor for neighbor in sorted(adjacency[current]) if neighbor != previous
            )
            if following == start:
                break
            if following in ordered:
                raise Structured3DError(f"plane {plane_index} contour self-intersects")
            ordered.append(following)
            previous, current = current, following
        remaining.difference_update(ordered)
        contours.append(tuple(junctions[index] for index in ordered))
    return tuple(contours)


def _semantic_by_plane(
    annotation: Structured3DAnnotation,
) -> tuple[Structured3DSemantic | None, ...]:
    mapped: dict[int, Structured3DSemantic] = {}
    for semantic in annotation.semantics:
        for plane_id in semantic.plane_ids:
            if plane_id in mapped:
                raise Structured3DError(f"plane {plane_id} has ambiguous semantic membership")
            mapped[plane_id] = semantic
    return tuple(mapped.get(plane.ID) for plane in annotation.planes)


def _floor_polygon(
    plane: Structured3DPlane,
    contours: tuple[tuple[tuple[float, float, float], ...], ...],
    transform: SourceAxisTransform,
) -> Polygon2D:
    normal = _transform(
        plane.normal, SourceAxisTransform(axes=transform.axes, scale_m_per_source_unit=1.0)
    )
    if abs(normal[2]) < 0.99 or any(
        max(point[2] for point in contour) - min(point[2] for point in contour) > _EPSILON_M
        for contour in contours
    ):
        raise Structured3DError(f"floor plane {plane.ID} is not horizontal")
    rings = tuple(tuple(Point2D(x_m=x, y_m=y) for x, y, _ in contour) for contour in contours)
    shapes = tuple(Polygon(tuple((point.x_m, point.y_m) for point in ring)) for ring in rings)
    if any(not shape.is_valid or shape.area <= _EPSILON_M for shape in shapes):
        raise Structured3DError(f"floor plane {plane.ID} is degenerate or self-intersecting")
    if any(
        first.boundary.intersects(second.boundary)
        for index, first in enumerate(shapes)
        for second in shapes[index + 1 :]
    ):
        raise Structured3DError(f"floor plane {plane.ID} has overlapping or touching contours")
    depths = tuple(
        sum(container.contains(shape) for container in shapes if container is not shape)
        for shape in shapes
    )
    if any(depth > 1 for depth in depths) or sum(depth == 0 for depth in depths) != 1:
        raise Structured3DError(f"floor plane {plane.ID} contours are ambiguously nested")
    shell_index = depths.index(0)
    holes = tuple(ring for index, ring in enumerate(rings) if index != shell_index)
    polygon = Polygon2D(vertices=rings[shell_index], holes=holes)
    if not Polygon(
        tuple((point.x_m, point.y_m) for point in polygon.vertices),
        tuple(tuple((point.x_m, point.y_m) for point in hole) for hole in polygon.holes),
    ).is_valid:
        raise Structured3DError(f"floor plane {plane.ID} has invalid shell or holes")
    return polygon


def _validate_single_floor(heights: tuple[float, ...]) -> None:
    if max(heights) - min(heights) > _EPSILON_M:
        raise Structured3DError("unsupported multi-floor scene")


def _validated_internal_openings(
    scene_id: str,
    rooms: tuple[Room, ...],
    openings: tuple[PlaneGeometry, ...],
    policy: GeometryToleranceConfig,
) -> tuple[tuple[OpeningEdge, ...], tuple[Polygon2D, ...], tuple[tuple[Point2D, Point2D], ...]]:
    edges: list[OpeningEdge] = []
    polygons: list[Polygon2D] = []
    spans: list[tuple[Point2D, Point2D]] = []
    clearance = float(SPATIAL_BENCHMARK_V1.footprint.side_length_m) + 2.0 * float(
        SPATIAL_BENCHMARK_V1.footprint.safety_margin_m
    )
    for opening in openings:
        if opening.semantic_type != "door":
            continue
        ground_segments = _ground_segments(opening)
        if not ground_segments:
            continue  # exterior/high doors cannot create topology
        for first, second in ground_segments:
            touched = tuple(
                room
                for room in rooms
                if _overlap_boundary(first, second, room.boundary) > _EPSILON_M
            )
            if len(touched) != 2:
                continue
            shared = _shared_boundary_line(touched[0].boundary, touched[1].boundary)
            opening_line = LineString(((first.x_m, first.y_m), (second.x_m, second.y_m)))
            span = opening_line.intersection(shared)
            if span.length <= _EPSILON_M:
                continue
            coordinates = tuple(span.coords)
            if len(coordinates) != 2:
                continue
            span_start, span_end = (Point2D(x_m=x, y_m=y) for x, y in coordinates)
            width = span.length
            if width <= clearance:
                raise Structured3DError(
                    f"door plane {opening.source_plane_id} lacks effective footprint clearance"
                )
            if not _stable_opening_span(span_start, span_end, shared, clearance, policy):
                raise Structured3DError(
                    f"door plane {opening.source_plane_id} is tolerance-unstable"
                )
            left, right = sorted(touched, key=lambda room: room.room_id)
            edges.append(
                OpeningEdge(
                    opening_id=stable_opaque_id(
                        "opening",
                        {
                            "scene": scene_id,
                            "plane": opening.source_plane_id,
                            "segment": len(edges),
                        },
                    ),
                    first_room_id=left.room_id,
                    second_room_id=right.room_id,
                )
            )
            polygons.append(
                Polygon2D(
                    vertices=(
                        span_start,
                        span_end,
                        Point2D(x_m=span_end.x_m + _EPSILON_M, y_m=span_end.y_m + _EPSILON_M),
                    )
                )
            )
            spans.append((span_start, span_end))
        paired = _paired_door_opening(
            scene_id, opening, rooms, ground_segments, clearance, policy, len(edges)
        )
        if paired is None:
            continue
        edge, polygon, paired_spans = paired
        if any(
            {edge.first_room_id, edge.second_room_id}
            == {existing.first_room_id, existing.second_room_id}
            for existing in edges
        ):
            continue
        edges.append(edge)
        polygons.append(polygon)
        spans.extend(paired_spans)
    return tuple(edges), tuple(polygons), tuple(spans)


def _paired_door_opening(
    scene_id: str,
    opening: PlaneGeometry,
    rooms: tuple[Room, ...],
    ground_segments: tuple[tuple[Point2D, Point2D], ...],
    clearance: float,
    policy: GeometryToleranceConfig,
    edge_index: int,
) -> tuple[OpeningEdge, Polygon2D, tuple[tuple[Point2D, Point2D], ...]] | None:
    room_segments: list[tuple[Room, Point2D, Point2D, float]] = []
    for first, second in ground_segments:
        touched = tuple(
            (room, _overlap_boundary(first, second, room.boundary))
            for room in rooms
            if _overlap_boundary(first, second, room.boundary) > _EPSILON_M
        )
        if len(touched) == 1:
            room, overlap = touched[0]
            room_segments.append((room, first, second, overlap))
    for index, first_segment in enumerate(room_segments):
        for second_segment in room_segments[index + 1 :]:
            first_room, first_start, first_end, first_width = first_segment
            second_room, second_start, second_end, second_width = second_segment
            if first_room.room_id == second_room.room_id:
                continue
            width = min(first_width, second_width)
            if width + _EPSILON_M < clearance:
                continue
            polygon = _door_footprint_polygon(opening)
            if polygon is None:
                continue
            if not _stable_paired_opening(
                first_start, first_end, second_start, second_end, clearance, policy
            ):
                continue
            left, right = sorted((first_room, second_room), key=lambda room: room.room_id)
            edge = OpeningEdge(
                opening_id=stable_opaque_id(
                    "opening",
                    {"scene": scene_id, "plane": opening.source_plane_id, "segment": edge_index},
                ),
                first_room_id=left.room_id,
                second_room_id=right.room_id,
            )
            return edge, polygon, ((first_start, first_end), (second_start, second_end))
    return None


def _door_footprint_polygon(opening: PlaneGeometry) -> Polygon2D | None:
    for contour in opening.contours_m:
        if not contour or any(abs(point[2]) > _EPSILON_M for point in contour):
            continue
        vertices = tuple(Point2D(x_m=point[0], y_m=point[1]) for point in contour)
        polygon = Polygon2D(vertices=vertices)
        if abs(_area(polygon)) > _EPSILON_M and _simple(polygon):
            return polygon
    return None


def _coverage_connectors(
    floor_regions: tuple[Polygon2D, ...],
    rooms: tuple[Room, ...],
    opening_spans: tuple[tuple[Point2D, Point2D], ...],
    barriers: tuple[BarrierSegment, ...],
    policy: GeometryToleranceConfig,
) -> tuple[CoverageConnector, ...]:
    clearance = (
        float(SPATIAL_BENCHMARK_V1.footprint.side_length_m) / 2.0
        + float(SPATIAL_BENCHMARK_V1.footprint.safety_margin_m)
        + float(policy.collision_uncertainty_margin_m)
    )
    approach_offset = clearance + float(policy.collision_uncertainty_margin_m)
    approaches: list[tuple[str, Point2D, Point2D]] = []
    for start, end in opening_spans:
        midpoint = Point2D(x_m=(start.x_m + end.x_m) / 2.0, y_m=(start.y_m + end.y_m) / 2.0)
        for room in rooms:
            if _overlap_boundary(start, end, room.boundary) <= _EPSILON_M:
                continue
            centroid = Polygon(
                tuple((point.x_m, point.y_m) for point in room.boundary.vertices)
            ).representative_point()
            dx, dy = centroid.x - midpoint.x_m, centroid.y - midpoint.y_m
            length = hypot(dx, dy)
            if length <= _EPSILON_M:
                continue
            approach = Point2D(
                x_m=midpoint.x_m + approach_offset * dx / length,
                y_m=midpoint.y_m + approach_offset * dy / length,
            )
            if _contains(room.boundary, approach):
                approaches.append((room.room_id, midpoint, approach))
    connectors: list[CoverageConnector] = []
    for index, first in enumerate(approaches):
        for second in approaches[index + 1 :]:
            first_room, first_midpoint, first_approach = first
            second_room, second_midpoint, second_approach = second
            if first_room == second_room:
                continue
            if (
                _distance(first_midpoint, second_midpoint)
                > max(3.0 * clearance, float(policy.opening_uncertainty_margin_m)) + _EPSILON_M
            ):
                continue
            connectors.append(CoverageConnector(start=first_approach, end=second_approach))
    return tuple(connectors)


def _stable_paired_opening(
    first_start: Point2D,
    first_end: Point2D,
    second_start: Point2D,
    second_end: Point2D,
    clearance: float,
    policy: GeometryToleranceConfig,
) -> bool:
    margin = float(policy.opening_uncertainty_margin_m)
    return (
        _distance(first_start, first_end) + _EPSILON_M >= clearance
        and _distance(second_start, second_end) + _EPSILON_M >= clearance
        and LineString(
            ((first_start.x_m, first_start.y_m), (first_end.x_m, first_end.y_m))
        ).distance(
            LineString(((second_start.x_m, second_start.y_m), (second_end.x_m, second_end.y_m)))
        )
        <= max(margin, clearance) + _EPSILON_M
    )


def _ground_segments(opening: PlaneGeometry) -> tuple[tuple[Point2D, Point2D], ...]:
    segments: list[tuple[Point2D, Point2D]] = []
    for contour in opening.contours_m:
        for first, second in zip(contour, contour[1:] + contour[:1], strict=True):
            if (
                abs(first[2]) <= _EPSILON_M
                and abs(second[2]) <= _EPSILON_M
                and _distance(
                    Point2D(x_m=first[0], y_m=first[1]), Point2D(x_m=second[0], y_m=second[1])
                )
                > _EPSILON_M
            ):
                segments.append(
                    (Point2D(x_m=first[0], y_m=first[1]), Point2D(x_m=second[0], y_m=second[1]))
                )
    return tuple(segments)


def _barriers(
    walls: tuple[PlaneGeometry, ...],
    opening_spans: tuple[tuple[Point2D, Point2D], ...],
) -> tuple[BarrierSegment, ...]:
    # Only spans that survived internal-opening validation remove a wall barrier.
    barriers: list[BarrierSegment] = []
    for wall in walls:
        for contour in wall.contours_m:
            for first, second in zip(contour, contour[1:] + contour[:1], strict=True):
                if abs(first[2]) <= _EPSILON_M and abs(second[2]) <= _EPSILON_M:
                    segment = (
                        Point2D(x_m=first[0], y_m=first[1]),
                        Point2D(x_m=second[0], y_m=second[1]),
                    )
                    barriers.extend(_subtract_door_intervals(segment, opening_spans))
    return tuple(barriers)


def _contour_barriers(
    contours: tuple[tuple[tuple[float, float, float], ...], ...],
) -> tuple[BarrierSegment, ...]:
    return tuple(
        BarrierSegment(
            start=Point2D(x_m=first[0], y_m=first[1]), end=Point2D(x_m=second[0], y_m=second[1])
        )
        for contour in contours
        for first, second in zip(contour, contour[1:] + contour[:1], strict=True)
        if abs(first[2]) <= _EPSILON_M and abs(second[2]) <= _EPSILON_M and first[:2] != second[:2]
    )


def _hole_barriers(holes: tuple[tuple[Point2D, ...], ...]) -> tuple[BarrierSegment, ...]:
    return tuple(
        BarrierSegment(start=first, end=second)
        for hole in holes
        for first, second in zip(hole, hole[1:] + hole[:1], strict=True)
        if first != second
    )


def _subtract_door_intervals(
    wall: tuple[Point2D, Point2D], doors: tuple[tuple[Point2D, Point2D], ...]
) -> tuple[BarrierSegment, ...]:
    """Split a wall around collinear doorway spans rather than removing all of it."""
    start, end = wall
    length = _distance(start, end)
    if length <= _EPSILON_M:
        return ()
    direction_x, direction_y = (end.x_m - start.x_m) / length, (end.y_m - start.y_m) / length
    intervals: list[tuple[float, float]] = []
    for door_start, door_end in doors:
        if _collinear_overlap(start, end, door_start, door_end) <= _EPSILON_M:
            continue
        first = (door_start.x_m - start.x_m) * direction_x + (
            door_start.y_m - start.y_m
        ) * direction_y
        second = (door_end.x_m - start.x_m) * direction_x + (door_end.y_m - start.y_m) * direction_y
        intervals.append((max(0.0, min(first, second)), min(length, max(first, second))))
    intervals.sort()
    cursor = 0.0
    retained: list[BarrierSegment] = []
    for left, right in intervals:
        if left > cursor + _EPSILON_M:
            retained.append(_barrier_subsegment(start, direction_x, direction_y, cursor, left))
        cursor = max(cursor, right)
    if cursor < length - _EPSILON_M:
        retained.append(_barrier_subsegment(start, direction_x, direction_y, cursor, length))
    return tuple(retained)


def _barrier_subsegment(
    start: Point2D, direction_x: float, direction_y: float, left: float, right: float
) -> BarrierSegment:
    return BarrierSegment(
        start=Point2D(x_m=start.x_m + direction_x * left, y_m=start.y_m + direction_y * left),
        end=Point2D(x_m=start.x_m + direction_x * right, y_m=start.y_m + direction_y * right),
    )


def _validate_room_regions(rooms: tuple[Room, ...]) -> None:
    """Reject room polygons with any shared interior; boundary contact is valid."""

    shapes = tuple(_polygon_shape(room.boundary) for room in rooms)
    for index, first in enumerate(shapes):
        for second in shapes[index + 1 :]:
            if first.relate_pattern(second, "T********"):
                raise Structured3DError("room regions must not overlap or contain one another")


def _reject_open_plan_subdivisions(
    rooms: tuple[Room, ...],
    opening_spans: tuple[tuple[Point2D, Point2D], ...],
    barriers: tuple[BarrierSegment, ...],
    policy: GeometryToleranceConfig,
) -> None:
    """Require walls or validated openings to cover every shared-boundary component."""

    coverage = unary_union(
        tuple(
            LineString(((barrier.start.x_m, barrier.start.y_m), (barrier.end.x_m, barrier.end.y_m)))
            for barrier in barriers
        )
        + tuple(
            LineString(((start.x_m, start.y_m), (end.x_m, end.y_m))) for start, end in opening_spans
        )
    )
    tolerance = float(policy.opening_uncertainty_margin_m)
    for index, first in enumerate(rooms):
        for second in rooms[index + 1 :]:
            shared = _polygon_shape(first.boundary).boundary.intersection(
                _polygon_shape(second.boundary).boundary
            )
            for component in _line_components(shared):
                if component.length <= _EPSILON_M:
                    continue
                uncovered = component.difference(coverage.buffer(tolerance))
                if uncovered.length > _EPSILON_M:
                    raise Structured3DError(
                        "open-plan semantic subdivision has uncovered shared boundary"
                    )


def _validate_topology(topology: Topology) -> None:
    room_ids = {room.room_id for room in topology.rooms}
    if any(
        edge.first_room_id not in room_ids or edge.second_room_id not in room_ids
        for edge in topology.direct_openings
    ):
        raise Structured3DError("opening references an ineligible room")
    if len({edge.opening_id for edge in topology.direct_openings}) != len(topology.direct_openings):
        raise Structured3DError("duplicate opening ID")
    pairs = tuple(
        frozenset((edge.first_room_id, edge.second_room_id)) for edge in topology.direct_openings
    )
    if len(set(pairs)) != len(pairs):
        raise Structured3DError("multiple direct openings between the same room pair are ambiguous")
    adjacency: dict[str, set[str]] = {room_id: set() for room_id in room_ids}
    for edge in topology.direct_openings:
        adjacency[edge.first_room_id].add(edge.second_room_id)
        adjacency[edge.second_room_id].add(edge.first_room_id)
    if any(
        room_id not in adjacency[neighbor]
        for room_id, neighbors in adjacency.items()
        for neighbor in neighbors
    ):
        raise Structured3DError("room adjacency is not symmetric")
    degrees = {room_id: len(neighbors) for room_id, neighbors in adjacency.items()}
    if sum(degrees.values()) != 2 * len(topology.direct_openings):
        raise Structured3DError("room adjacency degrees are inconsistent")


def _shared_boundary_line(first: Polygon2D, second: Polygon2D) -> LineString:
    first_shape = _polygon_shape(first)
    second_shape = _polygon_shape(second)
    shared = first_shape.boundary.intersection(second_shape.boundary)
    if isinstance(shared, LineString):
        return shared
    if isinstance(shared, MultiLineString):
        return max(shared.geoms, key=lambda segment: segment.length)
    return LineString()


def _polygon_shape(polygon: Polygon2D) -> Polygon:
    return Polygon(
        tuple((point.x_m, point.y_m) for point in polygon.vertices),
        tuple(tuple((point.x_m, point.y_m) for point in hole) for hole in polygon.holes),
    )


def _line_components(geometry: BaseGeometry) -> tuple[LineString, ...]:
    if isinstance(geometry, LineString):
        return (geometry,)
    if isinstance(geometry, MultiLineString):
        return tuple(geometry.geoms)
    return ()


def _stable_opening_span(
    start: Point2D,
    end: Point2D,
    shared: LineString,
    clearance: float,
    policy: GeometryToleranceConfig,
) -> bool:
    margin = float(policy.opening_uncertainty_margin_m)
    span = LineString(((start.x_m, start.y_m), (end.x_m, end.y_m)))
    # Parent-wall contact is expected.  Perturb only the retained span and its
    # clearance, rejecting any aperture that can fall below usable width.
    return (
        span.length > clearance + 2.0 * margin + _EPSILON_M
        and shared.covers(span)
        and Point(start.x_m, start.y_m).distance(shared.boundary) >= margin - _EPSILON_M
        and Point(end.x_m, end.y_m).distance(shared.boundary) >= margin - _EPSILON_M
    )


def _area(polygon: Polygon2D) -> float:
    return (
        sum(
            first.x_m * second.y_m - second.x_m * first.y_m
            for first, second in zip(
                polygon.vertices, polygon.vertices[1:] + polygon.vertices[:1], strict=True
            )
        )
        / 2.0
    )


def _simple(polygon: Polygon2D) -> bool:
    edges = tuple(zip(polygon.vertices, polygon.vertices[1:] + polygon.vertices[:1], strict=True))
    return not any(
        _intersects(*first, *second)
        for index, first in enumerate(edges)
        for other, second in enumerate(edges[index + 1 :], index + 1)
        if other != index + 1 and not (index == 0 and other == len(edges) - 1)
    )


def _contains(polygon: Polygon2D, point: Point2D) -> bool:
    shape = Polygon(
        tuple((vertex.x_m, vertex.y_m) for vertex in polygon.vertices),
        tuple(tuple((vertex.x_m, vertex.y_m) for vertex in hole) for hole in polygon.holes),
    )
    return bool(shape.contains(Point(point.x_m, point.y_m)))


def _boundary_distance(point: Point2D, polygon: Polygon2D) -> float:
    shape = Polygon(
        tuple((vertex.x_m, vertex.y_m) for vertex in polygon.vertices),
        tuple(tuple((vertex.x_m, vertex.y_m) for vertex in hole) for hole in polygon.holes),
    )
    return float(shape.boundary.distance(Point(point.x_m, point.y_m)))


def _distance(first: Point2D, second: Point2D) -> float:
    return hypot(first.x_m - second.x_m, first.y_m - second.y_m)


def _point_segment_distance(point: Point2D, first: Point2D, second: Point2D) -> float:
    length = (second.x_m - first.x_m) ** 2 + (second.y_m - first.y_m) ** 2
    if length <= _EPSILON_M:
        return _distance(point, first)
    fraction = max(
        0.0,
        min(
            1.0,
            (
                (point.x_m - first.x_m) * (second.x_m - first.x_m)
                + (point.y_m - first.y_m) * (second.y_m - first.y_m)
            )
            / length,
        ),
    )
    return _distance(
        point,
        Point2D(
            x_m=first.x_m + fraction * (second.x_m - first.x_m),
            y_m=first.y_m + fraction * (second.y_m - first.y_m),
        ),
    )


def _overlap_boundary(first: Point2D, second: Point2D, polygon: Polygon2D) -> float:
    return max(
        (
            _collinear_overlap(first, second, left, right)
            for left, right in zip(
                polygon.vertices, polygon.vertices[1:] + polygon.vertices[:1], strict=True
            )
        ),
        default=0.0,
    )


def _shared_boundary(first: Polygon2D, second: Polygon2D) -> float:
    return max(
        (
            _collinear_overlap(a, b, c, d)
            for a, b in zip(first.vertices, first.vertices[1:] + first.vertices[:1], strict=True)
            for c, d in zip(second.vertices, second.vertices[1:] + second.vertices[:1], strict=True)
        ),
        default=0.0,
    )


def _collinear_overlap(a: Point2D, b: Point2D, c: Point2D, d: Point2D) -> float:
    if (
        abs((b.x_m - a.x_m) * (c.y_m - a.y_m) - (b.y_m - a.y_m) * (c.x_m - a.x_m)) > _EPSILON_M
        or abs((b.x_m - a.x_m) * (d.y_m - a.y_m) - (b.y_m - a.y_m) * (d.x_m - a.x_m)) > _EPSILON_M
    ):
        return 0.0
    axis = 0 if abs(b.x_m - a.x_m) >= abs(b.y_m - a.y_m) else 1
    values = (a.x_m, b.x_m, c.x_m, d.x_m) if axis == 0 else (a.y_m, b.y_m, c.y_m, d.y_m)
    span = max(
        0.0,
        min(max(values[0], values[1]), max(values[2], values[3]))
        - max(min(values[0], values[1]), min(values[2], values[3])),
    )
    return span


def _same_segment(first: tuple[Point2D, Point2D], second: tuple[Point2D, Point2D]) -> bool:
    return _collinear_overlap(*first, *second) >= _distance(*first) - _EPSILON_M


def _on_segment(point: Point2D, first: Point2D, second: Point2D) -> bool:
    return (
        abs(
            (second.x_m - first.x_m) * (point.y_m - first.y_m)
            - (second.y_m - first.y_m) * (point.x_m - first.x_m)
        )
        <= _EPSILON_M
        and min(first.x_m, second.x_m) - _EPSILON_M
        <= point.x_m
        <= max(first.x_m, second.x_m) + _EPSILON_M
        and min(first.y_m, second.y_m) - _EPSILON_M
        <= point.y_m
        <= max(first.y_m, second.y_m) + _EPSILON_M
    )


def _intersects(a: Point2D, b: Point2D, c: Point2D, d: Point2D) -> bool:
    def orientation(start: Point2D, end: Point2D, point: Point2D) -> float:
        return (end.x_m - start.x_m) * (point.y_m - start.y_m) - (end.y_m - start.y_m) * (
            point.x_m - start.x_m
        )

    one, two, three, four = (
        orientation(a, b, c),
        orientation(a, b, d),
        orientation(c, d, a),
        orientation(c, d, b),
    )
    return (
        (_on_segment(c, a, b) if abs(one) <= _EPSILON_M else False)
        or (_on_segment(d, a, b) if abs(two) <= _EPSILON_M else False)
        or (_on_segment(a, c, d) if abs(three) <= _EPSILON_M else False)
        or (_on_segment(b, c, d) if abs(four) <= _EPSILON_M else False)
        or ((one > 0) != (two > 0) and (three > 0) != (four > 0))
    )
