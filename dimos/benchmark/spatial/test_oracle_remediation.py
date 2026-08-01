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

"""Hermetic regressions for exact spatial oracle geometry."""

import math

import pytest
from shapely.geometry import LineString

from dimos.benchmark.spatial.collision_oracle import (
    CandidateDisposition,
    SquareFootprintCollisionOracle,
)
from dimos.benchmark.spatial.config import GeometryToleranceConfig, SquareFootprintConfig
from dimos.benchmark.spatial.models import (
    BarrierSegment,
    OpeningEdge,
    Point2D,
    Polygon2D,
    Pose2D,
    Room,
    Topology,
)
from dimos.benchmark.spatial.structured3d import (
    PlaneGeometry,
    SourceAxisTransform,
    Structured3DError,
    Structured3DPlane,
    _barriers,
    _floor_polygon,
    _reject_open_plan_subdivisions,
    _stable_opening_span,
    _validate_room_regions,
    _validate_topology,
    unique_room_for_marker,
)


def point(x, y):
    return Point2D(x_m=x, y_m=y)


def polygon(*coordinates):
    return Polygon2D(vertices=tuple(point(x, y) for x, y in coordinates))


def tolerance(margin=0.0):
    return GeometryToleranceConfig(
        collision_uncertainty_margin_m=margin,
        opening_uncertainty_margin_m=0.1,
        room_boundary_uncertainty_margin_m=0.1,
        translation_sweep_step_m=0.1,
        rotation_sweep_step_rad=math.pi / 8,
        rotation_refinement_limit=12,
    )


def room(name, boundary):
    return Room(room_id=f"room_{name * 64}", boundary=boundary)


def test_unordered_floor_contours_preserve_hole_and_collision():
    shell = ((0.0, 0.0, 0.0), (4.0, 0.0, 0.0), (4.0, 4.0, 0.0), (0.0, 4.0, 0.0))
    hole = ((1.0, 1.0, 0.0), (1.0, 3.0, 0.0), (3.0, 3.0, 0.0), (3.0, 1.0, 0.0))
    floor = _floor_polygon(
        Structured3DPlane(ID=0, type="floor", normal=(0.0, 0.0, 1.0), offset=0.0),
        (hole, shell),
        SourceAxisTransform(),
    )
    assert floor.vertices[0] == point(0.0, 0.0)
    assert len(floor.holes) == 1
    oracle = SquareFootprintCollisionOracle(
        (floor,), (), SquareFootprintConfig(side_length_m=0.2, safety_margin_m=0.0), tolerance()
    )
    assert oracle.evaluate_pose(Pose2D(x_m=2.0, y_m=2.0, yaw_rad=0.0)).is_collision


def test_exact_union_does_not_bridge_a_gap_for_a_footprint():
    oracle = SquareFootprintCollisionOracle(
        (polygon((0, 0), (1, 0), (1, 2), (0, 2)), polygon((1.01, 0), (2, 0), (2, 2), (1.01, 2))),
        (),
        SquareFootprintConfig(side_length_m=0.2, safety_margin_m=0.0),
        tolerance(),
    )
    assert oracle.evaluate_pose(Pose2D(x_m=1.005, y_m=1.0, yaw_rad=0.0)).is_collision


def test_invalid_or_exterior_door_cannot_remove_wall_barrier():
    wall = PlaneGeometry(
        source_plane_id=1,
        semantic_type="wall",
        contours_m=(((0.0, 0.0, 0.0), (3.0, 0.0, 0.0), (3.0, 0.0, 2.0), (0.0, 0.0, 2.0)),),
    )
    barriers = _barriers((wall,), ())
    assert len(barriers) == 1
    assert barriers[0].start == point(0.0, 0.0)
    assert barriers[0].end == point(3.0, 0.0)


def test_walled_shared_boundary_is_allowed_but_open_plan_is_rejected():
    left, right = (
        room("a", polygon((0, 0), (1, 0), (1, 1), (0, 1))),
        room("b", polygon((1, 0), (2, 0), (2, 1), (1, 1))),
    )
    wall = BarrierSegment(start=point(1, 0), end=point(1, 1))
    _reject_open_plan_subdivisions((left, right), (), (wall,), tolerance())
    with pytest.raises(Structured3DError, match="uncovered"):
        _reject_open_plan_subdivisions((left, right), (), (), tolerance())


def test_valid_doorway_cannot_skip_an_uncovered_shared_seam():
    left, right = (
        room("a", polygon((0, 0), (1, 0), (1, 3), (0, 3))),
        room("b", polygon((1, 0), (2, 0), (2, 3), (1, 3))),
    )
    wall = BarrierSegment(start=point(1, 0), end=point(1, 0.5))
    doorway = (point(1, 0.5), point(1, 1.5))
    with pytest.raises(Structured3DError, match="uncovered"):
        _reject_open_plan_subdivisions((left, right), (doorway,), (wall,), tolerance())


@pytest.mark.parametrize(
    "other",
    (
        polygon((1, 1), (3, 1), (3, 3), (1, 3)),
        polygon((0.5, 0.5), (1.5, 0.5), (1.5, 1.5), (0.5, 1.5)),
    ),
)
def test_overlapping_or_contained_room_interiors_are_rejected(other):
    with pytest.raises(Structured3DError, match="overlap or contain"):
        _validate_room_regions(
            (room("a", polygon((0, 0), (2, 0), (2, 2), (0, 2))), room("b", other))
        )


def test_multi_turn_rotation_detects_an_intermediate_collision():
    oracle = SquareFootprintCollisionOracle(
        (polygon((-2, -2), (2, -2), (2, 2), (-2, 2)),),
        (polygon((0.58, -0.02), (0.62, -0.02), (0.62, 0.02), (0.58, 0.02)),),
        SquareFootprintConfig(side_length_m=1.0, safety_margin_m=0.0),
        tolerance(),
    )
    result = oracle.evaluate_rotation(Pose2D(x_m=0.0, y_m=0.0, yaw_rad=0.0), 4 * math.pi)
    assert result.disposition is CandidateDisposition.COLLISION


def test_rotation_expanded_margin_rejects_an_unstable_clearance():
    oracle = SquareFootprintCollisionOracle(
        (polygon((-0.72, -0.72), (0.72, -0.72), (0.72, 0.72), (-0.72, 0.72)),),
        (),
        SquareFootprintConfig(side_length_m=1.0, safety_margin_m=0.0),
        tolerance(margin=0.05),
    )
    result = oracle.evaluate_rotation(Pose2D(x_m=0.0, y_m=0.0, yaw_rad=0.0), math.pi / 2)
    assert result.disposition is CandidateDisposition.REJECTED_UNCERTAIN


def test_marker_and_opening_uncertainty_bands_are_rejected():
    candidate_room = room("a", polygon((0, 0), (2, 0), (2, 2), (0, 2)))
    topology = Topology(scene_id="scene_" + "a" * 64, rooms=(candidate_room,), direct_openings=())
    with pytest.raises(Structured3DError, match="unambiguously"):
        unique_room_for_marker(point(0.05, 1.0), topology, tolerance())
    shared = LineString(((0, 0), (2, 0)))
    assert not _stable_opening_span(point(0.5, 0), point(1.3, 0), shared, 0.7, tolerance())


def test_topology_rejects_duplicate_pair_and_has_symmetric_degrees():
    left, right = (
        room("a", polygon((0, 0), (1, 0), (1, 1), (0, 1))),
        room("b", polygon((1, 0), (2, 0), (2, 1), (1, 1))),
    )
    edge = OpeningEdge(
        opening_id="opening_" + "a" * 64, first_room_id=left.room_id, second_room_id=right.room_id
    )
    _validate_topology(
        Topology(scene_id="scene_" + "a" * 64, rooms=(left, right), direct_openings=(edge,))
    )
    duplicate = edge.model_copy(update={"opening_id": "opening_" + "b" * 64})
    with pytest.raises(Structured3DError, match="multiple direct openings"):
        _validate_topology(
            Topology(
                scene_id="scene_" + "a" * 64, rooms=(left, right), direct_openings=(edge, duplicate)
            )
        )
