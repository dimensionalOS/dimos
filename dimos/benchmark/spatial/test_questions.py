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
"""Hermetic tests for physical spatial question generation."""

from __future__ import annotations

from dimos.benchmark.spatial.collision_oracle import SquareFootprintCollisionOracle
from dimos.benchmark.spatial.config import SPATIAL_BENCHMARK_V1
from dimos.benchmark.spatial.models import (
    OpeningEdge,
    Point2D,
    Polygon2D,
    Predicate,
    Room,
    Topology,
)
from dimos.benchmark.spatial.questions import candidate_pool_stats, generate_physical_questions
from dimos.benchmark.spatial.utilities import stable_opaque_id


def _id(namespace: str) -> str:
    return stable_opaque_id(namespace, {"test": namespace})


def _square(left: float) -> Polygon2D:
    return Polygon2D(
        vertices=(
            Point2D(x_m=left, y_m=0.0),
            Point2D(x_m=left + 4.0, y_m=0.0),
            Point2D(x_m=left + 4.0, y_m=4.0),
            Point2D(x_m=left, y_m=4.0),
        )
    )


def _three_room_questions(scene_namespace: str):
    scene_id, trajectory_id = _id(scene_namespace), _id(f"{scene_namespace}-trajectory")
    rooms = tuple(
        Room(room_id=_id(f"{scene_namespace}-room-{index}"), boundary=_square(index * 4.0))
        for index in range(3)
    )
    topology = Topology(
        scene_id=scene_id,
        rooms=rooms,
        direct_openings=(
            OpeningEdge(
                opening_id=_id(f"{scene_namespace}-opening"),
                first_room_id=rooms[0].room_id,
                second_room_id=rooms[1].room_id,
            ),
        ),
    )
    oracle = SquareFootprintCollisionOracle(
        floor_regions=tuple(room.boundary for room in rooms),
        blocked_regions=(),
        footprint=SPATIAL_BENCHMARK_V1.footprint,
        tolerances=SPATIAL_BENCHMARK_V1.geometry_tolerances,
    )
    return generate_physical_questions(
        scene_id=scene_id, trajectory_id=trajectory_id, topology=topology, oracle=oracle
    )


def test_generation_is_deterministic_and_balances_supported_boolean_predicates() -> None:
    scene_id, trajectory_id = _id("scene"), _id("trajectory")
    rooms = tuple(
        Room(room_id=_id(f"room-{index}"), boundary=_square(index * 4.0)) for index in range(3)
    )
    topology = Topology(
        scene_id=scene_id,
        rooms=rooms,
        direct_openings=(
            OpeningEdge(
                opening_id=_id("opening"),
                first_room_id=rooms[0].room_id,
                second_room_id=rooms[1].room_id,
            ),
        ),
    )
    oracle = SquareFootprintCollisionOracle(
        floor_regions=tuple(room.boundary for room in rooms),
        blocked_regions=(),
        footprint=SPATIAL_BENCHMARK_V1.footprint,
        tolerances=SPATIAL_BENCHMARK_V1.geometry_tolerances,
    )
    generated = generate_physical_questions(
        scene_id=scene_id, trajectory_id=trajectory_id, topology=topology, oracle=oracle
    )
    repeated = generate_physical_questions(
        scene_id=scene_id, trajectory_id=trajectory_id, topology=topology, oracle=oracle
    )

    assert generated == repeated
    assert len(generated) == 13
    assert {item.question.predicate for item in generated} == set(Predicate)
    assert {stat.predicate: stat.retained for stat in candidate_pool_stats(generated)} == {
        predicate: 1 if predicate is Predicate.ELIGIBLE_ROOM_COUNT else 2 for predicate in Predicate
    }
    for predicate in (
        "pose-occupancy",
        "straight-translation",
        "in-place-rotation",
        "same-room",
        "direct-room-connection",
    ):
        values = [
            item.answer.value.value
            for item in generated
            if item.question.predicate.value == predicate
        ]
        assert sorted(values) == [False, True]
    assert len({item.question.question_id for item in generated}) == len(generated)
    assert all(item.answer.question_id == item.question.question_id for item in generated)


def test_candidate_pools_reject_uncertainty_and_keep_both_labels() -> None:
    scene_id, trajectory_id = _id("scene-pools"), _id("trajectory-pools")
    rooms = tuple(
        Room(room_id=_id(f"pool-room-{index}"), boundary=_square(index * 4.0)) for index in range(3)
    )
    topology = Topology(
        scene_id=scene_id,
        rooms=rooms,
        direct_openings=(
            OpeningEdge(
                opening_id=_id("pool-opening"),
                first_room_id=rooms[0].room_id,
                second_room_id=rooms[1].room_id,
            ),
        ),
    )
    oracle = SquareFootprintCollisionOracle(
        floor_regions=tuple(room.boundary for room in rooms),
        blocked_regions=(),
        footprint=SPATIAL_BENCHMARK_V1.footprint,
        tolerances=SPATIAL_BENCHMARK_V1.geometry_tolerances,
    )

    generated = generate_physical_questions(
        scene_id=scene_id, trajectory_id=trajectory_id, topology=topology, oracle=oracle
    )
    stats = {stat.predicate: stat for stat in candidate_pool_stats(generated)}

    assert stats[Predicate.POSE_OCCUPANCY].pool_size > 2
    assert stats[Predicate.POSE_OCCUPANCY].positive > 1
    assert stats[Predicate.POSE_OCCUPANCY].negative > 0
    assert stats[Predicate.POSE_OCCUPANCY].rejected_uncertain > 0


def test_neighbor_count_prefers_distinct_degree_values() -> None:
    scene_id, trajectory_id = _id("scene-neighbor"), _id("trajectory-neighbor")
    rooms = tuple(
        Room(room_id=_id(f"neighbor-room-{index}"), boundary=_square(index * 4.0))
        for index in range(3)
    )
    topology = Topology(
        scene_id=scene_id,
        rooms=rooms,
        direct_openings=(
            OpeningEdge(
                opening_id=_id("neighbor-opening-0"),
                first_room_id=rooms[0].room_id,
                second_room_id=rooms[1].room_id,
            ),
            OpeningEdge(
                opening_id=_id("neighbor-opening-1"),
                first_room_id=rooms[1].room_id,
                second_room_id=rooms[2].room_id,
            ),
        ),
    )
    oracle = SquareFootprintCollisionOracle(
        floor_regions=tuple(room.boundary for room in rooms),
        blocked_regions=(),
        footprint=SPATIAL_BENCHMARK_V1.footprint,
        tolerances=SPATIAL_BENCHMARK_V1.geometry_tolerances,
    )

    generated = generate_physical_questions(
        scene_id=scene_id, trajectory_id=trajectory_id, topology=topology, oracle=oracle
    )
    values = [
        item.answer.value.value
        for item in generated
        if item.question.predicate is Predicate.DIRECT_NEIGHBOR_COUNT
    ]

    assert sorted(values) == [1, 2]


def test_reordered_topology_and_geometry_inputs_are_identical() -> None:
    scene_id, trajectory_id = _id("scene-reorder"), _id("trajectory-reorder")
    rooms = tuple(
        Room(room_id=_id(f"reorder-room-{index}"), boundary=_square(index * 4.0))
        for index in range(3)
    )
    edges = (
        OpeningEdge(
            opening_id=_id("reorder-opening-0"),
            first_room_id=rooms[0].room_id,
            second_room_id=rooms[1].room_id,
        ),
        OpeningEdge(
            opening_id=_id("reorder-opening-1"),
            first_room_id=rooms[1].room_id,
            second_room_id=rooms[2].room_id,
        ),
    )
    topology = Topology(scene_id=scene_id, rooms=rooms, direct_openings=edges)
    reordered = Topology(
        scene_id=scene_id, rooms=tuple(reversed(rooms)), direct_openings=tuple(reversed(edges))
    )
    oracle = SquareFootprintCollisionOracle(
        floor_regions=tuple(room.boundary for room in rooms),
        blocked_regions=(),
        footprint=SPATIAL_BENCHMARK_V1.footprint,
        tolerances=SPATIAL_BENCHMARK_V1.geometry_tolerances,
    )
    reordered_oracle = SquareFootprintCollisionOracle(
        floor_regions=tuple(room.boundary for room in reversed(rooms)),
        blocked_regions=(),
        footprint=SPATIAL_BENCHMARK_V1.footprint,
        tolerances=SPATIAL_BENCHMARK_V1.geometry_tolerances,
    )

    generated = generate_physical_questions(
        scene_id=scene_id, trajectory_id=trajectory_id, topology=topology, oracle=oracle
    )
    regenerated = generate_physical_questions(
        scene_id=scene_id, trajectory_id=trajectory_id, topology=reordered, oracle=reordered_oracle
    )

    assert generated == regenerated


def test_boolean_public_ordinals_text_and_ids_do_not_have_fixed_label_encoding() -> None:
    labels_by_variant: dict[Predicate, dict[str, set[bool]]] = {
        predicate: {"variant 1": set(), "variant 2": set()}
        for predicate in (
            Predicate.POSE_OCCUPANCY,
            Predicate.STRAIGHT_TRANSLATION,
            Predicate.IN_PLACE_ROTATION,
            Predicate.SAME_ROOM,
            Predicate.DIRECT_ROOM_CONNECTION,
        )
    }
    labels_by_id_order: dict[Predicate, set[bool]] = {
        predicate: set() for predicate in labels_by_variant
    }

    for scene_index in range(10):
        generated = _three_room_questions(f"anti-encoding-scene-{scene_index}")
        assert len(generated) == 13
        for predicate in labels_by_variant:
            predicate_items = tuple(
                item for item in generated if item.question.predicate is predicate
            )
            assert len(predicate_items) == 2
            for item in predicate_items:
                label = item.answer.value.value
                assert isinstance(label, bool)
                if "variant 1" in item.question.text:
                    labels_by_variant[predicate]["variant 1"].add(label)
                elif "variant 2" in item.question.text:
                    labels_by_variant[predicate]["variant 2"].add(label)
                else:
                    raise AssertionError(f"missing neutral variant text: {item.question.text}")
            first_by_question_id = min(predicate_items, key=lambda item: item.question.question_id)
            first_label = first_by_question_id.answer.value.value
            assert isinstance(first_label, bool)
            labels_by_id_order[predicate].add(first_label)

    for predicate in labels_by_variant:
        assert labels_by_variant[predicate]["variant 1"] == {False, True}
        assert labels_by_variant[predicate]["variant 2"] == {False, True}
        assert labels_by_id_order[predicate] == {False, True}
