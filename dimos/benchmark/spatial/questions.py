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
"""Deterministic physical-question generation for spatial benchmark v1."""

from __future__ import annotations

from dataclasses import dataclass
import math
import random
from typing import TypeVar

from dimos.benchmark.spatial.collision_oracle import (
    CandidateDisposition,
    SquareFootprintCollisionOracle,
)
from dimos.benchmark.spatial.config import SPATIAL_BENCHMARK_V1, derive_v1_seed
from dimos.benchmark.spatial.models import (
    Answer,
    AnswerType,
    BooleanAnswerValue,
    DirectNeighborCountContract,
    DirectRoomConnectionContract,
    EligibleRoomCountContract,
    IntegerAnswerValue,
    MapVariant,
    Marker,
    Point2D,
    Pose2D,
    PoseOccupancyContract,
    Predicate,
    Question,
    QuestionContract,
    Room,
    RotationContract,
    SameRoomContract,
    Topology,
    TranslationContract,
)
from dimos.benchmark.spatial.structured3d import Structured3DError, unique_room_for_marker
from dimos.benchmark.spatial.utilities import JsonValue, stable_opaque_id

QUESTION_DEFINITION_VERSION = "spatial-predicates-v1"
TEMPLATE_VERSION = "spatial-question-templates-v1"

_TEXT = {
    Predicate.POSE_OCCUPANCY: "Can the robot occupy the marked pose?",
    Predicate.STRAIGHT_TRANSLATION: "Can the robot drive straight by the stated distance?",
    Predicate.IN_PLACE_ROTATION: "Can the robot rotate in place by the stated angle?",
    Predicate.ELIGIBLE_ROOM_COUNT: "How many eligible rooms are on this floor?",
    Predicate.SAME_ROOM: "Are the two markers in the same room?",
    Predicate.DIRECT_ROOM_CONNECTION: "Do the rooms containing the two markers share a direct opening?",
    Predicate.DIRECT_NEIGHBOR_COUNT: "How many rooms are directly connected to the marked room?",
}


@dataclass(frozen=True)
class PhysicalQuestion:
    """A private answer plus public-safe geometry before map-variant projection."""

    question: Question
    answer: Answer
    pose: Pose2D | None = None
    markers: tuple[Marker, ...] = ()


@dataclass(frozen=True)
class PredicateCandidateStats:
    predicate: Predicate
    retained: int
    rejected_uncertain: int
    positive: int = 0
    negative: int = 0
    pool_size: int = 0
    rejected_malformed: int = 0
    count_distribution: tuple[tuple[int, int], ...] = ()


@dataclass(frozen=True)
class _PoolStats:
    pool_size: int = 0
    retained: int = 0
    rejected_uncertain: int = 0
    rejected_malformed: int = 0
    positive: int = 0
    negative: int = 0
    counts: tuple[int, ...] = ()


_LAST_POOL_STATS: dict[tuple[str, ...], dict[Predicate, _PoolStats]] = {}
_T = TypeVar("_T")


def executable_definitions() -> dict[Predicate, str]:
    """Return the pinned executable-policy version for every predicate."""

    return {predicate: QUESTION_DEFINITION_VERSION for predicate in Predicate}


def question_identity_payload(
    *, scene_id: str, trajectory_id: str, predicate: Predicate, index: int, definition: str
) -> JsonValue:
    """Return the stable-ID payload for a public executable question identity."""

    return {
        "scene": scene_id,
        "trajectory": trajectory_id,
        "predicate": predicate.value,
        "index": index,
        "definition": definition,
    }


def public_question_index(question: Question) -> int:
    """Derive the public ordinal from question text/template metadata only."""

    if question.template_version != TEMPLATE_VERSION:
        raise ValueError("question template version does not match executable templates")
    if question.predicate is Predicate.ELIGIBLE_ROOM_COUNT:
        if question.text != _question_text(question.predicate, 0):
            raise ValueError("question text does not match executable template")
        return 0

    base = _TEXT[question.predicate]
    prefix = f"{base} (variant "
    if not question.text.startswith(prefix) or not question.text.endswith(")"):
        raise ValueError("question text does not match executable template")
    suffix = question.text.removeprefix(prefix)[:-1]
    try:
        variant = int(suffix)
    except ValueError as error:
        raise ValueError("question variant ordinal is not an integer") from error
    index = variant - 1
    if index < 0 or question.text != _question_text(question.predicate, index):
        raise ValueError("question text does not match executable template")
    return index


def generate_physical_questions(
    *, scene_id: str, trajectory_id: str, topology: Topology, oracle: SquareFootprintCollisionOracle
) -> tuple[PhysicalQuestion, ...]:
    """Generate up to two stable questions per predicate without guessed labels.

    Boolean predicates are emitted as one true and one false question where the
    authoritative scene supports both.  Counts use distinct stable query anchors.
    """

    if topology.scene_id != scene_id:
        raise ValueError("topology scene_id must match scene_id")
    stats: dict[Predicate, _PoolStats] = {}
    points = _interior_points(topology)
    if not points:
        raise ValueError("scene has no stable interior marker candidates")
    result: list[PhysicalQuestion] = []
    pose_pool = _motion_candidate_poses(topology, points, oracle)
    result.extend(
        _required(
            Predicate.POSE_OCCUPANCY,
            _boolean_motion_questions(
                scene_id,
                trajectory_id,
                Predicate.POSE_OCCUPANCY,
                oracle,
                pose_pool,
                stats,
            ),
        )
    )
    result.extend(
        _required(
            Predicate.STRAIGHT_TRANSLATION,
            _boolean_motion_questions(
                scene_id,
                trajectory_id,
                Predicate.STRAIGHT_TRANSLATION,
                oracle,
                pose_pool,
                stats,
            ),
        )
    )
    result.extend(
        _required(
            Predicate.IN_PLACE_ROTATION,
            _boolean_motion_questions(
                scene_id,
                trajectory_id,
                Predicate.IN_PLACE_ROTATION,
                oracle,
                pose_pool,
                stats,
            ),
        )
    )
    # eligible-room-count is scene-global: there is only one physical truth per
    # scene, so emit one question rather than duplicate the same physical query.
    if len(topology.rooms) < 1:
        raise ValueError("eligible-room-count requires at least one retained room")
    result.append(
        _count_question(
            scene_id,
            trajectory_id,
            Predicate.ELIGIBLE_ROOM_COUNT,
            0,
            len(topology.rooms),
            (),
        )
    )
    stats[Predicate.ELIGIBLE_ROOM_COUNT] = _PoolStats(
        pool_size=1, retained=1, counts=(len(topology.rooms),)
    )

    room_points = tuple(
        sorted(
            ((point, unique_room_for_marker(point, topology)) for point in points),
            key=lambda item: (item[1].room_id, item[0].x_m, item[0].y_m),
        )
    )
    same_pool = tuple(
        (a, b)
        for a, room_a in room_points
        for b, room_b in room_points
        if a != b and room_a.room_id == room_b.room_id
    )
    different_pool = tuple(
        (a, b)
        for a, room_a in room_points
        for b, room_b in room_points
        if room_a.room_id != room_b.room_id
    )
    stats[Predicate.SAME_ROOM] = _bool_stats(
        len(same_pool) + len(different_pool), len(same_pool), len(different_pool), 0
    )
    same = _pick(scene_id, Predicate.SAME_ROOM, "true", same_pool)
    different = _pick(scene_id, Predicate.SAME_ROOM, "false", different_pool)
    selected_same_room: list[tuple[tuple[Point2D, Point2D], bool]] = []
    for pair, value in ((same, True), (different, False)):
        if pair is None:
            raise ValueError(
                f"{Predicate.SAME_ROOM.value} lacks a {'positive' if value else 'negative'} candidate"
            )
        selected_same_room.append((pair, value))
    for index, (pair, value) in enumerate(
        _public_boolean_order(scene_id, Predicate.SAME_ROOM, tuple(selected_same_room))
    ):
        result.append(
            _marker_boolean_question(
                scene_id, trajectory_id, Predicate.SAME_ROOM, index, pair, value
            )
        )
    adjacency: dict[str, set[str]] = {room.room_id: set() for room in topology.rooms}
    for edge in sorted(
        topology.direct_openings,
        key=lambda edge: (
            min(edge.first_room_id, edge.second_room_id),
            max(edge.first_room_id, edge.second_room_id),
            edge.opening_id,
        ),
    ):
        adjacency[edge.first_room_id].add(edge.second_room_id)
        adjacency[edge.second_room_id].add(edge.first_room_id)
    direct_pool = tuple(
        (a, b)
        for a, ra in room_points
        for b, rb in room_points
        if rb.room_id in adjacency[ra.room_id]
    )
    indirect_pool = tuple(
        (a, b)
        for a, ra in room_points
        for b, rb in room_points
        if rb.room_id != ra.room_id and rb.room_id not in adjacency[ra.room_id]
    )
    stats[Predicate.DIRECT_ROOM_CONNECTION] = _bool_stats(
        len(direct_pool) + len(indirect_pool), len(direct_pool), len(indirect_pool), 0
    )
    direct = _pick(scene_id, Predicate.DIRECT_ROOM_CONNECTION, "true", direct_pool)
    indirect = _pick(scene_id, Predicate.DIRECT_ROOM_CONNECTION, "false", indirect_pool)
    selected_direct_connections: list[tuple[tuple[Point2D, Point2D], bool]] = []
    for pair, value in ((direct, True), (indirect, False)):
        if pair is None:
            raise ValueError(
                "direct-room-connection lacks a "
                + ("positive" if value else "negative")
                + " candidate"
            )
        selected_direct_connections.append((pair, value))
    for index, (pair, value) in enumerate(
        _public_boolean_order(
            scene_id, Predicate.DIRECT_ROOM_CONNECTION, tuple(selected_direct_connections)
        )
    ):
        result.append(
            _marker_boolean_question(
                scene_id, trajectory_id, Predicate.DIRECT_ROOM_CONNECTION, index, pair, value
            )
        )
    distinct_neighbor_rooms = tuple(
        {room.room_id: (point, room) for point, room in room_points}.values()
    )
    if len(distinct_neighbor_rooms) < 2:
        raise ValueError("direct-neighbor-count requires two distinct room anchors")
    ranked_rooms = _ranked(
        scene_id, Predicate.DIRECT_NEIGHBOR_COUNT, "anchors", distinct_neighbor_rooms
    )
    selected_rooms = _distinct_count_anchors(ranked_rooms, adjacency)
    stats[Predicate.DIRECT_NEIGHBOR_COUNT] = _PoolStats(
        pool_size=len(distinct_neighbor_rooms),
        retained=len(distinct_neighbor_rooms),
        counts=tuple(len(adjacency[room.room_id]) for _, room in distinct_neighbor_rooms),
    )
    for index, (point, room) in enumerate(selected_rooms):
        result.append(
            _count_question(
                scene_id,
                trajectory_id,
                Predicate.DIRECT_NEIGHBOR_COUNT,
                index,
                len(adjacency[room.room_id]),
                (point,),
            )
        )
    _validate_cardinality(tuple(result))
    _LAST_POOL_STATS[_question_key(tuple(result))] = stats
    return tuple(result)


def candidate_pool_stats(
    questions: tuple[PhysicalQuestion, ...],
) -> tuple[PredicateCandidateStats, ...]:
    stats: list[PredicateCandidateStats] = []
    for predicate in Predicate:
        selected = tuple(item for item in questions if item.question.predicate is predicate)
        positives = sum(
            1
            for item in selected
            if isinstance(item.answer.value, BooleanAnswerValue) and item.answer.value.value
        )
        negatives = sum(
            1
            for item in selected
            if isinstance(item.answer.value, BooleanAnswerValue) and not item.answer.value.value
        )
        pool_stats = _LAST_POOL_STATS.get(_question_key(questions), {}).get(
            predicate, _PoolStats(retained=len(selected), positive=positives, negative=negatives)
        )
        counts: dict[int, int] = {}
        for value in pool_stats.counts:
            counts[value] = counts.get(value, 0) + 1
        stats.append(
            PredicateCandidateStats(
                predicate,
                len(selected),
                pool_stats.rejected_uncertain,
                pool_stats.positive or positives,
                pool_stats.negative or negatives,
                pool_stats.pool_size,
                pool_stats.rejected_malformed,
                tuple(sorted(counts.items())),
            )
        )
    return tuple(stats)


def _question_key(questions: tuple[PhysicalQuestion, ...]) -> tuple[str, ...]:
    return tuple(sorted(item.question.question_id for item in questions))


def _bool_stats(
    pool_size: int, positive: int, negative: int, rejected_uncertain: int
) -> _PoolStats:
    return _PoolStats(
        pool_size=pool_size,
        retained=positive + negative,
        rejected_uncertain=rejected_uncertain,
        positive=positive,
        negative=negative,
    )


def _ranked(
    scene_id: str, predicate: Predicate, salt: str, candidates: tuple[_T, ...]
) -> tuple[_T, ...]:
    indexed = list(enumerate(candidates))
    seed = derive_v1_seed(
        MapVariant.CLEAN, scene_id, predicate.value, QUESTION_DEFINITION_VERSION, salt
    )
    random.Random(seed).shuffle(indexed)
    return tuple(item for _, item in indexed)


def _pick(scene_id: str, predicate: Predicate, salt: str, candidates: tuple[_T, ...]) -> _T | None:
    ranked = _ranked(scene_id, predicate, salt, candidates)
    return ranked[0] if ranked else None


def _public_boolean_order(
    scene_id: str,
    predicate: Predicate,
    selected: tuple[tuple[Pose2D | tuple[Point2D, Point2D], bool], ...],
) -> tuple[tuple[Pose2D | tuple[Point2D, Point2D], bool], ...]:
    """Order selected boolean questions without using their answer values."""

    seed = derive_v1_seed(
        MapVariant.CLEAN,
        scene_id,
        predicate.value,
        QUESTION_DEFINITION_VERSION,
        "public-boolean-order",
    )
    return tuple(
        item
        for _, item in sorted(
            (
                (
                    stable_opaque_id(
                        "question_order",
                        {
                            "seed": seed,
                            "predicate": predicate.value,
                            "public_candidate": _public_candidate_payload(candidate),
                        },
                    ),
                    (candidate, value),
                )
                for candidate, value in selected
            ),
            key=lambda keyed: keyed[0],
        )
    )


def _public_candidate_payload(candidate: Pose2D | tuple[Point2D, Point2D]) -> JsonValue:
    if isinstance(candidate, Pose2D):
        return {"pose": candidate.model_dump(mode="json")}
    return {"points": [point.model_dump(mode="json") for point in candidate]}


def _distinct_count_anchors(
    ranked_rooms: tuple[tuple[Point2D, Room], ...], adjacency: dict[str, set[str]]
) -> tuple[tuple[Point2D, Room], ...]:
    if len(ranked_rooms) < 2:
        return ranked_rooms
    first = ranked_rooms[0]
    first_count = len(adjacency[first[1].room_id])
    for candidate in ranked_rooms[1:]:
        if len(adjacency[candidate[1].room_id]) != first_count:
            return (first, candidate)
    return ranked_rooms[:2]


def _motion_candidate_poses(
    topology: Topology, points: tuple[Point2D, ...], oracle: SquareFootprintCollisionOracle
) -> tuple[Pose2D, ...]:
    poses: list[Pose2D] = []
    headings = (0.0, math.pi / 2.0, math.pi, -math.pi / 2.0)
    for point in points:
        for yaw in headings:
            poses.append(Pose2D(x_m=point.x_m, y_m=point.y_m, yaw_rad=yaw))
            poses.append(
                Pose2D(
                    x_m=point.x_m + 0.4 * math.cos(yaw),
                    y_m=point.y_m + 0.4 * math.sin(yaw),
                    yaw_rad=yaw,
                )
            )
    for room in sorted(topology.rooms, key=lambda room: room.room_id):
        vertices = tuple(sorted(room.boundary.vertices, key=lambda point: (point.x_m, point.y_m)))
        cx = sum(point.x_m for point in vertices) / len(vertices)
        cy = sum(point.y_m for point in vertices) / len(vertices)
        for vertex in vertices:
            poses.append(
                Pose2D(x_m=(vertex.x_m + cx) / 2.0, y_m=(vertex.y_m + cy) / 2.0, yaw_rad=0.0)
            )
            poses.append(
                Pose2D(
                    x_m=(vertex.x_m * 3.0 + cx) / 4.0,
                    y_m=(vertex.y_m * 3.0 + cy) / 4.0,
                    yaw_rad=math.pi / 4.0,
                )
            )
    poses.append(_outside_pose(topology))
    for barrier in oracle.barriers:
        mx = (barrier.start.x_m + barrier.end.x_m) / 2.0
        my = (barrier.start.y_m + barrier.end.y_m) / 2.0
        poses.append(Pose2D(x_m=mx, y_m=my, yaw_rad=0.0))
        poses.append(Pose2D(x_m=mx + 0.05, y_m=my + 0.05, yaw_rad=math.pi / 2.0))
    unique = {pose.model_dump_json(): pose for pose in poses}
    return tuple(unique[key] for key in sorted(unique))


def _required(
    predicate: Predicate, questions: tuple[PhysicalQuestion, ...]
) -> tuple[PhysicalQuestion, ...]:
    if len(questions) != 2:
        raise ValueError(f"{predicate.value} requires exactly two stable candidates")
    return questions


def _validate_cardinality(questions: tuple[PhysicalQuestion, ...]) -> None:
    missing = set(Predicate) - {item.question.predicate for item in questions}
    if missing:
        raise ValueError(
            "scene does not cover predicates: " + ", ".join(sorted(p.value for p in missing))
        )
    for predicate in Predicate:
        count = sum(1 for item in questions if item.question.predicate is predicate)
        expected = 1 if predicate is Predicate.ELIGIBLE_ROOM_COUNT else 2
        if count != expected:
            raise ValueError(
                f"{predicate.value} requires exactly {expected} physical candidates, got {count}"
            )


def _boolean_motion_questions(
    scene_id: str,
    trajectory_id: str,
    predicate: Predicate,
    oracle: SquareFootprintCollisionOracle,
    candidates: tuple[Pose2D, ...],
    stats: dict[Predicate, _PoolStats],
) -> tuple[PhysicalQuestion, ...]:
    accepted: list[tuple[Pose2D, bool]] = []
    rejected_uncertain = 0
    for pose in candidates:
        evaluation = (
            oracle.evaluate_pose(pose)
            if predicate is Predicate.POSE_OCCUPANCY
            else oracle.evaluate_translation(pose, 1.0)
            if predicate is Predicate.STRAIGHT_TRANSLATION
            else oracle.evaluate_rotation(pose, math.pi / 2.0)
        )
        if evaluation.disposition is CandidateDisposition.REJECTED_UNCERTAIN:
            rejected_uncertain += 1
            continue
        accepted.append((pose, evaluation.disposition is CandidateDisposition.CLEAR))
    positives = tuple(item for item in accepted if item[1])
    negatives = tuple(item for item in accepted if not item[1])
    stats[predicate] = _bool_stats(
        len(candidates), len(positives), len(negatives), rejected_uncertain
    )
    output: list[PhysicalQuestion] = []
    selected_candidates = tuple(
        selected_candidate
        for selected_candidate in (
            _pick(scene_id, predicate, "true", positives),
            _pick(scene_id, predicate, "false", negatives),
        )
        if selected_candidate is not None
    )
    for index, selected_candidate in enumerate(
        _public_boolean_order(scene_id, predicate, selected_candidates)
    ):
        pose, expected = selected_candidate
        contract = (
            PoseOccupancyContract(footprint_policy_version="square-footprint-v1")
            if predicate is Predicate.POSE_OCCUPANCY
            else TranslationContract(distance_m=1.0, footprint_policy_version="square-footprint-v1")
            if predicate is Predicate.STRAIGHT_TRANSLATION
            else RotationContract(
                yaw_delta_rad=math.pi / 2.0, footprint_policy_version="square-footprint-v1"
            )
        )
        output.append(
            _question(
                scene_id,
                trajectory_id,
                predicate,
                index,
                contract,
                BooleanAnswerValue(value=expected),
                pose=pose,
            )
        )
    return tuple(output)


def _marker_boolean_question(
    scene_id: str,
    trajectory_id: str,
    predicate: Predicate,
    index: int,
    points: tuple[Point2D, Point2D],
    value: bool,
) -> PhysicalQuestion:
    markers = _markers(scene_id, predicate, points)
    contract = (
        SameRoomContract(
            first_marker_id=markers[0].marker_id, second_marker_id=markers[1].marker_id
        )
        if predicate is Predicate.SAME_ROOM
        else DirectRoomConnectionContract(
            first_marker_id=markers[0].marker_id, second_marker_id=markers[1].marker_id
        )
    )
    return _question(
        scene_id,
        trajectory_id,
        predicate,
        index,
        contract,
        BooleanAnswerValue(value=value),
        markers=markers,
    )


def _count_question(
    scene_id: str,
    trajectory_id: str,
    predicate: Predicate,
    index: int,
    value: int,
    points: tuple[Point2D, ...],
) -> PhysicalQuestion:
    markers = _markers(scene_id, predicate, points) if points else ()
    contract: QuestionContract
    if predicate is Predicate.ELIGIBLE_ROOM_COUNT:
        contract = EligibleRoomCountContract()
    else:
        if not markers:
            raise ValueError("direct-neighbor-count requires a marker")
        contract = DirectNeighborCountContract(marker_id=markers[0].marker_id)
    return _question(
        scene_id,
        trajectory_id,
        predicate,
        index,
        contract,
        IntegerAnswerValue(value=value),
        markers=markers,
    )


def _question(
    scene_id: str,
    trajectory_id: str,
    predicate: Predicate,
    index: int,
    contract: QuestionContract,
    value: BooleanAnswerValue | IntegerAnswerValue,
    *,
    pose: Pose2D | None = None,
    markers: tuple[Marker, ...] = (),
) -> PhysicalQuestion:
    question_id = stable_opaque_id(
        "question",
        question_identity_payload(
            scene_id=scene_id,
            trajectory_id=trajectory_id,
            predicate=predicate,
            index=index,
            definition=QUESTION_DEFINITION_VERSION,
        ),
    )
    question = Question(
        question_id=question_id,
        scene_id=scene_id,
        trajectory_id=trajectory_id,
        predicate=predicate,
        template_version=TEMPLATE_VERSION,
        text=_question_text(predicate, index),
        answer_type=AnswerType.BOOLEAN
        if isinstance(value, BooleanAnswerValue)
        else AnswerType.INTEGER,
        contract=contract,
    )
    return PhysicalQuestion(
        question,
        Answer(
            question_id=question_id,
            predicate=predicate,
            value=value,
            oracle_policy_version=QUESTION_DEFINITION_VERSION,
        ),
        pose,
        markers,
    )


def _question_text(predicate: Predicate, index: int) -> str:
    base = _TEXT[predicate]
    if predicate is Predicate.ELIGIBLE_ROOM_COUNT:
        return base
    return f"{base} (variant {index + 1})"


def _markers(
    scene_id: str, predicate: Predicate, points: tuple[Point2D, ...]
) -> tuple[Marker, ...]:
    return tuple(
        Marker(
            marker_id=stable_opaque_id(
                "marker",
                {
                    "scene": scene_id,
                    "predicate": predicate.value,
                    "index": index,
                    "point": point.model_dump(mode="json"),
                },
            ),
            position=point,
        )
        for index, point in enumerate(points)
    )


def _interior_points(topology: Topology) -> tuple[Point2D, ...]:
    points: list[Point2D] = []
    for room in topology.rooms:
        xs, ys = (
            tuple(point.x_m for point in room.boundary.vertices),
            tuple(point.y_m for point in room.boundary.vertices),
        )
        candidate = Point2D(x_m=sum(xs) / len(xs), y_m=sum(ys) / len(ys))
        try:
            unique_room_for_marker(candidate, topology, SPATIAL_BENCHMARK_V1.geometry_tolerances)
        except Structured3DError:
            continue
        points.extend(
            (candidate, Point2D(x_m=(candidate.x_m * 3 + min(xs)) / 4, y_m=candidate.y_m))
        )
    return tuple(points)


def _outside_pose(topology: Topology) -> Pose2D:
    maximum = max(point.x_m for room in topology.rooms for point in room.boundary.vertices)
    maximum_y = max(point.y_m for room in topology.rooms for point in room.boundary.vertices)
    return Pose2D(x_m=maximum + 2.0, y_m=maximum_y + 2.0, yaw_rad=0.0)
