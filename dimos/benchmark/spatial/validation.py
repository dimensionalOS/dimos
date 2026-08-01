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
"""Release validation for hierarchical static spatial benchmark bundles."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import TypeVar

import numpy as np
from pydantic import BaseModel, ValidationError

from dimos.benchmark.spatial.collision_oracle import (
    CandidateDisposition,
    SquareFootprintCollisionOracle,
)
from dimos.benchmark.spatial.config import SPATIAL_BENCHMARK_V1
from dimos.benchmark.spatial.map_generation import load_snapshot_map
from dimos.benchmark.spatial.models import (
    RECORD_MODELS,
    Answer,
    BooleanAnswerValue,
    DirectNeighborCountContract,
    DirectRoomConnectionContract,
    Geometry,
    Instance,
    Manifest,
    MapVariant,
    Marker,
    MarkerGeometry,
    OracleQuestionGeometry,
    Predicate,
    Question,
    ReviewOverride,
    RotationContract,
    SameRoomContract,
    Scene,
    Snapshot,
    SourceProvenance,
    Split,
    Topology,
    Trajectory,
    TranslationContract,
)
from dimos.benchmark.spatial.structured3d import Structured3DError, unique_room_for_marker
from dimos.benchmark.spatial.utilities import JsonValue, canonical_json, hash_file_sha256


@dataclass(frozen=True)
class ValidationFailure:
    """One actionable failed invariant."""

    check: str
    location: str
    message: str


@dataclass(frozen=True)
class ValidationReport:
    """Complete mandatory validation result for a corpus root."""

    complete: bool
    failures: tuple[ValidationFailure, ...]

    def require_complete(self) -> None:
        """Refuse release completion when any mandatory invariant failed."""

        if self.failures:
            first = self.failures[0]
            raise ReleaseValidationError(
                f"release validation failed: {first.check} at {first.location}: {first.message}"
            )


class ReleaseValidationError(ValueError):
    """Raised when a corpus cannot be marked complete."""


_ModelT = TypeVar("_ModelT", bound=BaseModel)


@dataclass(frozen=True)
class _SceneRecords:
    scene: Scene
    source: SourceProvenance
    topology: Topology
    geometry: Geometry
    trajectory: Trajectory
    questions: tuple[Question, ...]
    answers: tuple[Answer, ...]
    question_geometries: tuple[OracleQuestionGeometry, ...]
    review_overrides: tuple[ReviewOverride, ...]
    snapshots: tuple[Snapshot, ...]
    instances: tuple[Instance, ...]


def validate_release(root: Path) -> ValidationReport:
    """Run every mandatory v1 release invariant and return actionable failures."""

    failures: list[ValidationFailure] = []
    manifest = _load_json_model(root / "manifest.json", Manifest, failures)
    if manifest is None:
        return ValidationReport(False, tuple(failures))
    _validate_deterministic_serialization(root, failures)
    _validate_package_separation(root, failures)
    scenes = tuple(_load_scene_records(root, item.scene_id, failures) for item in manifest.scenes)
    records = tuple(scene for scene in scenes if scene is not None)
    _validate_manifest_refs(root, manifest, records, failures)
    for record in records:
        _validate_scene_record(root, record, failures)
    _validate_corpus_distributions(root, records, failures)
    report = ValidationReport(not failures, tuple(failures))
    return report


def require_release_complete(root: Path) -> ValidationReport:
    """Validate a release and raise unless all mandatory invariants pass."""

    report = validate_release(root)
    report.require_complete()
    return report


def write_validation_report(root: Path, report: ValidationReport) -> Path:
    """Write the immutable release validation report next to the manifest."""

    manifest = _load_json_model(root / "manifest.json", Manifest, [])
    scene_count = len(manifest.scenes) if manifest is not None else 0
    instance_count = sum(
        1
        for path in (root / "public").glob("scenes/*/trajectories/*/variants/*/instances.jsonl")
        for line in path.read_text(encoding="utf-8").splitlines()
        if line
    )
    payload: JsonValue = {
        "complete": report.complete,
        "scene_count": scene_count,
        "expected_scene_count": 30,
        "instance_count": instance_count,
        "expected_instance_count_approx": 1170,
        "failures": [failure.__dict__ for failure in report.failures],
    }
    path = root / "release_validation_report.json"
    path.write_bytes(canonical_json(payload) + b"\n")
    return path


def _load_scene_records(
    root: Path, scene_id: str, failures: list[ValidationFailure]
) -> _SceneRecords | None:
    scene = _load_json_model(root / "public" / "scenes" / scene_id / "scene.json", Scene, failures)
    source = _load_json_model(
        root / "oracle" / "scenes" / scene_id / "source.json", SourceProvenance, failures
    )
    topology = _load_json_model(
        root / "oracle" / "scenes" / scene_id / "topology.json", Topology, failures
    )
    geometry = _load_json_model(
        root / "oracle" / "scenes" / scene_id / "geometry.json", Geometry, failures
    )
    if scene is not None and len(scene.trajectory_ids) == 1:
        _load_jsonl_models(
            root
            / "oracle"
            / "scenes"
            / scene_id
            / "trajectories"
            / scene.trajectory_ids[0]
            / "review_overrides.jsonl",
            ReviewOverride,
            failures,
        )
    if scene is None or source is None or topology is None or geometry is None:
        return None
    if len(scene.trajectory_ids) != 1:
        failures.append(
            ValidationFailure(
                "required-cardinality", scene.scene_id, "scene must have one trajectory"
            )
        )
        return None
    trajectory_id = scene.trajectory_ids[0]
    public_trajectory = root / "public" / "scenes" / scene_id / "trajectories" / trajectory_id
    oracle_trajectory = root / "oracle" / "scenes" / scene_id / "trajectories" / trajectory_id
    trajectory = _load_json_model(public_trajectory / "trajectory.json", Trajectory, failures)
    questions = _load_jsonl_models(public_trajectory / "questions.jsonl", Question, failures)
    answers = _load_jsonl_models(oracle_trajectory / "answers.jsonl", Answer, failures)
    question_geometries = _load_jsonl_models(
        oracle_trajectory / "question_geometry.jsonl", OracleQuestionGeometry, failures
    )
    review_overrides = _load_jsonl_models(
        oracle_trajectory / "review_overrides.jsonl", ReviewOverride, failures
    )
    snapshots = tuple(
        snapshot
        for variant in MapVariant
        if (
            snapshot := _load_json_model(
                public_trajectory / "variants" / variant.value / "snapshot.json", Snapshot, failures
            )
        )
        is not None
    )
    instances: list[Instance] = []
    for variant in MapVariant:
        instances.extend(
            _load_jsonl_models(
                public_trajectory / "variants" / variant.value / "instances.jsonl",
                Instance,
                failures,
            )
        )
    if trajectory is None:
        return None
    return _SceneRecords(
        scene,
        source,
        topology,
        geometry,
        trajectory,
        questions,
        answers,
        question_geometries,
        review_overrides,
        snapshots,
        tuple(instances),
    )


def _validate_manifest_refs(
    root: Path,
    manifest: Manifest,
    scenes: tuple[_SceneRecords, ...],
    failures: list[ValidationFailure],
) -> None:
    scene_ids = {record.scene.scene_id for record in scenes}
    if len(scene_ids) != len(manifest.scenes):
        failures.append(
            ValidationFailure("foreign-reference", "manifest.json", "duplicate or missing scene")
        )
    for item in manifest.scenes:
        expected = f"public/scenes/{item.scene_id}/scene.json"
        if item.scene_path != expected or not (root / item.scene_path).is_file():
            failures.append(ValidationFailure("foreign-reference", item.scene_id, "bad scene_path"))
        scene = next(
            (record.scene for record in scenes if record.scene.scene_id == item.scene_id), None
        )
        if scene is not None and scene.split != item.split:
            failures.append(
                ValidationFailure(
                    "scene-disjoint-split",
                    item.scene_id,
                    "manifest split does not match scene.json split",
                )
            )


def _validate_scene_record(
    root: Path, record: _SceneRecords, failures: list[ValidationFailure]
) -> None:
    scene_id = record.scene.scene_id
    trajectory_id = record.trajectory.trajectory_id
    if record.source.scene_id != scene_id or record.topology.scene_id != scene_id:
        failures.append(
            ValidationFailure("foreign-reference", scene_id, "oracle scene_id mismatch")
        )
    if record.geometry.scene_id != scene_id:
        failures.append(
            ValidationFailure("foreign-reference", scene_id, "geometry scene_id mismatch")
        )
    if record.trajectory.scene_id != scene_id:
        failures.append(
            ValidationFailure("foreign-reference", trajectory_id, "trajectory scene_id mismatch")
        )
    _validate_room_graph(record, failures)
    _validate_collision_answers(record, failures)
    _validate_questions_answers(record, failures)
    _validate_paired_variants(root, record, failures)
    _validate_snapshots(root, record, failures)


def _validate_room_graph(record: _SceneRecords, failures: list[ValidationFailure]) -> None:
    room_ids = {room.room_id for room in record.topology.rooms}
    adjacency: dict[str, set[str]] = {room_id: set() for room_id in room_ids}
    for edge in record.topology.direct_openings:
        if edge.first_room_id == edge.second_room_id:
            failures.append(
                ValidationFailure("room-graph", edge.opening_id, "opening is not distinct")
            )
        if edge.first_room_id not in room_ids or edge.second_room_id not in room_ids:
            failures.append(
                ValidationFailure("room-graph", edge.opening_id, "opening references unknown room")
            )
            continue
        adjacency[edge.first_room_id].add(edge.second_room_id)
        adjacency[edge.second_room_id].add(edge.first_room_id)
    if sum(len(neighbors) for neighbors in adjacency.values()) != 2 * len(
        record.topology.direct_openings
    ):
        failures.append(
            ValidationFailure("room-graph", record.scene.scene_id, "degree sum mismatch")
        )
    marker_points = (
        marker.position
        for oracle_geometry in record.question_geometries
        for marker in oracle_geometry.markers
    )
    for point in marker_points:
        try:
            unique_room_for_marker(point, record.topology)
        except Structured3DError as error:
            failures.append(ValidationFailure("room-graph", record.scene.scene_id, str(error)))
    answer_by_question = {answer.question_id: answer for answer in record.answers}
    for question in record.questions:
        answer = answer_by_question.get(question.question_id)
        if answer is None:
            continue
        if question.predicate is Predicate.ELIGIBLE_ROOM_COUNT:
            if answer.value.kind != "integer":
                failures.append(
                    ValidationFailure(
                        "room-graph",
                        question.question_id,
                        "eligible-room-count answer must be integer",
                    )
                )
            elif answer.value.value != len(record.topology.rooms):
                failures.append(
                    ValidationFailure(
                        "room-graph",
                        question.question_id,
                        f"eligible-room-count mismatch: expected {len(record.topology.rooms)}",
                    )
                )
        if isinstance(question.contract, DirectNeighborCountContract):
            marker = _oracle_marker_for_question(
                record, question.question_id, question.contract.marker_id
            )
            if answer.value.kind != "integer":
                failures.append(
                    ValidationFailure(
                        "room-graph",
                        question.question_id,
                        "direct-neighbor-count answer must be integer",
                    )
                )
            elif marker is not None:
                room = unique_room_for_marker(marker.position, record.topology)
                if answer.value.value != len(adjacency[room.room_id]):
                    failures.append(
                        ValidationFailure(
                            "room-graph", question.question_id, "neighbor count mismatch"
                        )
                    )
        if isinstance(question.contract, (SameRoomContract, DirectRoomConnectionContract)):
            first = _oracle_marker_for_question(
                record, question.question_id, question.contract.first_marker_id
            )
            second = _oracle_marker_for_question(
                record, question.question_id, question.contract.second_marker_id
            )
            if first is None or second is None or not isinstance(answer.value, BooleanAnswerValue):
                continue
            first_room = unique_room_for_marker(first.position, record.topology).room_id
            second_room = unique_room_for_marker(second.position, record.topology).room_id
            if (
                isinstance(question.contract, DirectRoomConnectionContract)
                and first_room == second_room
            ):
                failures.append(
                    ValidationFailure(
                        "room-graph",
                        question.question_id,
                        "direct-room-connection markers must resolve to distinct rooms",
                    )
                )
            expected = (
                first_room == second_room
                if isinstance(question.contract, SameRoomContract)
                else second_room in adjacency[first_room]
            )
            if answer.value.value != expected:
                failures.append(
                    ValidationFailure(
                        "room-graph", question.question_id, "topology answer mismatch"
                    )
                )


def _validate_collision_answers(record: _SceneRecords, failures: list[ValidationFailure]) -> None:
    oracle = SquareFootprintCollisionOracle(
        record.geometry.floor_regions,
        record.geometry.blocked_regions,
        SPATIAL_BENCHMARK_V1.footprint,
        SPATIAL_BENCHMARK_V1.geometry_tolerances,
        barriers=record.geometry.barrier_segments,
    )
    answer_by_question = {answer.question_id: answer for answer in record.answers}
    geometry_by_question = {item.question_id: item for item in record.question_geometries}
    for question in record.questions:
        if question.predicate not in {
            Predicate.POSE_OCCUPANCY,
            Predicate.STRAIGHT_TRANSLATION,
            Predicate.IN_PLACE_ROTATION,
        }:
            continue
        answer = answer_by_question.get(question.question_id)
        oracle_geometry = geometry_by_question.get(question.question_id)
        if answer is None or oracle_geometry is None or oracle_geometry.pose is None:
            failures.append(
                ValidationFailure(
                    "collision-oracle", question.question_id, "missing private pose geometry"
                )
            )
            continue
        if not isinstance(answer.value, BooleanAnswerValue):
            failures.append(
                ValidationFailure(
                    "collision-oracle", question.question_id, "collision answer must be boolean"
                )
            )
            continue
        if question.predicate is Predicate.POSE_OCCUPANCY:
            evaluation = oracle.evaluate_pose(oracle_geometry.pose)
        elif question.predicate is Predicate.STRAIGHT_TRANSLATION and isinstance(
            question.contract, TranslationContract
        ):
            evaluation = oracle.evaluate_translation(
                oracle_geometry.pose, question.contract.distance_m
            )
        elif isinstance(question.contract, RotationContract):
            evaluation = oracle.evaluate_rotation(
                oracle_geometry.pose, question.contract.yaw_delta_rad
            )
        else:
            failures.append(
                ValidationFailure(
                    "collision-oracle", question.question_id, "bad collision contract"
                )
            )
            continue
        if evaluation.disposition is CandidateDisposition.REJECTED_UNCERTAIN:
            failures.append(
                ValidationFailure("collision-oracle", question.question_id, evaluation.reason)
            )
            continue
        expected_clear = evaluation.disposition is CandidateDisposition.CLEAR
        if answer.value.value != expected_clear:
            failures.append(
                ValidationFailure(
                    "collision-oracle", question.question_id, "collision label mismatch"
                )
            )


def _validate_questions_answers(record: _SceneRecords, failures: list[ValidationFailure]) -> None:
    question_ids = [question.question_id for question in record.questions]
    answer_ids = [answer.question_id for answer in record.answers]
    if len(set(question_ids)) != len(question_ids) or len(set(answer_ids)) != len(answer_ids):
        failures.append(
            ValidationFailure(
                "required-cardinality", record.scene.scene_id, "duplicate question/answer IDs"
            )
        )
    if set(question_ids) != set(answer_ids):
        failures.append(
            ValidationFailure(
                "foreign-reference",
                record.scene.scene_id,
                "answers must match questions one-to-one",
            )
        )
    question_by_id = {question.question_id: question for question in record.questions}
    for answer in record.answers:
        question = question_by_id.get(answer.question_id)
        if question is None:
            failures.append(
                ValidationFailure(
                    "foreign-reference", answer.question_id, "answer references unknown question_id"
                )
            )
            continue
        if answer.predicate != question.predicate:
            failures.append(
                ValidationFailure(
                    "foreign-reference",
                    answer.question_id,
                    f"answer predicate {answer.predicate.value} does not match question predicate {question.predicate.value}",
                )
            )
        if answer.value.kind != question.answer_type.value:
            failures.append(
                ValidationFailure(
                    "required-cardinality",
                    answer.question_id,
                    f"answer value kind {answer.value.kind} does not match question answer_type {question.answer_type.value}",
                )
            )
    geometry_ids = [item.question_id for item in record.question_geometries]
    if len(set(geometry_ids)) != len(geometry_ids) or set(question_ids) != set(geometry_ids):
        failures.append(
            ValidationFailure(
                "foreign-reference",
                record.scene.scene_id,
                "oracle question geometry must match questions one-to-one",
            )
        )
    override_ids = [item.override_id for item in record.review_overrides]
    if len(set(override_ids)) != len(override_ids):
        failures.append(
            ValidationFailure(
                "required-cardinality", record.scene.scene_id, "duplicate review override IDs"
            )
        )
    for override in record.review_overrides:
        if override.question_id not in question_ids:
            failures.append(
                ValidationFailure(
                    "foreign-reference", override.override_id, "review override unknown question_id"
                )
            )
    geometry_by_question = {item.question_id: item for item in record.question_geometries}
    for question in record.questions:
        required_markers = _required_marker_ids(question)
        if not required_markers:
            continue
        oracle_geometry = geometry_by_question.get(question.question_id)
        if oracle_geometry is None:
            failures.append(
                ValidationFailure(
                    "foreign-reference", question.question_id, "missing oracle question geometry"
                )
            )
            continue
        present = {marker.marker_id for marker in oracle_geometry.markers}
        missing = required_markers - present
        if missing:
            failures.append(
                ValidationFailure(
                    "foreign-reference",
                    question.question_id,
                    "missing private marker IDs: " + ", ".join(sorted(missing)),
                )
            )
        if len(oracle_geometry.markers) != len(required_markers):
            failures.append(
                ValidationFailure(
                    "required-cardinality",
                    question.question_id,
                    f"expected {len(required_markers)} private markers, got {len(oracle_geometry.markers)}",
                )
            )
    for predicate in Predicate:
        expected = 1 if predicate is Predicate.ELIGIBLE_ROOM_COUNT else 2
        count = sum(question.predicate is predicate for question in record.questions)
        if count != expected:
            failures.append(
                ValidationFailure(
                    "required-cardinality",
                    record.scene.scene_id,
                    f"{predicate.value} count {count}, expected {expected}",
                )
            )


def _required_marker_ids(question: Question) -> set[str]:
    if isinstance(question.contract, (SameRoomContract, DirectRoomConnectionContract)):
        return {question.contract.first_marker_id, question.contract.second_marker_id}
    if isinstance(question.contract, DirectNeighborCountContract):
        return {question.contract.marker_id}
    return set()


def _validate_paired_variants(
    root: Path, record: _SceneRecords, failures: list[ValidationFailure]
) -> None:
    snapshots_by_variant = {snapshot.variant: snapshot for snapshot in record.snapshots}
    if set(snapshots_by_variant) != set(MapVariant) or len(record.snapshots) != 3:
        failures.append(
            ValidationFailure(
                "paired-variants", record.scene.scene_id, "expected exactly three snapshots"
            )
        )
    variants_dir = (
        root
        / "public"
        / "scenes"
        / record.scene.scene_id
        / "trajectories"
        / record.trajectory.trajectory_id
        / "variants"
    )
    if variants_dir.is_dir():
        extra = {path.name for path in variants_dir.iterdir() if path.is_dir()} - {
            variant.value for variant in MapVariant
        }
        for name in sorted(extra):
            failures.append(
                ValidationFailure(
                    "paired-variants", str(variants_dir / name), "unexpected variant directory"
                )
            )
    instances_by_question: dict[str, list[Instance]] = {
        question.question_id: [] for question in record.questions
    }
    seen_pairs: set[tuple[str, MapVariant]] = set()
    for instance in record.instances:
        if instance.question_id not in instances_by_question:
            failures.append(
                ValidationFailure("foreign-reference", instance.instance_id, "unknown question_id")
            )
        instances_by_question.setdefault(instance.question_id, []).append(instance)
        pair = (instance.question_id, instance.variant)
        if pair in seen_pairs:
            failures.append(
                ValidationFailure(
                    "paired-variants", instance.instance_id, "duplicate question/variant instance"
                )
            )
        seen_pairs.add(pair)
        if (
            instance.scene_id != record.scene.scene_id
            or instance.trajectory_id != record.trajectory.trajectory_id
        ):
            failures.append(
                ValidationFailure(
                    "foreign-reference", instance.instance_id, "instance parent mismatch"
                )
            )
        snapshot = snapshots_by_variant.get(instance.variant)
        if snapshot is None or instance.snapshot_id != snapshot.snapshot_id:
            failures.append(
                ValidationFailure(
                    "foreign-reference", instance.instance_id, "instance snapshot mismatch"
                )
            )
        if not _has_complete_coordinates(instance):
            failures.append(
                ValidationFailure(
                    "paired-variants", instance.instance_id, "missing variant coordinates"
                )
            )
    for question_id, instances in instances_by_question.items():
        if len(instances) != 3 or {item.variant for item in instances} != set(MapVariant):
            failures.append(
                ValidationFailure(
                    "paired-variants", question_id, "expected one clean and two noisy instances"
                )
            )


def _validate_snapshots(
    root: Path, record: _SceneRecords, failures: list[ValidationFailure]
) -> None:
    base = (
        root
        / "public"
        / "scenes"
        / record.scene.scene_id
        / "trajectories"
        / record.trajectory.trajectory_id
    )
    for snapshot in record.snapshots:
        variant_root = base / "variants" / snapshot.variant.value
        artifact = variant_root / snapshot.map_artifact_path
        if not artifact.is_file() or hash_file_sha256(artifact) != snapshot.map_artifact_sha256:
            failures.append(ValidationFailure("hash", str(artifact), "map artifact hash mismatch"))
        try:
            load_snapshot_map(variant_root, snapshot)
        except (OSError, ValueError) as error:
            failures.append(ValidationFailure("pointcloud2-decode", str(artifact), str(error)))


def _validate_corpus_distributions(
    root: Path, records: tuple[_SceneRecords, ...], failures: list[ValidationFailure]
) -> None:
    split_members: dict[Split, set[str]] = {Split.DEVELOPMENT: set(), Split.HELD_OUT: set()}
    labels_by_predicate: dict[Predicate, set[bool]] = {predicate: set() for predicate in Predicate}
    for record in records:
        split_members[record.scene.split].add(record.scene.scene_id)
        answer_by_question = {answer.question_id: answer for answer in record.answers}
        for question in record.questions:
            answer = answer_by_question.get(question.question_id)
            if answer is not None and isinstance(answer.value, BooleanAnswerValue):
                labels_by_predicate[question.predicate].add(answer.value.value)
    if split_members[Split.DEVELOPMENT] & split_members[Split.HELD_OUT]:
        failures.append(
            ValidationFailure(
                "scene-disjoint-split", "manifest.json", "scene appears in multiple splits"
            )
        )
    boolean_predicates = tuple(
        predicate
        for predicate in Predicate
        if predicate not in {Predicate.ELIGIBLE_ROOM_COUNT, Predicate.DIRECT_NEIGHBOR_COUNT}
    )
    for predicate in boolean_predicates:
        if labels_by_predicate[predicate] not in ({True, False}, set()):
            failures.append(
                ValidationFailure(
                    "balanced-label", predicate.value, "boolean predicate lacks both labels"
                )
            )
    _validate_nuisance_correlation(root, records, failures)


def _validate_nuisance_correlation(
    root: Path, records: tuple[_SceneRecords, ...], failures: list[ValidationFailure]
) -> None:
    for predicate in Predicate:
        texts: dict[str, set[bool]] = {}
        for record in records:
            answer_by_question = {answer.question_id: answer for answer in record.answers}
            for question in record.questions:
                answer = answer_by_question.get(question.question_id)
                if (
                    question.predicate is predicate
                    and answer is not None
                    and isinstance(answer.value, BooleanAnswerValue)
                ):
                    texts.setdefault(question.text, set()).add(answer.value.value)
        # Require enough repeated observations before declaring a template-only
        # nuisance correlation; tiny smoke fixtures intentionally contain one
        # positive and one negative text variant per predicate.
        if (
            sum(len(values) for values in texts.values()) >= 4
            and all(len(values) == 1 for values in texts.values())
            and len({next(iter(values)) for values in texts.values()}) > 1
        ):
            failures.append(
                ValidationFailure(
                    "nuisance-correlation", predicate.value, "template text predicts label"
                )
            )
    labels_by_ordinal: dict[str, set[bool]] = {}
    for record in records:
        answer_by_question = {answer.question_id: answer for answer in record.answers}
        for question in record.questions:
            answer = answer_by_question.get(question.question_id)
            if answer is None or not isinstance(answer.value, BooleanAnswerValue):
                continue
            ordinal = (
                question.text.rsplit("(variant ", 1)[-1].removesuffix(")")
                if "(variant " in question.text
                else "none"
            )
            labels_by_ordinal.setdefault(ordinal, set()).add(answer.value.value)
    if len(labels_by_ordinal) >= 2 and all(
        len(values) == 1 for values in labels_by_ordinal.values()
    ):
        failures.append(
            ValidationFailure(
                "nuisance-correlation",
                "template-variant-order",
                "template variant/order predicts label",
            )
        )
    _validate_marker_order_correlation(records, failures)
    _validate_map_stat_correlation(root, records, failures)


def _validate_marker_order_correlation(
    records: tuple[_SceneRecords, ...], failures: list[ValidationFailure]
) -> None:
    buckets: dict[str, set[bool]] = {}
    for record in records:
        answers = {answer.question_id: answer for answer in record.answers}
        geometries = {geometry.question_id: geometry for geometry in record.question_geometries}
        for question in record.questions:
            if not isinstance(question.contract, (SameRoomContract, DirectRoomConnectionContract)):
                continue
            answer = answers.get(question.question_id)
            if answer is None or not isinstance(answer.value, BooleanAnswerValue):
                continue
            geometry = geometries.get(question.question_id)
            if geometry is None or len(geometry.markers) != 2:
                continue
            bucket = (
                "first-before-second"
                if geometry.markers[0].marker_id < geometry.markers[1].marker_id
                else "second-before-first"
            )
            buckets.setdefault(bucket, set()).add(answer.value.value)
    if len(buckets) >= 2 and all(len(values) == 1 for values in buckets.values()):
        failures.append(
            ValidationFailure(
                "nuisance-correlation", "marker-order", "marker ordering predicts label"
            )
        )


def _validate_map_stat_correlation(
    root: Path, records: tuple[_SceneRecords, ...], failures: list[ValidationFailure]
) -> None:
    release_buckets: dict[str, set[bool]] = {}
    for record in records:
        answers = {answer.question_id: answer for answer in record.answers}
        snapshot_stats = _snapshot_stats(root, record)
        buckets: dict[str, set[bool]] = {}
        for instance in record.instances:
            answer = answers.get(instance.question_id)
            stats = snapshot_stats.get(instance.snapshot_id)
            if answer is None or stats is None or not isinstance(answer.value, BooleanAnswerValue):
                continue
            buckets.setdefault(stats, set()).add(answer.value.value)
            release_buckets.setdefault(stats, set()).add(answer.value.value)
        if len(buckets) >= 2 and all(len(values) == 1 for values in buckets.values()):
            failures.append(
                ValidationFailure(
                    "nuisance-correlation",
                    record.scene.scene_id,
                    "map point-count/extent bucket predicts label",
                )
            )
    if len(release_buckets) >= 2 and all(len(values) == 1 for values in release_buckets.values()):
        failures.append(
            ValidationFailure(
                "nuisance-correlation",
                "release-map-stats",
                "release-level map point-count/extent bucket predicts label",
            )
        )


def _snapshot_stats(root: Path, record: _SceneRecords) -> dict[str, str]:
    stats: dict[str, str] = {}
    base = (
        root
        / "public"
        / "scenes"
        / record.scene.scene_id
        / "trajectories"
        / record.trajectory.trajectory_id
    )
    for snapshot in record.snapshots:
        try:
            points, _ = load_snapshot_map(
                base / "variants" / snapshot.variant.value, snapshot
            ).as_numpy()
        except (OSError, ValueError):
            continue
        if len(points):
            mins = np.min(points[:, :3], axis=0)
            maxes = np.max(points[:, :3], axis=0)
            extent = maxes - mins
            stats[snapshot.snapshot_id] = (
                f"count={len(points)};extent={tuple(round(float(v), 3) for v in extent)}"
            )
        else:
            stats[snapshot.snapshot_id] = "count=0;extent=(0,0,0)"
    return stats


def _validate_package_separation(root: Path, failures: list[ValidationFailure]) -> None:
    public_root = root / "public"
    if not public_root.is_dir() or not (root / "oracle").is_dir():
        failures.append(
            ValidationFailure(
                "package-separation", str(root), "public and oracle roots are required"
            )
        )
    forbidden_keys = {
        "room_id",
        "source_scene_key",
        "source_artifact_sha256",
        "oracle_policy_version",
    }
    forbidden_values = {"source-provenance", "geometry", "topology", "answers", "oracle", "private"}
    for path in public_root.rglob("*") if public_root.is_dir() else ():
        if path.is_file() and path.suffix in {".json", ".jsonl"}:
            for record in _iter_json_records(path, failures):
                _scan_public_record(record, forbidden_keys, forbidden_values, path, failures)


def _validate_deterministic_serialization(root: Path, failures: list[ValidationFailure]) -> None:
    record_suffixes = {".json", ".jsonl"}
    for path in sorted(root.rglob("*")):
        if not path.is_file() or "schemas" in path.relative_to(root).parts:
            continue
        if path.suffix == ".json":
            records = tuple(_json_value(record) for record in _iter_json_records(path, failures))
            if records and path.read_bytes() != canonical_json(records[0]) + b"\n":
                failures.append(
                    ValidationFailure(
                        "determinism",
                        str(path),
                        "JSON is not canonical deterministic serialization",
                    )
                )
        elif path.suffix == ".jsonl":
            try:
                expected = b"".join(
                    canonical_json(_json_value(record)) + b"\n"
                    for record in _iter_json_records(path, failures)
                )
            except ValueError as error:
                failures.append(ValidationFailure("determinism", str(path), str(error)))
                continue
            if path.read_bytes() != expected:
                failures.append(
                    ValidationFailure(
                        "determinism",
                        str(path),
                        "JSONL is not canonical deterministic serialization",
                    )
                )
        elif path.suffix in record_suffixes:
            continue
    schemas = root / "schemas"
    for model in RECORD_MODELS:
        path = schemas / f"{model.__name__.lower()}.schema.json"
        try:
            content = path.read_text(encoding="utf-8")
        except OSError as error:
            failures.append(ValidationFailure("determinism", str(path), str(error)))
            continue
        expected_text = (
            json.dumps(model.model_json_schema(), ensure_ascii=False, indent=2, sort_keys=True)
            + "\n"
        )
        if content != expected_text:
            failures.append(
                ValidationFailure(
                    "determinism", str(path), "schema file is not deterministic current output"
                )
            )


def _json_value(value: object) -> JsonValue:
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    if isinstance(value, list):
        return [_json_value(item) for item in value]
    if isinstance(value, dict):
        return {str(key): _json_value(child) for key, child in value.items()}
    raise ValueError(f"non-JSON value {value!r}")


def _scan_public_record(
    value: object,
    forbidden_keys: set[str],
    forbidden_values: set[str],
    path: Path,
    failures: list[ValidationFailure],
) -> None:
    if isinstance(value, dict):
        for key, child in value.items():
            if key in forbidden_keys:
                failures.append(
                    ValidationFailure("public-leakage", str(path), f"private field {key}")
                )
            if key != "query_geometry":
                _scan_token(key, forbidden_values, path, failures)
            _scan_public_record(child, forbidden_keys, forbidden_values, path, failures)
    elif isinstance(value, list):
        for child in value:
            _scan_public_record(child, forbidden_keys, forbidden_values, path, failures)
    elif isinstance(value, str):
        _scan_token(value, forbidden_values, path, failures)


def _scan_token(
    value: str, forbidden_values: set[str], path: Path, failures: list[ValidationFailure]
) -> None:
    lowered = value.casefold()
    tokens = {token for token in lowered.replace("/", "-").replace("_", "-").split("-") if token}
    if lowered == "query_geometry":
        return
    for forbidden in forbidden_values:
        if forbidden in tokens or f"/{forbidden}/" in f"/{lowered}/" or forbidden in lowered:
            failures.append(
                ValidationFailure(
                    "public-leakage", str(path), f"private token {forbidden!r} in {value!r}"
                )
            )
            return


def _marker_for_question(
    instances: tuple[Instance, ...], question_id: str, marker_id: str
) -> Marker | None:
    for instance in instances:
        if instance.question_id == question_id and isinstance(
            instance.query_geometry, MarkerGeometry
        ):
            for marker in instance.query_geometry.markers:
                if marker.marker_id == marker_id:
                    return marker
    return None


def _oracle_marker_for_question(
    record: _SceneRecords, question_id: str, marker_id: str
) -> Marker | None:
    for item in record.question_geometries:
        if item.question_id == question_id:
            for marker in item.markers:
                if marker.marker_id == marker_id:
                    return marker
    return None


def _has_complete_coordinates(instance: Instance) -> bool:
    geometry = instance.query_geometry
    if isinstance(geometry, MarkerGeometry):
        return all(
            _finite_point(marker.position.x_m, marker.position.y_m) for marker in geometry.markers
        )
    if geometry.kind == "pose-occupancy":
        return _finite_pose(geometry.pose.x_m, geometry.pose.y_m, geometry.pose.yaw_rad)
    if geometry.kind == "straight-translation":
        return _finite_pose(
            geometry.start_pose.x_m, geometry.start_pose.y_m, geometry.start_pose.yaw_rad
        )
    if geometry.kind == "in-place-rotation":
        return _finite_pose(geometry.pose.x_m, geometry.pose.y_m, geometry.pose.yaw_rad)
    return True


def _finite_point(x_m: float, y_m: float) -> bool:
    return math.isfinite(x_m) and math.isfinite(y_m)


def _finite_pose(x_m: float, y_m: float, yaw_rad: float) -> bool:
    return math.isfinite(x_m) and math.isfinite(y_m) and math.isfinite(yaw_rad)


def _load_json_model(
    path: Path, model: type[_ModelT], failures: list[ValidationFailure]
) -> _ModelT | None:
    try:
        return model.model_validate_json(path.read_bytes())
    except (OSError, ValidationError, ValueError) as error:
        failures.append(ValidationFailure("schema", str(path), str(error)))
        return None


def _load_jsonl_models(
    path: Path, model: type[_ModelT], failures: list[ValidationFailure]
) -> tuple[_ModelT, ...]:
    records: list[_ModelT] = []
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError as error:
        failures.append(ValidationFailure("schema", str(path), str(error)))
        return ()
    for index, line in enumerate(lines, 1):
        if not line:
            continue
        try:
            records.append(model.model_validate_json(line))
        except (ValidationError, ValueError) as error:
            failures.append(ValidationFailure("schema", f"{path}:{index}", str(error)))
    return tuple(records)


def _iter_json_records(path: Path, failures: list[ValidationFailure]) -> tuple[object, ...]:
    try:
        if path.suffix == ".jsonl":
            return tuple(
                json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line
            )
        return (json.loads(path.read_text(encoding="utf-8")),)
    except (OSError, json.JSONDecodeError) as error:
        failures.append(ValidationFailure("schema", str(path), str(error)))
        return ()
