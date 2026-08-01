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

"""Hermetic release validation tests for mandatory spatial corpus invariants."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

from dimos.benchmark.spatial.bundles import BundleInput, SnapshotVariant, write_bundle
from dimos.benchmark.spatial.collision_oracle import (
    CandidateDisposition,
    SquareFootprintCollisionOracle,
)
from dimos.benchmark.spatial.config import GeometryToleranceConfig, SquareFootprintConfig
from dimos.benchmark.spatial.models import (
    Answer,
    BarrierSegment,
    BooleanAnswerValue,
    Geometry,
    IntegerAnswerValue,
    Point2D,
    Polygon2D,
    Pose2D,
    Predicate,
)
from dimos.benchmark.spatial.structured3d import SourceAxisTransform, load_structured3d_scene
from dimos.benchmark.spatial.test_bundles import _bundle
from dimos.benchmark.spatial.utilities import hash_file_sha256
from dimos.benchmark.spatial.validation import (
    ReleaseValidationError,
    require_release_complete,
    validate_release,
)
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


def _valid_bundle(tmp_path: Path) -> BundleInput:
    bundle = _bundle(tmp_path)
    variants: list[SnapshotVariant] = []
    for variant in bundle.variants:
        artifact = variant.directory / "global_map.pc2.lcm"
        cloud = PointCloud2.from_numpy(
            np.array(((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)), dtype=np.float32),
            frame_id=variant.snapshot.frame_id,
            timestamp=1.0,
        )
        artifact.write_bytes(cloud.lcm_encode(frame_id=variant.snapshot.frame_id))
        snapshot = variant.snapshot.model_copy(
            update={"map_artifact_sha256": hash_file_sha256(artifact)}
        )
        variants.append(SnapshotVariant(snapshot, variant.directory, variant.alignment))
    return BundleInput(
        bundle.manifest,
        bundle.scene,
        bundle.trajectory,
        bundle.source,
        bundle.geometry,
        bundle.topology,
        bundle.questions,
        tuple(variants),
    )


def _write_valid_release(tmp_path: Path) -> tuple[Path, BundleInput]:
    bundle = _valid_bundle(tmp_path)
    root = tmp_path / "corpus"
    write_bundle(root, bundle)
    # The tiny hermetic fixture can otherwise make marker ordering perfectly
    # predictive by accident. Keep the valid baseline nuisance-neutral.
    path = _oracle_trajectory_root(root, bundle) / "question_geometry.jsonl"
    records = [json.loads(line) for line in path.read_text().splitlines()]
    for record in records:
        if len(record["markers"]) == 2:
            record["markers"] = list(reversed(record["markers"]))
            break
    path.write_text(
        "".join(
            json.dumps(record, separators=(",", ":"), sort_keys=True) + "\n" for record in records
        )
    )
    return root, bundle


def _failure_checks(root: Path) -> set[str]:
    return {failure.check for failure in validate_release(root).failures}


def _trajectory_root(root: Path, bundle: BundleInput) -> Path:
    return (
        root
        / "public"
        / "scenes"
        / bundle.scene.scene_id
        / "trajectories"
        / bundle.trajectory.trajectory_id
    )


def _oracle_trajectory_root(root: Path, bundle: BundleInput) -> Path:
    return (
        root
        / "oracle"
        / "scenes"
        / bundle.scene.scene_id
        / "trajectories"
        / bundle.trajectory.trajectory_id
    )


def test_release_validation_accepts_complete_corpus(tmp_path: Path) -> None:
    root, _ = _write_valid_release(tmp_path)

    report = require_release_complete(root)

    assert report.complete
    assert report.failures == ()


def test_release_validation_reports_schema_refs_cardinality_hash_decode_and_separation(
    tmp_path: Path,
) -> None:
    root, bundle = _write_valid_release(tmp_path)
    questions_path = (
        root
        / "public"
        / "scenes"
        / bundle.scene.scene_id
        / "trajectories"
        / bundle.trajectory.trajectory_id
        / "questions.jsonl"
    )
    questions_path.write_text("\n".join(questions_path.read_text().splitlines()[:-1]) + "\n")
    clean_map = questions_path.parent / "variants" / "clean" / "global_map.pc2.lcm"
    clean_map.write_bytes(b"not a pointcloud")
    instances_path = questions_path.parent / "variants" / "clean" / "instances.jsonl"
    first_instance = json.loads(instances_path.read_text().splitlines()[0])
    first_instance["room_id"] = bundle.topology.rooms[0].room_id
    first_instance["snapshot_id"] = "snapshot_" + "f" * 64
    instances_path.write_text(json.dumps(first_instance) + "\n", encoding="utf-8")

    checks = _failure_checks(root)

    assert {
        "schema",
        "foreign-reference",
        "required-cardinality",
        "hash",
        "pointcloud2-decode",
        "public-leakage",
    } <= checks


def test_release_validation_reports_paired_variants_and_refuses_completion(
    tmp_path: Path,
) -> None:
    root, bundle = _write_valid_release(tmp_path)
    instances_path = (
        root
        / "public"
        / "scenes"
        / bundle.scene.scene_id
        / "trajectories"
        / bundle.trajectory.trajectory_id
        / "variants"
        / "noisy-02"
        / "instances.jsonl"
    )
    instances_path.write_text("", encoding="utf-8")

    report = validate_release(root)

    assert not report.complete
    assert "paired-variants" in {failure.check for failure in report.failures}
    with pytest.raises(ReleaseValidationError, match="release validation failed"):
        require_release_complete(root)


def test_release_validation_reports_missing_required_oracle_files(tmp_path: Path) -> None:
    root, bundle = _write_valid_release(tmp_path)
    (root / "oracle" / "scenes" / bundle.scene.scene_id / "geometry.json").unlink()
    (_oracle_trajectory_root(root, bundle) / "review_overrides.jsonl").unlink()

    failures = validate_release(root).failures

    assert any(
        failure.check == "schema" and "geometry.json" in failure.location for failure in failures
    )
    assert any(
        failure.check == "schema" and "review_overrides.jsonl" in failure.location
        for failure in failures
    )


def test_release_validation_reports_missing_oracle_markers(tmp_path: Path) -> None:
    root, bundle = _write_valid_release(tmp_path)
    path = _oracle_trajectory_root(root, bundle) / "question_geometry.jsonl"
    records = [json.loads(line) for line in path.read_text().splitlines()]
    marker_record = next(record for record in records if len(record["markers"]) == 2)
    marker_record["markers"] = marker_record["markers"][:1]
    path.write_text("".join(json.dumps(record, sort_keys=True) + "\n" for record in records))

    checks = _failure_checks(root)

    assert "foreign-reference" in checks
    assert "required-cardinality" in checks


def test_release_validation_rejects_direct_connection_markers_in_same_room(
    tmp_path: Path,
) -> None:
    root, bundle = _write_valid_release(tmp_path)
    question = next(
        item.question
        for item in bundle.questions
        if item.question.predicate.value == "direct-room-connection"
    )
    source = next(
        item
        for item in bundle.questions
        if item.question.predicate.value == "same-room" and len(item.markers) == 2
    )
    path = _oracle_trajectory_root(root, bundle) / "question_geometry.jsonl"
    records = [json.loads(line) for line in path.read_text().splitlines()]
    for record in records:
        if record["question_id"] == question.question_id:
            for index, marker in enumerate(record["markers"]):
                marker["position"] = source.markers[index].position.model_dump(mode="json")
    path.write_text("".join(json.dumps(record, sort_keys=True) + "\n" for record in records))

    assert "room-graph" in _failure_checks(root)


def test_release_validation_reports_unknown_instance_public_oracle_path_and_nan_yaw(
    tmp_path: Path,
) -> None:
    root, bundle = _write_valid_release(tmp_path)
    instances_path = _trajectory_root(root, bundle) / "variants" / "clean" / "instances.jsonl"
    records = [json.loads(line) for line in instances_path.read_text().splitlines()]
    records[0]["debug_path"] = "oracle/scenes/private/topology/answers.jsonl"
    records[1]["question_id"] = "question_" + "f" * 64
    for record in records:
        geometry = record["query_geometry"]
        if geometry["kind"] == "pose-occupancy":
            geometry["pose"]["yaw_rad"] = float("nan")
            break
    instances_path.write_text("".join(json.dumps(record) + "\n" for record in records))

    checks = _failure_checks(root)

    assert "foreign-reference" in checks
    assert "public-leakage" in checks
    assert "paired-variants" in checks


def test_release_validation_reports_balanced_label_and_nuisance_correlation(
    tmp_path: Path,
) -> None:
    root, bundle = _write_valid_release(tmp_path)
    answers_path = (
        root
        / "oracle"
        / "scenes"
        / bundle.scene.scene_id
        / "trajectories"
        / bundle.trajectory.trajectory_id
        / "answers.jsonl"
    )
    answers: list[Answer] = []
    for question in bundle.questions:
        answer = next(
            item.answer
            for item in bundle.questions
            if item.question.question_id == question.question.question_id
        )
        if question.question.predicate.value == "same-room":
            answer = answer.model_copy(update={"value": BooleanAnswerValue(value=True)})
        answers.append(answer)
    answers_path.write_text(
        "".join(answer.model_dump_json() + "\n" for answer in answers), encoding="utf-8"
    )

    checks = _failure_checks(root)

    assert "balanced-label" in checks
    assert "room-graph" in checks

    nuisance_root, nuisance_bundle = _write_valid_release(tmp_path / "nuisance")
    nuisance_answers_path = (
        _oracle_trajectory_root(nuisance_root, nuisance_bundle) / "answers.jsonl"
    )
    nuisance_answers: list[Answer] = []
    for question in nuisance_bundle.questions:
        answer = next(
            item.answer
            for item in nuisance_bundle.questions
            if item.question.question_id == question.question.question_id
        )
        if question.question.answer_type.value == "boolean":
            answer = answer.model_copy(
                update={
                    "value": BooleanAnswerValue(
                        value=question.question.text.endswith("(variant 2)")
                    )
                }
            )
        nuisance_answers.append(answer)
    nuisance_answers_path.write_text(
        "".join(answer.model_dump_json() + "\n" for answer in nuisance_answers),
        encoding="utf-8",
    )
    checks = _failure_checks(nuisance_root)
    assert "nuisance-correlation" in checks


def test_release_validation_reports_wrong_collision_label(tmp_path: Path) -> None:
    root, bundle = _write_valid_release(tmp_path)
    answers_path = _oracle_trajectory_root(root, bundle) / "answers.jsonl"
    answers: list[Answer] = []
    flipped = False
    for line in answers_path.read_text().splitlines():
        answer = Answer.model_validate_json(line)
        if not flipped and answer.predicate.value == "pose-occupancy":
            assert isinstance(answer.value, BooleanAnswerValue)
            answer = answer.model_copy(
                update={"value": BooleanAnswerValue(value=not answer.value.value)}
            )
            flipped = True
        answers.append(answer)
    answers_path.write_text("".join(answer.model_dump_json() + "\n" for answer in answers))

    assert "collision-oracle" in _failure_checks(root)


def test_release_validation_uses_barrier_model_for_collision_recompute(tmp_path: Path) -> None:
    root, bundle = _write_valid_release(tmp_path)
    target = next(
        item for item in bundle.questions if item.question.predicate.value == "pose-occupancy"
    )
    geometry_path = root / "oracle" / "scenes" / bundle.scene.scene_id / "geometry.json"
    geometry = Geometry.model_validate_json(geometry_path.read_bytes())
    assert target.pose is not None
    geometry = geometry.model_copy(
        update={
            "barrier_segments": (
                BarrierSegment(
                    start=Point2D(x_m=target.pose.x_m - 1.0, y_m=target.pose.y_m),
                    end=Point2D(x_m=target.pose.x_m + 1.0, y_m=target.pose.y_m),
                ),
            )
        }
    )
    geometry_path.write_text(geometry.model_dump_json(), encoding="utf-8")
    answers_path = _oracle_trajectory_root(root, bundle) / "answers.jsonl"
    answers: list[Answer] = []
    for line in answers_path.read_text().splitlines():
        answer = Answer.model_validate_json(line)
        if answer.question_id == target.question.question_id:
            answer = answer.model_copy(update={"value": BooleanAnswerValue(value=True)})
        answers.append(answer)
    answers_path.write_text("".join(answer.model_dump_json() + "\n" for answer in answers))

    assert "collision-oracle" in _failure_checks(root)


def test_structured3d_import_populates_geometry_barrier_segments(tmp_path: Path) -> None:
    annotation = {
        "junctions": [
            {"ID": 0, "coordinate": [0.0, 0.0, 0.0]},
            {"ID": 1, "coordinate": [4.0, 0.0, 0.0]},
            {"ID": 2, "coordinate": [4.0, 4.0, 0.0]},
            {"ID": 3, "coordinate": [0.0, 4.0, 0.0]},
            {"ID": 4, "coordinate": [4.0, 0.0, 2.0]},
            {"ID": 5, "coordinate": [0.0, 0.0, 2.0]},
        ],
        "lines": [
            {"ID": index, "point": [0.0, 0.0, 0.0], "direction": [1.0, 0.0, 0.0]}
            for index in range(8)
        ],
        "planes": [
            {"ID": 0, "type": "floor", "normal": [0.0, 0.0, 1.0], "offset": 0.0},
            {"ID": 1, "type": "wall", "normal": [0.0, -1.0, 0.0], "offset": 0.0},
        ],
        "semantics": [{"ID": 0, "type": "living room", "planeID": [0]}],
        "planeLineMatrix": [[1, 1, 1, 1, 0, 0, 0, 0], [0, 0, 0, 0, 1, 1, 1, 1]],
        "lineJunctionMatrix": [
            [1, 1, 0, 0, 0, 0],
            [0, 1, 1, 0, 0, 0],
            [0, 0, 1, 1, 0, 0],
            [1, 0, 0, 1, 0, 0],
            [1, 1, 0, 0, 0, 0],
            [0, 1, 0, 0, 1, 0],
            [0, 0, 0, 0, 1, 1],
            [1, 0, 0, 0, 0, 1],
        ],
    }
    path = tmp_path / "annotation.json"
    path.write_text(json.dumps(annotation), encoding="utf-8")

    imported = load_structured3d_scene(
        path,
        scene_id="scene_" + "a" * 64,
        source_scene_key="unit-test",
        source_revision="test",
        source_to_benchmark=SourceAxisTransform(scale_m_per_source_unit=1.0),
    )

    assert imported.free_space.barriers
    assert imported.geometry.barrier_segments == imported.free_space.barriers


def test_release_validation_reports_wrong_eligible_room_count(tmp_path: Path) -> None:
    root, bundle = _write_valid_release(tmp_path)
    path = _oracle_trajectory_root(root, bundle) / "answers.jsonl"
    answers: list[Answer] = []
    for line in path.read_text().splitlines():
        answer = Answer.model_validate_json(line)
        if answer.predicate.value == "eligible-room-count":
            answer = answer.model_copy(update={"value": IntegerAnswerValue(value=999)})
        answers.append(answer)
    path.write_text("".join(answer.model_dump_json() + "\n" for answer in answers))

    report = validate_release(root)

    assert "room-graph" in {failure.check for failure in report.failures}
    with pytest.raises(ReleaseValidationError, match="release validation failed"):
        require_release_complete(root)


def test_release_validation_rejects_direct_neighbor_boolean_answer(tmp_path: Path) -> None:
    root, bundle = _write_valid_release(tmp_path)
    path = _oracle_trajectory_root(root, bundle) / "answers.jsonl"
    answers: list[Answer] = []
    for line in path.read_text().splitlines():
        answer = Answer.model_validate_json(line)
        if answer.predicate is Predicate.DIRECT_NEIGHBOR_COUNT:
            answer = answer.model_copy(update={"value": BooleanAnswerValue(value=True)})
        answers.append(answer)
    path.write_text("".join(answer.model_dump_json() + "\n" for answer in answers))

    report = validate_release(root)
    checks = {failure.check for failure in report.failures}

    assert {"required-cardinality", "room-graph"} <= checks
    with pytest.raises(ReleaseValidationError, match="release validation failed"):
        require_release_complete(root)


def test_release_validation_rejects_answer_predicate_mismatch(tmp_path: Path) -> None:
    root, bundle = _write_valid_release(tmp_path)
    path = _oracle_trajectory_root(root, bundle) / "answers.jsonl"
    answers: list[Answer] = []
    changed = False
    for line in path.read_text().splitlines():
        answer = Answer.model_validate_json(line)
        if not changed and answer.predicate is Predicate.DIRECT_NEIGHBOR_COUNT:
            answer = answer.model_copy(update={"predicate": Predicate.ELIGIBLE_ROOM_COUNT})
            changed = True
        answers.append(answer)
    path.write_text("".join(answer.model_dump_json() + "\n" for answer in answers))

    report = validate_release(root)

    assert "foreign-reference" in {failure.check for failure in report.failures}
    with pytest.raises(ReleaseValidationError, match="release validation failed"):
        require_release_complete(root)


def test_release_validation_reports_determinism_mutation(tmp_path: Path) -> None:
    root, bundle = _write_valid_release(tmp_path)
    scene_path = root / "public" / "scenes" / bundle.scene.scene_id / "scene.json"
    scene = json.loads(scene_path.read_text())
    scene_path.write_text(json.dumps(scene, indent=4), encoding="utf-8")

    assert "determinism" in _failure_checks(root)


def test_release_validation_reports_marker_order_nuisance(tmp_path: Path) -> None:
    root, bundle = _write_valid_release(tmp_path)
    answers_path = _oracle_trajectory_root(root, bundle) / "answers.jsonl"
    answers: list[Answer] = []
    for item in bundle.questions:
        answer = item.answer
        if item.question.predicate.value in {"same-room", "direct-room-connection"}:
            assert len(item.markers) == 2
            answer = answer.model_copy(
                update={
                    "value": BooleanAnswerValue(
                        value=item.markers[0].marker_id < item.markers[1].marker_id
                    )
                }
            )
        answers.append(answer)
    answers_path.write_text("".join(answer.model_dump_json() + "\n" for answer in answers))
    geometry_path = _oracle_trajectory_root(root, bundle) / "question_geometry.jsonl"
    records = [json.loads(line) for line in geometry_path.read_text().splitlines()]
    answer_by_question = {answer.question_id: answer for answer in answers}
    for record in records:
        if len(record["markers"]) != 2:
            continue
        answer = answer_by_question[record["question_id"]]
        if not isinstance(answer.value, BooleanAnswerValue):
            continue
        ordered = sorted(record["markers"], key=lambda marker: marker["marker_id"])
        record["markers"] = ordered if answer.value.value else list(reversed(ordered))
    geometry_path.write_text(
        "".join(
            json.dumps(record, separators=(",", ":"), sort_keys=True) + "\n" for record in records
        )
    )

    assert "nuisance-correlation" in _failure_checks(root)


def test_release_validation_reports_point_count_extent_nuisance(tmp_path: Path) -> None:
    root, bundle = _write_valid_release(tmp_path)
    clean_root = _trajectory_root(root, bundle) / "variants" / "clean"
    clean_map = clean_root / "global_map.pc2.lcm"
    cloud = PointCloud2.from_numpy(
        np.array(((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (2.0, 0.0, 0.0)), dtype=np.float32),
        frame_id="world",
        timestamp=1.0,
    )
    clean_map.write_bytes(cloud.lcm_encode(frame_id="world"))
    snapshot_path = clean_root / "snapshot.json"
    snapshot = json.loads(snapshot_path.read_text())
    snapshot["map_artifact_sha256"] = hash_file_sha256(clean_map)
    snapshot_path.write_text(json.dumps(snapshot, separators=(",", ":"), sort_keys=True) + "\n")
    answers_path = _oracle_trajectory_root(root, bundle) / "answers.jsonl"
    answers: list[Answer] = []
    for line in answers_path.read_text().splitlines():
        answer = Answer.model_validate_json(line)
        if isinstance(answer.value, BooleanAnswerValue):
            answer = answer.model_copy(update={"value": BooleanAnswerValue(value=True)})
        answers.append(answer)
    answers_path.write_text("".join(answer.model_dump_json() + "\n" for answer in answers))

    assert "nuisance-correlation" in _failure_checks(root)


def test_room_graph_uses_private_geometry_not_noisy_public_marker_coordinates(
    tmp_path: Path,
) -> None:
    root, bundle = _write_valid_release(tmp_path)
    instances_path = _trajectory_root(root, bundle) / "variants" / "noisy-01" / "instances.jsonl"
    records = [json.loads(line) for line in instances_path.read_text().splitlines()]
    for record in records:
        geometry = record["query_geometry"]
        if geometry["kind"] == "markers":
            for marker in geometry["markers"]:
                marker["position"] = {"x_m": 10_000.0, "y_m": 10_000.0}
    instances_path.write_text("".join(json.dumps(record) + "\n" for record in records))

    assert "room-graph" not in _failure_checks(root)


def test_release_validation_reports_split_mismatch(tmp_path: Path) -> None:
    root, bundle = _write_valid_release(tmp_path)
    manifest_path = root / "manifest.json"
    manifest = json.loads(manifest_path.read_text())
    manifest["scenes"][0]["split"] = "held-out"
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")

    assert "scene-disjoint-split" in _failure_checks(root)


def test_collision_oracle_detects_colliding_intermediate_translation_with_clear_endpoints() -> None:
    def point(x_m: float, y_m: float) -> Point2D:
        return Point2D(x_m=x_m, y_m=y_m)

    def polygon(*coordinates: tuple[float, float]) -> Polygon2D:
        return Polygon2D(vertices=tuple(point(x, y) for x, y in coordinates))

    oracle = SquareFootprintCollisionOracle(
        (polygon((-2, -2), (2, -2), (2, 2), (-2, 2)),),
        (polygon((-0.05, -0.05), (0.05, -0.05), (0.05, 0.05), (-0.05, 0.05)),),
        SquareFootprintConfig(side_length_m=0.2, safety_margin_m=0.0),
        GeometryToleranceConfig(
            collision_uncertainty_margin_m=0.0,
            opening_uncertainty_margin_m=0.1,
            room_boundary_uncertainty_margin_m=0.1,
            translation_sweep_step_m=0.1,
            rotation_sweep_step_rad=0.1,
            rotation_refinement_limit=8,
        ),
    )
    start = Pose2D(x_m=-1.0, y_m=0.0, yaw_rad=0.0)
    end = Pose2D(x_m=1.0, y_m=0.0, yaw_rad=0.0)

    assert oracle.evaluate_pose(start).disposition is CandidateDisposition.CLEAR
    assert oracle.evaluate_pose(end).disposition is CandidateDisposition.CLEAR
    assert oracle.evaluate_translation(start, 2.0).disposition is CandidateDisposition.COLLISION
