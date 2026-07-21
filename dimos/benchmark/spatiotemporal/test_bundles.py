# Copyright 2025-2026 Dimensional Inc.
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

"""Tests for deterministic public/private evaluation bundles."""

from hashlib import sha256
from pathlib import Path

import pytest

from dimos.benchmark.spatiotemporal.bundles import load_bundle, write_bundle
from dimos.benchmark.spatiotemporal.models import (
    BoundingBox2D,
    ObjectObservation,
    OracleAnswer,
    OracleBundleManifest,
    PublicBundleManifest,
    Question,
    QuestionKind,
    RelationFact,
    RelationInterval,
    SpatialPredicate,
    TemporalPredicate,
)
from dimos.benchmark.spatiotemporal.utilities import (
    SCHEMA_VERSION,
    JsonValue,
    canonical_model_json,
    stable_id,
    stable_question_id,
)


def _spatial_records() -> tuple[
    Question,
    ObjectObservation,
    RelationFact,
    OracleAnswer,
]:
    episode_id = "episode_1"
    subject_id = "obj_red"
    object_id = "obj_blue"
    predicate = SpatialPredicate.LEFT_OF
    relation_id = stable_id(
        "relation",
        {
            "object_ids": (subject_id, object_id),
            "predicate": predicate.value,
            "schema_version": SCHEMA_VERSION,
        },
    )
    question_id = stable_question_id(
        episode_id=episode_id,
        object_ids=(subject_id, object_id),
        predicate=predicate.value,
        question_kind=QuestionKind.SPATIAL.value,
    )
    question = Question(
        question_id=question_id,
        episode_id=episode_id,
        text="Is obj_red left of obj_blue?",
        question_kind=QuestionKind.SPATIAL,
        predicate=predicate,
        object_ids=(subject_id, object_id),
    )
    observation = ObjectObservation(
        episode_id=episode_id,
        frame_id=3,
        timestamp_s=0.3,
        object_id=subject_id,
        label="red object",
        box=BoundingBox2D(x_min=0.1, y_min=0.2, x_max=0.3, y_max=0.4),
        confidence=0.9,
    )
    fact = RelationFact(
        relation_id=relation_id,
        episode_id=episode_id,
        frame_id=3,
        timestamp_s=0.3,
        subject_id=subject_id,
        predicate=predicate,
        object_id=object_id,
        evidence_frame_ids=(3,),
    )
    answer = OracleAnswer(
        question_id=question_id,
        expected=True,
        evidence_frame_ids=(3,),
    )
    return question, observation, fact, answer


def _interval_for_fact(fact: RelationFact) -> RelationInterval:
    contract: dict[str, JsonValue] = {
        "end_frame_id": fact.frame_id,
        "end_timestamp_s": fact.timestamp_s,
        "episode_id": fact.episode_id,
        "object_id": fact.object_id,
        "predicate": fact.predicate.value,
        "relation_id": fact.relation_id,
        "schema_version": SCHEMA_VERSION,
        "start_frame_id": fact.frame_id,
        "start_timestamp_s": fact.timestamp_s,
        "subject_id": fact.subject_id,
    }
    return RelationInterval(
        interval_id=stable_id("interval", contract),
        relation_id=fact.relation_id,
        episode_id=fact.episode_id,
        subject_id=fact.subject_id,
        predicate=fact.predicate,
        object_id=fact.object_id,
        start_frame_id=fact.frame_id,
        end_frame_id=fact.frame_id,
        start_timestamp_s=fact.timestamp_s,
        end_timestamp_s=fact.timestamp_s,
        evidence_frame_ids=(fact.frame_id,),
    )


def test_writes_and_loads_public_questions_without_private_teacher_data(tmp_path: Path) -> None:
    question, observation, fact, answer = _spatial_records()

    write_bundle(
        tmp_path,
        episode_id="episode_1",
        source_video_sha256="1" * 64,
        questions=(question,),
        observations=(observation,),
        relation_facts=(fact,),
        relation_intervals=(),
        answers=(answer,),
    )
    loaded = load_bundle(tmp_path)

    assert loaded.questions == (question,)
    assert loaded.observations == (observation,)
    assert loaded.relation_facts == (fact,)
    assert loaded.relation_intervals == ()
    assert loaded.answers == (answer,)
    public_text = "\n".join(
        path.read_text(encoding="utf-8") for path in sorted((tmp_path / "public").iterdir())
    )
    assert "expected" not in public_text
    assert "evidence_frame_ids" not in public_text
    assert "confidence" not in public_text
    assert (tmp_path / "oracle" / "answers.jsonl").is_file()


def test_bundle_output_is_byte_identical_across_roots(tmp_path: Path) -> None:
    question, observation, fact, answer = _spatial_records()
    roots = (tmp_path / "first", tmp_path / "second")
    for root in roots:
        write_bundle(
            root,
            episode_id="episode_1",
            source_video_sha256="1" * 64,
            questions=(question,),
            observations=(observation,),
            relation_facts=(fact,),
            relation_intervals=(),
            answers=(answer,),
        )

    outputs = [
        {path.relative_to(root): path.read_bytes() for path in root.rglob("*") if path.is_file()}
        for root in roots
    ]
    assert outputs[0] == outputs[1]


def test_rejects_duplicate_public_question_references(tmp_path: Path) -> None:
    question, observation, fact, answer = _spatial_records()

    with pytest.raises(ValueError, match="duplicate question ID"):
        write_bundle(
            tmp_path,
            episode_id="episode_1",
            source_video_sha256="1" * 64,
            questions=(question, question),
            observations=(observation,),
            relation_facts=(fact,),
            relation_intervals=(),
            answers=(answer,),
        )


def test_rejects_oracle_answers_for_foreign_questions(tmp_path: Path) -> None:
    question, observation, fact, _ = _spatial_records()
    foreign_answer = OracleAnswer(
        question_id=f"question_{'0' * 64}",
        expected=True,
        evidence_frame_ids=(3,),
    )

    with pytest.raises(ValueError, match="foreign question ID"):
        write_bundle(
            tmp_path,
            episode_id="episode_1",
            source_video_sha256="1" * 64,
            questions=(question,),
            observations=(observation,),
            relation_facts=(fact,),
            relation_intervals=(),
            answers=(foreign_answer,),
        )


def test_rejects_missing_oracle_answer_references(tmp_path: Path) -> None:
    question, observation, fact, _ = _spatial_records()

    with pytest.raises(ValueError, match="missing oracle answer"):
        write_bundle(
            tmp_path,
            episode_id="episode_1",
            source_video_sha256="1" * 64,
            questions=(question,),
            observations=(observation,),
            relation_facts=(fact,),
            relation_intervals=(),
            answers=(),
        )


def test_rejects_duplicate_oracle_answer_references(tmp_path: Path) -> None:
    question, observation, fact, answer = _spatial_records()

    with pytest.raises(ValueError, match="duplicate oracle answer"):
        write_bundle(
            tmp_path,
            episode_id="episode_1",
            source_video_sha256="1" * 64,
            questions=(question,),
            observations=(observation,),
            relation_facts=(fact,),
            relation_intervals=(),
            answers=(answer, answer),
        )


def test_rejects_temporal_questions_with_foreign_relation_references(tmp_path: Path) -> None:
    _, observation, fact, _ = _spatial_records()
    references = (fact.relation_id, f"relation_{'0' * 64}")
    question_id = stable_question_id(
        episode_id="episode_1",
        predicate=TemporalPredicate.BEFORE.value,
        question_kind=QuestionKind.TEMPORAL.value,
        reference_ids=references,
    )
    question = Question(
        question_id=question_id,
        episode_id="episode_1",
        text="Did the first relation happen before the second?",
        question_kind=QuestionKind.TEMPORAL,
        predicate=TemporalPredicate.BEFORE,
        object_ids=(),
        reference_ids=references,
    )
    answer = OracleAnswer(question_id=question_id, expected=True, evidence_frame_ids=(3,))

    with pytest.raises(ValueError, match="foreign relation reference"):
        write_bundle(
            tmp_path,
            episode_id="episode_1",
            source_video_sha256="1" * 64,
            questions=(question,),
            observations=(observation,),
            relation_facts=(fact,),
            relation_intervals=(_interval_for_fact(fact),),
            answers=(answer,),
        )


def test_rejects_answers_with_foreign_interval_evidence(tmp_path: Path) -> None:
    question, observation, fact, _ = _spatial_records()
    answer = OracleAnswer(
        question_id=question.question_id,
        expected=True,
        evidence_interval_ids=(f"interval_{'0' * 64}",),
    )

    with pytest.raises(ValueError, match="foreign interval reference"):
        write_bundle(
            tmp_path,
            episode_id="episode_1",
            source_video_sha256="1" * 64,
            questions=(question,),
            observations=(observation,),
            relation_facts=(fact,),
            relation_intervals=(_interval_for_fact(fact),),
            answers=(answer,),
        )


def test_rejects_records_from_a_foreign_episode(tmp_path: Path) -> None:
    question, observation, fact, answer = _spatial_records()
    foreign_question = question.model_copy(update={"episode_id": "episode_2"})

    with pytest.raises(ValueError, match="foreign episode"):
        write_bundle(
            tmp_path,
            episode_id="episode_1",
            source_video_sha256="1" * 64,
            questions=(foreign_question,),
            observations=(observation,),
            relation_facts=(fact,),
            relation_intervals=(),
            answers=(answer,),
        )


def test_loader_rejects_artifacts_that_do_not_match_the_manifest(tmp_path: Path) -> None:
    question, observation, fact, answer = _spatial_records()
    write_bundle(
        tmp_path,
        episode_id="episode_1",
        source_video_sha256="1" * 64,
        questions=(question,),
        observations=(observation,),
        relation_facts=(fact,),
        relation_intervals=(),
        answers=(answer,),
    )
    questions_path = tmp_path / "public" / "questions.jsonl"
    questions_path.write_bytes(questions_path.read_bytes() + b"\n")

    with pytest.raises(ValueError, match="artifact digest"):
        load_bundle(tmp_path)


def test_loader_rejects_traversal_artifact_paths(tmp_path: Path) -> None:
    question, observation, fact, answer = _spatial_records()
    write_bundle(
        tmp_path,
        episode_id="episode_1",
        source_video_sha256="1" * 64,
        questions=(question,),
        observations=(observation,),
        relation_facts=(fact,),
        relation_intervals=(),
        answers=(answer,),
    )
    public_path = tmp_path / "public" / "manifest.json"
    public = PublicBundleManifest.model_validate_json(public_path.read_bytes())
    public = public.model_copy(
        update={"questions": public.questions.model_copy(update={"path": "../questions.jsonl"})}
    )
    public_bytes = f"{canonical_model_json(public)}\n".encode()
    public_path.write_bytes(public_bytes)
    oracle_path = tmp_path / "oracle" / "manifest.json"
    oracle = OracleBundleManifest.model_validate_json(oracle_path.read_bytes()).model_copy(
        update={"public_manifest_sha256": sha256(public_bytes).hexdigest()}
    )
    oracle_path.write_text(f"{canonical_model_json(oracle)}\n", encoding="utf-8")

    with pytest.raises(ValueError, match="relative canonical path"):
        load_bundle(tmp_path)


def test_loader_rejects_symlinked_artifacts(tmp_path: Path) -> None:
    question, observation, fact, answer = _spatial_records()
    bundle_root = tmp_path / "bundle"
    write_bundle(
        bundle_root,
        episode_id="episode_1",
        source_video_sha256="1" * 64,
        questions=(question,),
        observations=(observation,),
        relation_facts=(fact,),
        relation_intervals=(),
        answers=(answer,),
    )
    questions_path = bundle_root / "public" / "questions.jsonl"
    external_questions_path = tmp_path / "questions.jsonl"
    questions_path.replace(external_questions_path)
    questions_path.symlink_to(external_questions_path)

    with pytest.raises(ValueError, match="symlink"):
        load_bundle(bundle_root)


def test_loader_rejects_symlinked_manifests(tmp_path: Path) -> None:
    question, observation, fact, answer = _spatial_records()
    bundle_root = tmp_path / "bundle"
    write_bundle(
        bundle_root,
        episode_id="episode_1",
        source_video_sha256="1" * 64,
        questions=(question,),
        observations=(observation,),
        relation_facts=(fact,),
        relation_intervals=(),
        answers=(answer,),
    )
    manifest_path = bundle_root / "oracle" / "manifest.json"
    external_manifest_path = tmp_path / "manifest.json"
    manifest_path.replace(external_manifest_path)
    manifest_path.symlink_to(external_manifest_path)

    with pytest.raises(ValueError, match="symlink"):
        load_bundle(bundle_root)


def test_writer_rejects_symlinked_output_directories(tmp_path: Path) -> None:
    question, observation, fact, answer = _spatial_records()
    bundle_root = tmp_path / "bundle"
    bundle_root.mkdir()
    external_root = tmp_path / "external"
    external_root.mkdir()
    (bundle_root / "public").symlink_to(external_root, target_is_directory=True)

    with pytest.raises(ValueError, match="symlink"):
        write_bundle(
            bundle_root,
            episode_id="episode_1",
            source_video_sha256="1" * 64,
            questions=(question,),
            observations=(observation,),
            relation_facts=(fact,),
            relation_intervals=(),
            answers=(answer,),
        )

    assert not tuple(external_root.iterdir())


def test_rejects_duplicate_interval_identities(tmp_path: Path) -> None:
    question, observation, fact, answer = _spatial_records()
    interval = _interval_for_fact(fact)

    with pytest.raises(ValueError, match="duplicate interval ID"):
        write_bundle(
            tmp_path,
            episode_id="episode_1",
            source_video_sha256="1" * 64,
            questions=(question,),
            observations=(observation,),
            relation_facts=(fact,),
            relation_intervals=(interval, interval),
            answers=(answer,),
        )


def test_loader_rejects_malformed_episode_metadata_with_valid_digests(tmp_path: Path) -> None:
    question, observation, fact, answer = _spatial_records()
    write_bundle(
        tmp_path,
        episode_id="episode_1",
        source_video_sha256="1" * 64,
        questions=(question,),
        observations=(observation,),
        relation_facts=(fact,),
        relation_intervals=(),
        answers=(answer,),
    )
    episode_bytes = b"{}\n"
    (tmp_path / "public" / "episode.json").write_bytes(episode_bytes)
    public_path = tmp_path / "public" / "manifest.json"
    public = PublicBundleManifest.model_validate_json(public_path.read_bytes())
    public = public.model_copy(
        update={
            "episode": public.episode.model_copy(
                update={"sha256": sha256(episode_bytes).hexdigest()}
            )
        }
    )
    public_bytes = f"{canonical_model_json(public)}\n".encode()
    public_path.write_bytes(public_bytes)
    oracle_path = tmp_path / "oracle" / "manifest.json"
    oracle = OracleBundleManifest.model_validate_json(oracle_path.read_bytes()).model_copy(
        update={"public_manifest_sha256": sha256(public_bytes).hexdigest()}
    )
    oracle_path.write_text(f"{canonical_model_json(oracle)}\n", encoding="utf-8")

    with pytest.raises(ValueError, match="episode metadata"):
        load_bundle(tmp_path)
