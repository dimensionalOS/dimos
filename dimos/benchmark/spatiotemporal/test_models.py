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

"""Tests for strict spatiotemporal QA contracts."""

from pydantic import ValidationError
import pytest

from dimos.benchmark.spatiotemporal.models import (
    BoundingBox2D,
    BundleArtifact,
    ObjectObservation,
    OracleAnswer,
    OracleBundleManifest,
    PredictionStatus,
    PublicBundleManifest,
    Question,
    QuestionKind,
    RelationFact,
    RelationInterval,
    SpatialPredicate,
    TemporalPredicate,
)
from dimos.benchmark.spatiotemporal.utilities import SCHEMA_VERSION, stable_id


def test_bounding_box_accepts_only_strict_normalized_valid_bounds() -> None:
    box = BoundingBox2D(x_min=0.1, y_min=0.2, x_max=0.4, y_max=0.8)

    assert box.model_dump() == {
        "x_min": 0.1,
        "y_min": 0.2,
        "x_max": 0.4,
        "y_max": 0.8,
    }

    invalid_payloads = (
        {"x_min": 0.4, "y_min": 0.2, "x_max": 0.4, "y_max": 0.8},
        {"x_min": 0.5, "y_min": 0.2, "x_max": 0.4, "y_max": 0.8},
        {"x_min": 0.1, "y_min": 0.8, "x_max": 0.4, "y_max": 0.8},
        {"x_min": -0.1, "y_min": 0.2, "x_max": 0.4, "y_max": 0.8},
        {"x_min": 0.1, "y_min": 0.2, "x_max": 1.1, "y_max": 0.8},
        {"x_min": float("nan"), "y_min": 0.2, "x_max": 0.4, "y_max": 0.8},
        {"x_min": "0.1", "y_min": 0.2, "x_max": 0.4, "y_max": 0.8},
    )
    for payload in invalid_payloads:
        with pytest.raises(ValidationError):
            BoundingBox2D.model_validate(payload)

    with pytest.raises(ValidationError):
        BoundingBox2D.model_validate(
            {
                "x_min": 0.1,
                "y_min": 0.2,
                "x_max": 0.4,
                "y_max": 0.8,
                "unexpected": True,
            }
        )
    with pytest.raises(ValidationError):
        box.x_min = 0.0


def test_object_observation_rejects_invalid_identity_time_and_confidence() -> None:
    observation = ObjectObservation(
        episode_id="episode_1",
        frame_id=12,
        timestamp_s=0.5,
        object_id="mug_1",
        label="mug",
        box=BoundingBox2D(x_min=0.1, y_min=0.2, x_max=0.3, y_max=0.4),
        confidence=0.9,
    )

    assert observation.object_id == "mug_1"

    invalid_overrides = (
        {"episode_id": ""},
        {"frame_id": -1},
        {"timestamp_s": float("inf")},
        {"object_id": ""},
        {"object_id": "mu\u0301g"},
        {"label": ""},
        {"confidence": -0.1},
        {"confidence": 1.1},
    )
    payload = observation.model_dump()
    for override in invalid_overrides:
        with pytest.raises(ValidationError):
            ObjectObservation.model_validate(payload | override)


def test_object_observation_allows_finite_negative_timestamp_translation() -> None:
    observation = ObjectObservation(
        episode_id="episode_1",
        frame_id=12,
        timestamp_s=-0.5,
        object_id="mug_1",
        label="mug",
        box=BoundingBox2D(x_min=0.1, y_min=0.2, x_max=0.3, y_max=0.4),
        confidence=0.9,
    )

    assert observation.timestamp_s == -0.5


def test_question_rejects_malformed_id_and_non_nfc_object_identity() -> None:
    payload = {
        "question_id": "question_4ef1579875e841ec7602de2e9cfc95f0bfdfb5e8d2e38ca798f462f8f72eda1e",
        "episode_id": "episode_1",
        "text": "Is the mug left of the laptop at the end?",
        "question_kind": QuestionKind.SPATIAL,
        "predicate": SpatialPredicate.LEFT_OF,
        "object_ids": ("obj_red", "obj_blue"),
    }

    assert Question.model_validate(payload).object_ids == ("obj_red", "obj_blue")
    with pytest.raises(ValidationError):
        Question.model_validate(payload | {"question_id": "question_1"})
    with pytest.raises(ValidationError):
        Question.model_validate(payload | {"question_id": f"question_{'0' * 64}"})
    with pytest.raises(ValidationError):
        Question.model_validate(payload | {"object_ids": ("mu\u0301g_1", "laptop_1")})
    with pytest.raises(ValidationError):
        Question.model_validate(payload | {"object_ids": ("\ud800", "laptop_1")})
    with pytest.raises(ValidationError):
        Question.model_validate(payload | {"object_ids": ("obj_red", "obj_red")})


def test_relation_fact_has_stable_private_sample_identity() -> None:
    contract = {
        "object_ids": ("obj_red", "obj_blue"),
        "predicate": SpatialPredicate.LEFT_OF.value,
        "schema_version": SCHEMA_VERSION,
    }
    fact = RelationFact(
        relation_id=stable_id("relation", contract),
        episode_id="episode_1",
        frame_id=12,
        timestamp_s=0.5,
        subject_id="obj_red",
        predicate=SpatialPredicate.LEFT_OF,
        object_id="obj_blue",
        evidence_frame_ids=(12,),
    )

    assert (
        fact.relation_id
        == "relation_50c38f99a34ced799c3b8c8bd3417ac288a6ce9481dc9de19525042a2e4f5ea3"
    )
    with pytest.raises(ValidationError, match="relation ID"):
        RelationFact.model_validate(fact.model_dump() | {"relation_id": f"relation_{'0' * 64}"})
    with pytest.raises(ValidationError, match="must differ"):
        RelationFact.model_validate(fact.model_dump() | {"object_id": "obj_red"})
    with pytest.raises(ValidationError, match="must cite exactly"):
        RelationFact.model_validate(fact.model_dump() | {"evidence_frame_ids": (11, 12)})
    with pytest.raises(ValidationError, match="ordered and unique"):
        RelationFact.model_validate(fact.model_dump() | {"evidence_frame_ids": (12, 12)})


def test_relation_interval_has_strict_consistent_bounds() -> None:
    relation_id = stable_id(
        "relation",
        {
            "object_ids": ("obj_red", "obj_blue"),
            "predicate": SpatialPredicate.LEFT_OF.value,
            "schema_version": SCHEMA_VERSION,
        },
    )
    contract = {
        "end_frame_id": 14,
        "end_timestamp_s": 0.7,
        "episode_id": "episode_1",
        "object_id": "obj_blue",
        "predicate": SpatialPredicate.LEFT_OF.value,
        "relation_id": relation_id,
        "schema_version": SCHEMA_VERSION,
        "start_frame_id": 12,
        "start_timestamp_s": 0.5,
        "subject_id": "obj_red",
    }
    interval = RelationInterval(
        interval_id=stable_id("interval", contract),
        relation_id=relation_id,
        episode_id="episode_1",
        subject_id="obj_red",
        predicate=SpatialPredicate.LEFT_OF,
        object_id="obj_blue",
        start_frame_id=12,
        end_frame_id=14,
        start_timestamp_s=0.5,
        end_timestamp_s=0.7,
        evidence_frame_ids=(12, 13, 14),
    )

    assert interval.start_timestamp_s < interval.end_timestamp_s
    for end_frame_id, end_timestamp_s in ((12, 0.7), (14, 0.5)):
        contradictory_contract = contract | {
            "end_frame_id": end_frame_id,
            "end_timestamp_s": end_timestamp_s,
        }
        contradictory_payload = interval.model_dump() | {
            "end_frame_id": end_frame_id,
            "end_timestamp_s": end_timestamp_s,
            "interval_id": stable_id("interval", contradictory_contract),
            "evidence_frame_ids": (12,) if end_frame_id == 12 else (12, 13, 14),
        }
        with pytest.raises(ValidationError, match="frame and timestamp bounds"):
            RelationInterval.model_validate(contradictory_payload)

    for override in (
        {"end_frame_id": 11},
        {"end_timestamp_s": 0.4},
        {"evidence_frame_ids": (12, 14, 13)},
        {"interval_id": f"interval_{'0' * 64}"},
    ):
        with pytest.raises(ValidationError):
            RelationInterval.model_validate(interval.model_dump() | override)


def test_question_variants_are_disjoint_and_have_stable_ids() -> None:
    references = (f"relation_{'1' * 64}", f"relation_{'2' * 64}")
    question_id = stable_id(
        "question",
        {
            "object_ids": (),
            "predicate": TemporalPredicate.BEFORE.value,
            "question_kind": QuestionKind.TEMPORAL.value,
            "reference_ids": references,
            "schema_version": SCHEMA_VERSION,
        },
    )
    temporal = Question(
        question_id=question_id,
        episode_id="episode_1",
        text="Did relation one happen before relation two?",
        question_kind=QuestionKind.TEMPORAL,
        predicate=TemporalPredicate.BEFORE,
        object_ids=(),
        reference_ids=references,
    )

    assert temporal.predicate is TemporalPredicate.BEFORE
    invalid_variants = (
        {"question_kind": QuestionKind.SPATIAL},
        {"predicate": SpatialPredicate.LEFT_OF},
        {"object_ids": ("obj_red", "obj_blue")},
        {"reference_ids": (references[0], references[0])},
        {"reference_ids": ("relation_not-a-digest", references[1])},
    )
    for override in invalid_variants:
        with pytest.raises(ValidationError):
            Question.model_validate(temporal.model_dump() | override)


def test_oracle_evidence_and_prediction_status_contracts_are_private_and_closed() -> None:
    question_id = f"question_{'1' * 64}"
    oracle = OracleAnswer(
        question_id=question_id,
        expected=True,
        evidence_frame_ids=(12,),
        evidence_interval_ids=(f"interval_{'2' * 64}",),
    )

    assert oracle.evidence_interval_ids == (f"interval_{'2' * 64}",)
    assert {status.value for status in PredictionStatus} == {
        "correct",
        "incorrect",
        "missing",
        "invalid",
    }


def test_public_and_oracle_manifests_share_only_release_identity() -> None:
    source_sha256 = "2" * 64
    bundle_id = stable_id(
        "bundle",
        {
            "episode_id": "episode_1",
            "schema_version": SCHEMA_VERSION,
            "source_video_sha256": source_sha256,
        },
    )
    public = PublicBundleManifest(
        schema_version=SCHEMA_VERSION,
        bundle_id=bundle_id,
        episode_id="episode_1",
        source_video_sha256=source_sha256,
        episode=BundleArtifact(path="public/episode.json", sha256="3" * 64, record_count=1),
        questions=BundleArtifact(path="public/questions.jsonl", sha256="4" * 64, record_count=12),
    )
    oracle = OracleBundleManifest(
        schema_version=SCHEMA_VERSION,
        bundle_id=public.bundle_id,
        episode_id=public.episode_id,
        source_video_sha256=source_sha256,
        public_manifest_sha256="8" * 64,
        observations=BundleArtifact(
            path="oracle/observations.jsonl", sha256="5" * 64, record_count=30
        ),
        relation_intervals=BundleArtifact(
            path="oracle/relation_intervals.jsonl", sha256="6" * 64, record_count=8
        ),
        relation_facts=BundleArtifact(
            path="oracle/relation_facts.jsonl", sha256="9" * 64, record_count=20
        ),
        answers=BundleArtifact(path="oracle/answers.jsonl", sha256="7" * 64, record_count=12),
    )

    public_keys = set(public.model_dump())
    assert "observations" not in public_keys
    assert "answers" not in public_keys
    assert oracle.bundle_id == public.bundle_id
    with pytest.raises(ValidationError, match="relative canonical path"):
        BundleArtifact(path="../oracle/answers.jsonl", sha256="7" * 64, record_count=12)
