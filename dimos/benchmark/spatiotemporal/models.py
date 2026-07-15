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

"""Strict data contracts for spatiotemporal video QA."""

from enum import StrEnum
from pathlib import PurePosixPath
from typing import Annotated, Self
import unicodedata

from pydantic import AfterValidator, BaseModel, ConfigDict, Field, model_validator

from dimos.benchmark.spatiotemporal.utilities import (
    SCHEMA_VERSION,
    stable_id,
    stable_question_id,
)


def _require_nfc(value: str) -> str:
    try:
        value.encode("utf-8")
    except UnicodeEncodeError as error:
        raise ValueError("string fields must contain valid Unicode scalar values") from error
    if not unicodedata.is_normalized("NFC", value):
        raise ValueError("string fields must be NFC-normalized")
    return value


NonEmptyString = Annotated[str, Field(min_length=1), AfterValidator(_require_nfc)]
QuestionId = Annotated[
    str,
    Field(pattern=r"^question_[0-9a-f]{64}$"),
    AfterValidator(_require_nfc),
]
RelationId = Annotated[
    str,
    Field(pattern=r"^relation_[0-9a-f]{64}$"),
    AfterValidator(_require_nfc),
]
IntervalId = Annotated[
    str,
    Field(pattern=r"^interval_[0-9a-f]{64}$"),
    AfterValidator(_require_nfc),
]
BundleId = Annotated[
    str,
    Field(pattern=r"^bundle_[0-9a-f]{64}$"),
    AfterValidator(_require_nfc),
]
Sha256Digest = Annotated[str, Field(pattern=r"^[0-9a-f]{64}$")]


class SpatialPredicate(StrEnum):
    """Supported image-plane predicates."""

    LEFT_OF = "left-of"
    RIGHT_OF = "right-of"
    ABOVE = "above"
    BELOW = "below"


class TemporalPredicate(StrEnum):
    """Supported strict interval predicates."""

    BEFORE = "before"
    AFTER = "after"


class QuestionKind(StrEnum):
    """Supported public question categories."""

    SPATIAL = "spatial"
    TEMPORAL = "temporal"


class PredictionStatus(StrEnum):
    """Closed set of candidate scoring outcomes."""

    CORRECT = "correct"
    INCORRECT = "incorrect"
    MISSING = "missing"
    INVALID = "invalid"


class StrictFrozenModel(BaseModel):
    """Base for immutable benchmark records with no implicit coercion."""

    model_config = ConfigDict(extra="forbid", frozen=True, strict=True)


class BoundingBox2D(StrictFrozenModel):
    """A normalized image-plane bounding box."""

    x_min: float = Field(ge=0.0, le=1.0, allow_inf_nan=False)
    y_min: float = Field(ge=0.0, le=1.0, allow_inf_nan=False)
    x_max: float = Field(ge=0.0, le=1.0, allow_inf_nan=False)
    y_max: float = Field(ge=0.0, le=1.0, allow_inf_nan=False)

    @model_validator(mode="after")
    def validate_bounds(self) -> Self:
        """Reject empty or inverted boxes."""
        if self.x_min >= self.x_max or self.y_min >= self.y_max:
            raise ValueError("bounding box minimums must be below maximums")
        return self


class ObjectObservation(StrictFrozenModel):
    """One object's canonical observation in a sampled video frame."""

    episode_id: NonEmptyString
    frame_id: int = Field(ge=0)
    timestamp_s: float = Field(allow_inf_nan=False)
    object_id: NonEmptyString
    label: NonEmptyString
    box: BoundingBox2D
    confidence: float = Field(ge=0.0, le=1.0, allow_inf_nan=False)


class RelationFact(StrictFrozenModel):
    """One private accepted spatial relation at a sampled frame."""

    relation_id: RelationId
    episode_id: NonEmptyString
    frame_id: int = Field(ge=0)
    timestamp_s: float = Field(allow_inf_nan=False)
    subject_id: NonEmptyString
    predicate: SpatialPredicate
    object_id: NonEmptyString
    evidence_frame_ids: tuple[int, ...] = Field(min_length=1)

    @model_validator(mode="after")
    def validate_relation_contract(self) -> Self:
        if self.subject_id == self.object_id:
            raise ValueError("relation subject and object IDs must differ")
        if any(frame_id < 0 for frame_id in self.evidence_frame_ids):
            raise ValueError("evidence frame IDs must be non-negative")
        if tuple(sorted(set(self.evidence_frame_ids))) != self.evidence_frame_ids:
            raise ValueError("evidence frame IDs must be ordered and unique")
        if self.evidence_frame_ids != (self.frame_id,):
            raise ValueError("a relation fact must cite exactly its own sample frame")
        expected_id = stable_id(
            "relation",
            {
                "object_ids": (self.subject_id, self.object_id),
                "predicate": self.predicate.value,
                "schema_version": SCHEMA_VERSION,
            },
        )
        if self.relation_id != expected_id:
            raise ValueError("relation ID does not match its executable contract")
        return self


class RelationInterval(StrictFrozenModel):
    """One private bounded interval of a stable accepted relation."""

    interval_id: IntervalId
    relation_id: RelationId
    episode_id: NonEmptyString
    subject_id: NonEmptyString
    predicate: SpatialPredicate
    object_id: NonEmptyString
    start_frame_id: int = Field(ge=0)
    end_frame_id: int = Field(ge=0)
    start_timestamp_s: float = Field(allow_inf_nan=False)
    end_timestamp_s: float = Field(allow_inf_nan=False)
    evidence_frame_ids: tuple[int, ...] = Field(min_length=1)

    @model_validator(mode="after")
    def validate_interval_contract(self) -> Self:
        if self.subject_id == self.object_id:
            raise ValueError("relation subject and object IDs must differ")
        if self.start_frame_id > self.end_frame_id:
            raise ValueError("interval frame bounds must be ordered")
        if self.start_timestamp_s > self.end_timestamp_s:
            raise ValueError("interval timestamp bounds must be ordered")
        if (self.start_frame_id == self.end_frame_id) != (
            self.start_timestamp_s == self.end_timestamp_s
        ):
            raise ValueError("interval frame and timestamp bounds must identify the same samples")
        if tuple(sorted(set(self.evidence_frame_ids))) != self.evidence_frame_ids:
            raise ValueError("evidence frame IDs must be ordered and unique")
        if self.evidence_frame_ids[0] != self.start_frame_id:
            raise ValueError("first evidence frame must equal interval start")
        if self.evidence_frame_ids[-1] != self.end_frame_id:
            raise ValueError("last evidence frame must equal interval end")
        expected_relation_id = stable_id(
            "relation",
            {
                "object_ids": (self.subject_id, self.object_id),
                "predicate": self.predicate.value,
                "schema_version": SCHEMA_VERSION,
            },
        )
        if self.relation_id != expected_relation_id:
            raise ValueError("relation ID does not match interval relation identity")
        expected_interval_id = stable_id(
            "interval",
            {
                "end_frame_id": self.end_frame_id,
                "end_timestamp_s": self.end_timestamp_s,
                "episode_id": self.episode_id,
                "object_id": self.object_id,
                "predicate": self.predicate.value,
                "relation_id": self.relation_id,
                "schema_version": SCHEMA_VERSION,
                "start_frame_id": self.start_frame_id,
                "start_timestamp_s": self.start_timestamp_s,
                "subject_id": self.subject_id,
            },
        )
        if self.interval_id != expected_interval_id:
            raise ValueError("interval ID does not match its executable contract")
        return self


class Question(StrictFrozenModel):
    """A public spatial or temporal question with an answer-free contract."""

    question_id: QuestionId
    episode_id: NonEmptyString
    text: NonEmptyString
    question_kind: QuestionKind
    predicate: SpatialPredicate | TemporalPredicate
    object_ids: tuple[NonEmptyString, NonEmptyString] | tuple[()]
    reference_ids: tuple[RelationId, RelationId] | tuple[()] = ()

    @model_validator(mode="after")
    def validate_executable_contract(self) -> Self:
        """Enforce disjoint variants and IDs derived only from public semantics."""
        if self.question_kind is QuestionKind.SPATIAL:
            if not isinstance(self.predicate, SpatialPredicate):
                raise ValueError("spatial questions require a spatial predicate")
            if len(self.object_ids) != 2 or self.object_ids[0] == self.object_ids[1]:
                raise ValueError("spatial question object IDs must differ")
            if self.reference_ids:
                raise ValueError("spatial questions cannot contain relation references")
        else:
            if not isinstance(self.predicate, TemporalPredicate):
                raise ValueError("temporal questions require a temporal predicate")
            if self.object_ids:
                raise ValueError("temporal questions cannot contain object IDs")
            if len(self.reference_ids) != 2 or self.reference_ids[0] == self.reference_ids[1]:
                raise ValueError("temporal question relation references must differ")
        expected_id = stable_question_id(
            episode_id=self.episode_id,
            object_ids=self.object_ids,
            predicate=self.predicate.value,
            question_kind=self.question_kind.value,
            reference_ids=self.reference_ids,
        )
        if self.question_id != expected_id:
            raise ValueError("question ID does not match its executable contract")
        return self


class OracleAnswer(StrictFrozenModel):
    """A private expected answer and its teacher evidence."""

    question_id: QuestionId
    expected: bool
    evidence_frame_ids: tuple[int, ...] = ()
    evidence_interval_ids: tuple[IntervalId, ...] = ()

    @model_validator(mode="after")
    def validate_evidence(self) -> Self:
        if any(frame_id < 0 for frame_id in self.evidence_frame_ids):
            raise ValueError("evidence frame IDs must be non-negative")
        if tuple(sorted(set(self.evidence_frame_ids))) != self.evidence_frame_ids:
            raise ValueError("evidence frame IDs must be ordered and unique")
        if len(set(self.evidence_interval_ids)) != len(self.evidence_interval_ids):
            raise ValueError("evidence interval IDs must be unique")
        if not self.evidence_frame_ids and not self.evidence_interval_ids:
            raise ValueError("oracle answers require private evidence")
        return self


class Prediction(StrictFrozenModel):
    """One candidate's typed Boolean answer."""

    question_id: QuestionId
    answer: bool


class QuestionResult(StrictFrozenModel):
    """Exact result for one question."""

    question_id: QuestionId
    expected: bool
    predicted: bool
    correct: bool


class BundleArtifact(StrictFrozenModel):
    """One canonical file referenced by a release manifest."""

    path: NonEmptyString
    sha256: Sha256Digest
    record_count: int = Field(ge=0)

    @model_validator(mode="after")
    def validate_path(self) -> Self:
        path = PurePosixPath(self.path)
        if (
            path.is_absolute()
            or "\\" in self.path
            or path.as_posix() != self.path
            or any(part in {"", ".", ".."} for part in path.parts)
        ):
            raise ValueError("artifact path must be a relative canonical path")
        return self


class PublicBundleManifest(StrictFrozenModel):
    """Public release metadata containing no teacher-derived artifacts."""

    schema_version: NonEmptyString
    bundle_id: BundleId
    episode_id: NonEmptyString
    source_video_sha256: Sha256Digest
    episode: BundleArtifact
    questions: BundleArtifact

    @model_validator(mode="after")
    def validate_manifest(self) -> Self:
        if self.schema_version != SCHEMA_VERSION:
            raise ValueError("unsupported schema version")
        expected_bundle_id = stable_id(
            "bundle",
            {
                "episode_id": self.episode_id,
                "schema_version": self.schema_version,
                "source_video_sha256": self.source_video_sha256,
            },
        )
        if self.bundle_id != expected_bundle_id:
            raise ValueError("bundle ID does not match public release identity")
        if not self.episode.path.startswith("public/") or not self.questions.path.startswith(
            "public/"
        ):
            raise ValueError("public manifest artifacts must remain under public/")
        if self.episode.path == self.questions.path:
            raise ValueError("public manifest artifacts must have distinct paths")
        return self


class OracleBundleManifest(StrictFrozenModel):
    """Private release metadata bound to an exact public release."""

    schema_version: NonEmptyString
    bundle_id: BundleId
    episode_id: NonEmptyString
    source_video_sha256: Sha256Digest
    public_manifest_sha256: Sha256Digest
    observations: BundleArtifact
    relation_facts: BundleArtifact
    relation_intervals: BundleArtifact
    answers: BundleArtifact

    @model_validator(mode="after")
    def validate_manifest(self) -> Self:
        if self.schema_version != SCHEMA_VERSION:
            raise ValueError("unsupported schema version")
        expected_bundle_id = stable_id(
            "bundle",
            {
                "episode_id": self.episode_id,
                "schema_version": self.schema_version,
                "source_video_sha256": self.source_video_sha256,
            },
        )
        if self.bundle_id != expected_bundle_id:
            raise ValueError("bundle ID does not match oracle release identity")
        artifacts = (
            self.observations,
            self.relation_facts,
            self.relation_intervals,
            self.answers,
        )
        if any(not artifact.path.startswith("oracle/") for artifact in artifacts):
            raise ValueError("oracle manifest artifacts must remain under oracle/")
        if len({artifact.path for artifact in artifacts}) != len(artifacts):
            raise ValueError("oracle manifest artifacts must have distinct paths")
        return self
