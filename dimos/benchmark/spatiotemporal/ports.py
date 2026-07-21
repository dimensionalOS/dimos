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

"""Frozen callable boundaries for perception and candidate evaluation."""

from collections.abc import Sequence
from enum import StrEnum
from hashlib import sha256
from pathlib import Path
from typing import Protocol, Self

from pydantic import Field, model_validator

from dimos.benchmark.spatiotemporal.models import (
    BoundingBox2D,
    NonEmptyString,
    ObjectObservation,
    OracleBundleManifest,
    PublicBundleManifest,
    Question,
    Sha256Digest,
    StrictFrozenModel,
)
from dimos.benchmark.spatiotemporal.utilities import canonical_json_bytes, canonical_model_json
from dimos.msgs.sensor_msgs.Image import Image


class DetectedObject(StrictFrozenModel):
    """One detector result containing only perception-seam data."""

    object_id: NonEmptyString
    label: NonEmptyString
    box: BoundingBox2D
    confidence: float = Field(ge=0.0, le=1.0, allow_inf_nan=False)


class CandidateReadiness(StrictFrozenModel):
    """Candidate ingestion state with no teacher or oracle artifacts."""

    ready: bool
    ingested_frame_count: int = Field(ge=0)
    detail: NonEmptyString | None = None

    @model_validator(mode="after")
    def validate_readiness(self) -> "CandidateReadiness":
        if self.ready and self.ingested_frame_count == 0:
            raise ValueError("a ready candidate must ingest at least one frame")
        return self


class ReplayInsufficiencyCode(StrEnum):
    """Stable reasons that saved observations cannot produce an evaluation bundle."""

    EMPTY_OBSERVATIONS = "empty_observations"
    MIXED_EPISODES = "mixed_episodes"
    NO_RELATIONS = "no_relations"
    NO_QUESTIONS = "no_questions"


class ReplayInsufficiencyError(ValueError):
    """Actionable replay failure with a machine-readable stable reason."""

    def __init__(self, code: ReplayInsufficiencyCode, message: str) -> None:
        self.code = code
        super().__init__(message)


def replay_bundle_logical_sha256(
    public_manifest: PublicBundleManifest,
    oracle_manifest: OracleBundleManifest,
) -> str:
    """Hash canonical manifest values without extraction-root paths."""
    return sha256(
        canonical_json_bytes(
            {
                "oracle_manifest": oracle_manifest.model_dump(mode="json"),
                "public_manifest": public_manifest.model_dump(mode="json"),
            }
        )
    ).hexdigest()


class ReplayBundleResult(StrictFrozenModel):
    """Root-independent manifests and logical hash returned by replay generation."""

    public_manifest: PublicBundleManifest
    oracle_manifest: OracleBundleManifest
    logical_sha256: Sha256Digest

    @model_validator(mode="after")
    def validate_result(self) -> Self:
        public = self.public_manifest
        oracle = self.oracle_manifest
        identities = {
            (
                public.schema_version,
                public.bundle_id,
                public.episode_id,
                public.source_video_sha256,
            ),
            (
                oracle.schema_version,
                oracle.bundle_id,
                oracle.episode_id,
                oracle.source_video_sha256,
            ),
        }
        if len(identities) != 1:
            raise ValueError("public and oracle manifests must share one release identity")
        expected_public_sha256 = sha256(f"{canonical_model_json(public)}\n".encode()).hexdigest()
        if oracle.public_manifest_sha256 != expected_public_sha256:
            raise ValueError("oracle manifest must bind the canonical public manifest")
        if self.logical_sha256 != replay_bundle_logical_sha256(public, oracle):
            raise ValueError("logical SHA-256 does not match canonical manifests")
        return self


class ObservationBundleGenerator(Protocol):
    """Teacher-side seam that turns canonical observations into one bundle."""

    def generate(
        self,
        observations: Sequence[ObjectObservation],
        output_root: Path,
        source_video_sha256: str,
    ) -> ReplayBundleResult: ...


class ObservationDetector(Protocol):
    """Narrow image-to-detections seam used by video sampling."""

    def detect(self, image: Image) -> Sequence[DetectedObject]: ...

    def close(self) -> None: ...


class CandidateAnswerer(Protocol):
    """Public-only video ingestion and question answering seam."""

    def ingest_video(self, video_path: Path) -> CandidateReadiness: ...

    def answer(self, question: Question) -> str | bool | None: ...

    def close(self) -> None: ...
