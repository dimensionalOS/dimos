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

"""Tests for frozen perception and candidate callable ports."""

from collections.abc import Sequence
from hashlib import sha256
from pathlib import Path

from pydantic import ValidationError
import pytest

from dimos.benchmark.spatiotemporal.models import (
    BoundingBox2D,
    BundleArtifact,
    ObjectObservation,
    OracleBundleManifest,
    PublicBundleManifest,
    Question,
)
from dimos.benchmark.spatiotemporal.ports import (
    CandidateAnswerer,
    CandidateReadiness,
    DetectedObject,
    ObservationBundleGenerator,
    ReplayBundleResult,
    replay_bundle_logical_sha256,
)
from dimos.benchmark.spatiotemporal.utilities import (
    SCHEMA_VERSION,
    canonical_model_json,
    stable_id,
)


def test_detected_object_contains_only_perception_seam_data() -> None:
    detected = DetectedObject(
        object_id="track_7",
        label="mug",
        box=BoundingBox2D(x_min=0.1, y_min=0.2, x_max=0.3, y_max=0.4),
        confidence=0.9,
    )

    assert set(detected.model_dump()) == {"object_id", "label", "box", "confidence"}


def test_candidate_readiness_contains_no_teacher_or_oracle_data() -> None:
    readiness = CandidateReadiness(ready=True, ingested_frame_count=24, detail=None)

    dumped = readiness.model_dump()
    assert dumped == {"ready": True, "ingested_frame_count": 24, "detail": None}
    forbidden = {"observations", "boxes", "intervals", "answers", "evidence", "oracle"}
    assert forbidden.isdisjoint(dumped)
    with pytest.raises(ValidationError, match="at least one frame"):
        CandidateReadiness(ready=True, ingested_frame_count=0, detail=None)


def test_candidate_answerer_signature_is_expressible_without_private_records() -> None:
    class FakeCandidate:
        def ingest_video(self, video_path: Path) -> CandidateReadiness:
            assert video_path == Path("episode.mp4")
            return CandidateReadiness(ready=True, ingested_frame_count=1, detail=None)

        def answer(self, question: Question) -> str | bool | None:
            return None

        def close(self) -> None:
            return None

    candidate = FakeCandidate()
    typed_candidate: CandidateAnswerer = candidate
    assert typed_candidate.ingest_video(Path("episode.mp4")).ready is True


def test_observation_bundle_generator_has_root_independent_typed_result() -> None:
    source_sha256 = "1" * 64
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
        episode=BundleArtifact(path="public/episode.json", sha256="2" * 64, record_count=1),
        questions=BundleArtifact(path="public/questions.jsonl", sha256="3" * 64, record_count=12),
    )
    public_sha256 = sha256(f"{canonical_model_json(public)}\n".encode()).hexdigest()
    oracle = OracleBundleManifest(
        schema_version=SCHEMA_VERSION,
        bundle_id=bundle_id,
        episode_id="episode_1",
        source_video_sha256=source_sha256,
        public_manifest_sha256=public_sha256,
        observations=BundleArtifact(
            path="oracle/observations.jsonl", sha256="5" * 64, record_count=30
        ),
        relation_facts=BundleArtifact(
            path="oracle/relation_facts.jsonl", sha256="6" * 64, record_count=20
        ),
        relation_intervals=BundleArtifact(
            path="oracle/relation_intervals.jsonl", sha256="7" * 64, record_count=8
        ),
        answers=BundleArtifact(path="oracle/answers.jsonl", sha256="8" * 64, record_count=12),
    )
    logical_sha256 = replay_bundle_logical_sha256(public, oracle)
    result = ReplayBundleResult(
        public_manifest=public,
        oracle_manifest=oracle,
        logical_sha256=logical_sha256,
    )

    class FakeGenerator:
        def generate(
            self,
            observations: Sequence[ObjectObservation],
            output_root: Path,
            source_video_sha256: str,
        ) -> ReplayBundleResult:
            assert output_root == Path("release")
            assert source_video_sha256 == source_sha256
            return result

    generator: ObservationBundleGenerator = FakeGenerator()
    assert generator.generate((), Path("release"), source_sha256) == result
    with pytest.raises(ValidationError, match="logical SHA-256"):
        ReplayBundleResult(
            public_manifest=public,
            oracle_manifest=oracle,
            logical_sha256="0" * 64,
        )
