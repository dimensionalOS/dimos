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

"""Deterministic public/private evaluation bundle storage."""

from collections.abc import Sequence
from dataclasses import dataclass
from hashlib import sha256
from pathlib import Path
from typing import TypeVar

from pydantic import BaseModel

from dimos.benchmark.spatiotemporal.models import (
    BundleArtifact,
    ObjectObservation,
    OracleAnswer,
    OracleBundleManifest,
    PublicBundleManifest,
    Question,
    RelationFact,
    RelationInterval,
)
from dimos.benchmark.spatiotemporal.utilities import (
    SCHEMA_VERSION,
    canonical_json_bytes,
    canonical_model_json,
    stable_id,
)

RecordT = TypeVar("RecordT", bound=BaseModel)


@dataclass(frozen=True)
class EvaluationBundle:
    """A fully validated public bundle and its private oracle records."""

    public_manifest: PublicBundleManifest
    oracle_manifest: OracleBundleManifest
    questions: tuple[Question, ...]
    observations: tuple[ObjectObservation, ...]
    relation_facts: tuple[RelationFact, ...]
    relation_intervals: tuple[RelationInterval, ...]
    answers: tuple[OracleAnswer, ...]


def _write_jsonl(root: Path, relative_path: str, records: Sequence[BaseModel]) -> BundleArtifact:
    path = _safe_path(root, relative_path)
    content = b"".join(f"{canonical_model_json(record)}\n".encode() for record in records)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(content)
    return BundleArtifact(
        path=path.parent.name + "/" + path.name,
        sha256=sha256(content).hexdigest(),
        record_count=len(records),
    )


def _safe_path(root: Path, relative_path: str) -> Path:
    path = root
    if path.is_symlink():
        raise ValueError(f"bundle path contains a symlink: {relative_path}")
    for part in Path(relative_path).parts:
        path /= part
        if path.is_symlink():
            raise ValueError(f"bundle path contains a symlink: {relative_path}")
    return path


def _read_jsonl(root: Path, artifact: BundleArtifact, model: type[RecordT]) -> tuple[RecordT, ...]:
    path = _safe_path(root, artifact.path)
    return tuple(model.model_validate_json(line) for line in path.read_bytes().splitlines())


def _verify_artifact(root: Path, artifact: BundleArtifact) -> None:
    content = _safe_path(root, artifact.path).read_bytes()
    if sha256(content).hexdigest() != artifact.sha256:
        raise ValueError(f"artifact digest mismatch: {artifact.path}")
    record_count = len(content.splitlines())
    if record_count != artifact.record_count:
        raise ValueError(f"artifact record count mismatch: {artifact.path}")


def _verify_episode_metadata(root: Path, manifest: PublicBundleManifest) -> None:
    expected = (
        canonical_json_bytes(
            {
                "bundle_id": manifest.bundle_id,
                "episode_id": manifest.episode_id,
                "schema_version": manifest.schema_version,
                "source_video_sha256": manifest.source_video_sha256,
            }
        )
        + b"\n"
    )
    if (root / manifest.episode.path).read_bytes() != expected:
        raise ValueError("episode metadata does not match the public manifest")


def _validate_records(
    *,
    episode_id: str,
    questions: Sequence[Question],
    observations: Sequence[ObjectObservation],
    relation_facts: Sequence[RelationFact],
    relation_intervals: Sequence[RelationInterval],
    answers: Sequence[OracleAnswer],
) -> None:
    question_ids = [question.question_id for question in questions]
    if len(set(question_ids)) != len(question_ids):
        raise ValueError("duplicate question ID")
    answer_ids = [answer.question_id for answer in answers]
    if len(set(answer_ids)) != len(answer_ids):
        raise ValueError("duplicate oracle answer reference")
    if any(answer_id not in set(question_ids) for answer_id in answer_ids):
        raise ValueError("oracle answer contains a foreign question ID")
    if any(question_id not in set(answer_ids) for question_id in question_ids):
        raise ValueError("public question is missing oracle answer")
    relation_ids = {fact.relation_id for fact in relation_facts} | {
        interval.relation_id for interval in relation_intervals
    }
    if any(
        reference_id not in relation_ids
        for question in questions
        for reference_id in question.reference_ids
    ):
        raise ValueError("question contains a foreign relation reference")
    interval_id_list = [interval.interval_id for interval in relation_intervals]
    interval_ids = set(interval_id_list)
    if len(interval_ids) != len(interval_id_list):
        raise ValueError("duplicate interval ID")
    if any(
        interval_id not in interval_ids
        for answer in answers
        for interval_id in answer.evidence_interval_ids
    ):
        raise ValueError("oracle answer contains a foreign interval reference")
    if (
        any(question.episode_id != episode_id for question in questions)
        or any(observation.episode_id != episode_id for observation in observations)
        or any(fact.episode_id != episode_id for fact in relation_facts)
        or any(interval.episode_id != episode_id for interval in relation_intervals)
    ):
        raise ValueError("bundle contains a record from a foreign episode")


def write_bundle(
    root: Path,
    *,
    episode_id: str,
    source_video_sha256: str,
    questions: Sequence[Question],
    observations: Sequence[ObjectObservation],
    relation_facts: Sequence[RelationFact],
    relation_intervals: Sequence[RelationInterval],
    answers: Sequence[OracleAnswer],
) -> None:
    """Write one deterministic public release and its private oracle companion."""
    _validate_records(
        episode_id=episode_id,
        questions=questions,
        observations=observations,
        relation_facts=relation_facts,
        relation_intervals=relation_intervals,
        answers=answers,
    )

    root.mkdir(parents=True, exist_ok=True)
    bundle_id = stable_id(
        "bundle",
        {
            "episode_id": episode_id,
            "schema_version": SCHEMA_VERSION,
            "source_video_sha256": source_video_sha256,
        },
    )
    episode_path = _safe_path(root, "public/episode.json")
    episode_content = (
        canonical_json_bytes(
            {
                "bundle_id": bundle_id,
                "episode_id": episode_id,
                "schema_version": SCHEMA_VERSION,
                "source_video_sha256": source_video_sha256,
            }
        )
        + b"\n"
    )
    episode_path.parent.mkdir(parents=True, exist_ok=True)
    episode_path.write_bytes(episode_content)

    public_manifest = PublicBundleManifest(
        schema_version=SCHEMA_VERSION,
        bundle_id=bundle_id,
        episode_id=episode_id,
        source_video_sha256=source_video_sha256,
        episode=BundleArtifact(
            path="public/episode.json",
            sha256=sha256(episode_content).hexdigest(),
            record_count=1,
        ),
        questions=_write_jsonl(root, "public/questions.jsonl", questions),
    )
    public_manifest_bytes = f"{canonical_model_json(public_manifest)}\n".encode()
    _safe_path(root, "public/manifest.json").write_bytes(public_manifest_bytes)

    oracle_manifest = OracleBundleManifest(
        schema_version=SCHEMA_VERSION,
        bundle_id=bundle_id,
        episode_id=episode_id,
        source_video_sha256=source_video_sha256,
        public_manifest_sha256=sha256(public_manifest_bytes).hexdigest(),
        observations=_write_jsonl(root, "oracle/observations.jsonl", observations),
        relation_facts=_write_jsonl(root, "oracle/relation_facts.jsonl", relation_facts),
        relation_intervals=_write_jsonl(
            root, "oracle/relation_intervals.jsonl", relation_intervals
        ),
        answers=_write_jsonl(root, "oracle/answers.jsonl", answers),
    )
    _safe_path(root, "oracle/manifest.json").write_text(
        f"{canonical_model_json(oracle_manifest)}\n",
        encoding="utf-8",
    )


def load_bundle(root: Path) -> EvaluationBundle:
    """Load one public release together with its private oracle companion."""
    public_manifest_path = _safe_path(root, "public/manifest.json")
    oracle_manifest_path = _safe_path(root, "oracle/manifest.json")
    public_manifest = PublicBundleManifest.model_validate_json(public_manifest_path.read_bytes())
    oracle_manifest = OracleBundleManifest.model_validate_json(oracle_manifest_path.read_bytes())
    public_manifest_bytes = public_manifest_path.read_bytes()
    if sha256(public_manifest_bytes).hexdigest() != oracle_manifest.public_manifest_sha256:
        raise ValueError("oracle manifest does not match the public manifest digest")
    if (
        oracle_manifest.bundle_id != public_manifest.bundle_id
        or oracle_manifest.episode_id != public_manifest.episode_id
        or oracle_manifest.source_video_sha256 != public_manifest.source_video_sha256
    ):
        raise ValueError("public and oracle manifest identities differ")
    artifacts = (
        public_manifest.episode,
        public_manifest.questions,
        oracle_manifest.observations,
        oracle_manifest.relation_facts,
        oracle_manifest.relation_intervals,
        oracle_manifest.answers,
    )
    for artifact in artifacts:
        _verify_artifact(root, artifact)
    _verify_episode_metadata(root, public_manifest)
    bundle = EvaluationBundle(
        public_manifest=public_manifest,
        oracle_manifest=oracle_manifest,
        questions=_read_jsonl(root, public_manifest.questions, Question),
        observations=_read_jsonl(root, oracle_manifest.observations, ObjectObservation),
        relation_facts=_read_jsonl(root, oracle_manifest.relation_facts, RelationFact),
        relation_intervals=_read_jsonl(root, oracle_manifest.relation_intervals, RelationInterval),
        answers=_read_jsonl(root, oracle_manifest.answers, OracleAnswer),
    )
    _validate_records(
        episode_id=public_manifest.episode_id,
        questions=bundle.questions,
        observations=bundle.observations,
        relation_facts=bundle.relation_facts,
        relation_intervals=bundle.relation_intervals,
        answers=bundle.answers,
    )
    return bundle
