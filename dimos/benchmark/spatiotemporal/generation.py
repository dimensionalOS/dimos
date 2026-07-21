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

"""Deterministic public question generation from accepted relation facts."""

from collections.abc import Sequence
from itertools import combinations, product

from dimos.benchmark.spatiotemporal.models import (
    OracleAnswer,
    Question,
    QuestionKind,
    RelationFact,
    RelationInterval,
    SpatialPredicate,
    TemporalPredicate,
)
from dimos.benchmark.spatiotemporal.utilities import stable_question_id

_OPPOSITE_SPATIAL_PREDICATE = {
    SpatialPredicate.LEFT_OF: SpatialPredicate.RIGHT_OF,
    SpatialPredicate.RIGHT_OF: SpatialPredicate.LEFT_OF,
    SpatialPredicate.ABOVE: SpatialPredicate.BELOW,
    SpatialPredicate.BELOW: SpatialPredicate.ABOVE,
}


def _canonical_spatial_relation(
    subject_id: str, predicate: SpatialPredicate, object_id: str
) -> tuple[str, SpatialPredicate, str]:
    if predicate is SpatialPredicate.RIGHT_OF:
        return object_id, SpatialPredicate.LEFT_OF, subject_id
    if predicate is SpatialPredicate.BELOW:
        return object_id, SpatialPredicate.ABOVE, subject_id
    return subject_id, predicate, object_id


def _describe_relation(interval: RelationInterval) -> str:
    return (
        f"{interval.subject_id} {interval.predicate.value.replace('-', ' ')} {interval.object_id}"
    )


def generate_spatial_question_cases(
    facts: Sequence[RelationFact],
    *,
    sample_frame_ids: Sequence[int] | None = None,
) -> tuple[tuple[Question, OracleAnswer], ...]:
    """Generate episode-level positives and only globally absent negative controls."""
    if len({fact.episode_id for fact in facts}) > 1:
        raise ValueError("spatial questions must be generated from one episode")

    grouped: dict[tuple[str, str, SpatialPredicate, str], set[int]] = {}
    for fact in facts:
        subject_id, predicate, object_id = _canonical_spatial_relation(
            fact.subject_id, fact.predicate, fact.object_id
        )
        key = (fact.episode_id, subject_id, predicate, object_id)
        grouped.setdefault(key, set()).update(fact.evidence_frame_ids)
    episode_evidence = tuple(
        sorted(
            set(sample_frame_ids)
            if sample_frame_ids is not None
            else {frame_id for evidence in grouped.values() for frame_id in evidence}
        )
    )

    cases: dict[str, tuple[Question, OracleAnswer]] = {}
    for (episode_id, subject_id, accepted_predicate, object_id), evidence in grouped.items():
        question_specs = [(accepted_predicate, True, tuple(sorted(evidence)))]
        inverse_key = (episode_id, object_id, accepted_predicate, subject_id)
        if inverse_key not in grouped:
            question_specs.append(
                (
                    _OPPOSITE_SPATIAL_PREDICATE[accepted_predicate],
                    False,
                    episode_evidence,
                )
            )
        for predicate, expected, evidence_frame_ids in question_specs:
            object_ids = (subject_id, object_id)
            question_id = stable_question_id(
                episode_id=episode_id,
                object_ids=object_ids,
                predicate=predicate.value,
                question_kind=QuestionKind.SPATIAL.value,
            )
            cases[question_id] = (
                Question(
                    question_id=question_id,
                    episode_id=episode_id,
                    text=(
                        f"Did {subject_id} ever appear "
                        f"{predicate.value.replace('-', ' ')} {object_id}?"
                    ),
                    question_kind=QuestionKind.SPATIAL,
                    predicate=predicate,
                    object_ids=object_ids,
                ),
                OracleAnswer(
                    question_id=question_id,
                    expected=expected,
                    evidence_frame_ids=evidence_frame_ids,
                ),
            )
    return tuple(cases[question_id] for question_id in sorted(cases))


def generate_spatial_questions(facts: Sequence[RelationFact]) -> tuple[Question, ...]:
    """Generate one answer-free public question per accepted spatial relation."""
    return tuple(
        question for question, answer in generate_spatial_question_cases(facts) if answer.expected
    )


def generate_temporal_question_cases(
    intervals: Sequence[RelationInterval],
) -> tuple[tuple[Question, OracleAnswer], ...]:
    """Generate balanced temporal cases with private interval-backed truth."""
    if len({interval.episode_id for interval in intervals}) > 1:
        raise ValueError("temporal questions must be generated from one episode")

    by_relation: dict[str, list[RelationInterval]] = {}
    for interval in intervals:
        by_relation.setdefault(interval.relation_id, []).append(interval)

    proofs: dict[tuple[str, str], list[tuple[RelationInterval, RelationInterval]]] = {}
    for left_id, right_id in combinations(sorted(by_relation), 2):
        interval_proofs: list[tuple[RelationInterval, RelationInterval]] = []
        orientations: set[tuple[str, str]] = set()
        for left, right in product(by_relation[left_id], by_relation[right_id]):
            if (
                left.end_frame_id < right.start_frame_id
                and left.end_timestamp_s < right.start_timestamp_s
            ):
                first, second = left, right
            elif (
                right.end_frame_id < left.start_frame_id
                and right.end_timestamp_s < left.start_timestamp_s
            ):
                first, second = right, left
            else:
                interval_proofs = []
                break
            interval_proofs.append((first, second))
            orientations.add((first.relation_id, second.relation_id))
        if interval_proofs and len(orientations) == 1:
            proofs[(left_id, right_id)] = interval_proofs

    cases: dict[str, tuple[Question, OracleAnswer]] = {}
    for relation_pair in sorted(proofs):
        interval_proofs = proofs[relation_pair]
        first, second = min(
            interval_proofs,
            key=lambda proof: (proof[0].interval_id, proof[1].interval_id),
        )
        descriptions = {
            first.relation_id: _describe_relation(first),
            second.relation_id: _describe_relation(second),
        }
        for predicate, references, expected in (
            (
                TemporalPredicate.BEFORE,
                (first.relation_id, second.relation_id),
                True,
            ),
            (
                TemporalPredicate.AFTER,
                (second.relation_id, first.relation_id),
                True,
            ),
            (
                TemporalPredicate.AFTER,
                (first.relation_id, second.relation_id),
                False,
            ),
            (
                TemporalPredicate.BEFORE,
                (second.relation_id, first.relation_id),
                False,
            ),
        ):
            question_id = stable_question_id(
                episode_id=first.episode_id,
                predicate=predicate.value,
                question_kind=QuestionKind.TEMPORAL.value,
                reference_ids=references,
            )
            question = Question(
                question_id=question_id,
                episode_id=first.episode_id,
                text=(
                    f'Did "{descriptions[references[0]]}" happen {predicate.value} '
                    f'"{descriptions[references[1]]}"?'
                ),
                question_kind=QuestionKind.TEMPORAL,
                predicate=predicate,
                object_ids=(),
                reference_ids=references,
            )
            cases[question_id] = (
                question,
                OracleAnswer(
                    question_id=question_id,
                    expected=expected,
                    evidence_interval_ids=(first.interval_id, second.interval_id),
                ),
            )

    return tuple(cases[question_id] for question_id in sorted(cases))


def generate_temporal_questions(
    intervals: Sequence[RelationInterval],
) -> tuple[Question, ...]:
    """Generate public questions for unambiguous strict interval orderings."""
    return tuple(question for question, _ in generate_temporal_question_cases(intervals))
