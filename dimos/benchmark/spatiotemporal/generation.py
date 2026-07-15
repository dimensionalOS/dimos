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

from dimos.benchmark.spatiotemporal.models import (
    OracleAnswer,
    Question,
    QuestionKind,
    RelationFact,
    RelationInterval,
    TemporalPredicate,
)
from dimos.benchmark.spatiotemporal.utilities import stable_question_id


def generate_spatial_questions(facts: Sequence[RelationFact]) -> tuple[Question, ...]:
    """Generate one answer-free public question per accepted spatial relation."""
    if len({fact.episode_id for fact in facts}) > 1:
        raise ValueError("spatial questions must be generated from one episode")

    questions: dict[str, Question] = {}
    for fact in facts:
        object_ids = (fact.subject_id, fact.object_id)
        question_id = stable_question_id(
            episode_id=fact.episode_id,
            object_ids=object_ids,
            predicate=fact.predicate.value,
            question_kind=QuestionKind.SPATIAL.value,
        )
        questions.setdefault(
            question_id,
            Question(
                question_id=question_id,
                episode_id=fact.episode_id,
                text=f"Is {fact.subject_id} {fact.predicate.value.replace('-', ' ')} {fact.object_id}?",
                question_kind=QuestionKind.SPATIAL,
                predicate=fact.predicate,
                object_ids=object_ids,
            ),
        )
    return tuple(questions[question_id] for question_id in sorted(questions))


def generate_temporal_question_cases(
    intervals: Sequence[RelationInterval],
) -> tuple[tuple[Question, OracleAnswer], ...]:
    """Generate balanced temporal cases with private interval-backed truth."""
    if len({interval.episode_id for interval in intervals}) > 1:
        raise ValueError("temporal questions must be generated from one episode")

    proofs: dict[
        tuple[str, str],
        dict[tuple[str, str], list[tuple[RelationInterval, RelationInterval]]],
    ] = {}
    for index, left in enumerate(intervals):
        for right in intervals[index + 1 :]:
            if left.relation_id == right.relation_id:
                continue
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
                continue

            relation_pair = (
                min(first.relation_id, second.relation_id),
                max(first.relation_id, second.relation_id),
            )
            orientation = (first.relation_id, second.relation_id)
            proofs.setdefault(relation_pair, {}).setdefault(orientation, []).append((first, second))

    cases: dict[str, tuple[Question, OracleAnswer]] = {}
    for relation_pair in sorted(proofs):
        orientations = proofs[relation_pair]
        if len(orientations) != 1:
            continue
        interval_proofs = next(iter(orientations.values()))
        first, second = min(
            interval_proofs,
            key=lambda proof: (proof[0].interval_id, proof[1].interval_id),
        )
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
                text=(f"Did {references[0]} happen {predicate.value} {references[1]}?"),
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
