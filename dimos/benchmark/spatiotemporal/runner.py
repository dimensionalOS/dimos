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

"""Candidate evaluation runner helpers."""

from collections.abc import Collection, Mapping, Sequence
import re

from dimos.benchmark.spatiotemporal.models import (
    OracleAnswer,
    Prediction,
    PredictionStatus,
    Question,
    QuestionId,
    QuestionKind,
)
from dimos.benchmark.spatiotemporal.scoring import (
    EvaluationReport,
    QuestionDiagnostic,
    QuestionPredicate,
    summarize_statuses,
)


def parse_candidate_prediction(
    question_id: QuestionId,
    raw_answer: str | bool | None,
    expected: bool,
) -> tuple[PredictionStatus, Prediction | None]:
    """Parse one candidate answer and classify it against private truth."""
    if raw_answer is None:
        return PredictionStatus.MISSING, None
    if isinstance(raw_answer, bool):
        answer = raw_answer
    elif raw_answer == "yes":
        answer = True
    elif raw_answer == "no":
        answer = False
    else:
        return PredictionStatus.INVALID, None

    prediction = Prediction(question_id=question_id, answer=answer)
    status = PredictionStatus.CORRECT if answer is expected else PredictionStatus.INCORRECT
    return status, prediction


def build_evaluation_report(
    questions: Sequence[Question],
    oracles: Mapping[QuestionId, OracleAnswer],
    raw_answers: Mapping[QuestionId, str | bool | None],
    *,
    source_video_sha256: str,
    diagnostic_statuses: Collection[PredictionStatus] = tuple(PredictionStatus),
) -> EvaluationReport:
    """Join public questions with private truth and aggregate candidate outcomes."""
    if re.fullmatch(r"[0-9a-f]{64}", source_video_sha256) is None:
        raise ValueError("source video SHA-256 must be 64 lowercase hexadecimal characters")

    question_ids = tuple(question.question_id for question in questions)
    if len(set(question_ids)) != len(question_ids):
        raise ValueError("question IDs must be unique")
    if set(oracles) != set(question_ids):
        raise ValueError("oracle answers must exactly match public questions")
    if set(raw_answers).difference(question_ids):
        raise ValueError("candidate answers contain unknown question IDs")

    selected_statuses = frozenset(diagnostic_statuses)
    statuses: list[PredictionStatus] = []
    diagnostics: list[QuestionDiagnostic] = []
    statuses_by_family: dict[QuestionKind, list[PredictionStatus]] = {}
    statuses_by_predicate: dict[QuestionPredicate, list[PredictionStatus]] = {}

    for question in questions:
        oracle = oracles[question.question_id]
        if oracle.question_id != question.question_id:
            raise ValueError("oracle map keys and question IDs must match")
        status, prediction = parse_candidate_prediction(
            question.question_id,
            raw_answers.get(question.question_id),
            oracle.expected,
        )
        statuses.append(status)
        statuses_by_family.setdefault(question.question_kind, []).append(status)
        statuses_by_predicate.setdefault(question.predicate, []).append(status)
        if status in selected_statuses:
            diagnostics.append(
                QuestionDiagnostic(
                    question_id=question.question_id,
                    status=status,
                    question_kind=question.question_kind,
                    predicate=question.predicate,
                    expected=oracle.expected,
                    predicted=prediction.answer if prediction is not None else None,
                    evidence_frame_ids=oracle.evidence_frame_ids,
                    evidence_interval_ids=oracle.evidence_interval_ids,
                )
            )

    return EvaluationReport(
        source_video_sha256=source_video_sha256,
        overall=summarize_statuses(tuple(statuses)),
        status_counts={status: statuses.count(status) for status in PredictionStatus},
        by_family={
            family: summarize_statuses(tuple(family_statuses))
            for family, family_statuses in statuses_by_family.items()
        },
        by_predicate={
            predicate: summarize_statuses(tuple(predicate_statuses))
            for predicate, predicate_statuses in statuses_by_predicate.items()
        },
        diagnostics=tuple(diagnostics),
    )
