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

"""Exact scoring for typed spatiotemporal predictions."""

from collections.abc import Mapping
from dataclasses import dataclass

from dimos.benchmark.spatiotemporal.models import (
    IntervalId,
    OracleAnswer,
    Prediction,
    PredictionStatus,
    Question,
    QuestionId,
    QuestionKind,
    QuestionResult,
    SpatialPredicate,
    TemporalPredicate,
)

QuestionPredicate = SpatialPredicate | TemporalPredicate


@dataclass(frozen=True, slots=True)
class ScoreSummary:
    """Aggregate exact-match counts for one report slice."""

    total: int
    correct: int
    accuracy: float


@dataclass(frozen=True, slots=True)
class QuestionDiagnostic:
    """Evidence-linked diagnostic information for one public question."""

    question_id: QuestionId
    status: PredictionStatus
    question_kind: QuestionKind
    predicate: QuestionPredicate
    expected: bool
    predicted: bool | None
    evidence_frame_ids: tuple[int, ...]
    evidence_interval_ids: tuple[IntervalId, ...]


@dataclass(frozen=True, slots=True)
class EvaluationReport:
    """Aggregate candidate scores linked to source identity and teacher evidence."""

    source_video_sha256: str
    overall: ScoreSummary
    status_counts: Mapping[PredictionStatus, int]
    by_family: Mapping[QuestionKind, ScoreSummary]
    by_predicate: Mapping[QuestionPredicate, ScoreSummary]
    diagnostics: tuple[QuestionDiagnostic, ...]


def summarize_statuses(statuses: tuple[PredictionStatus, ...]) -> ScoreSummary:
    """Summarize exact candidate statuses, counting all outcomes in the denominator."""
    total = len(statuses)
    correct = statuses.count(PredictionStatus.CORRECT)
    return ScoreSummary(total=total, correct=correct, accuracy=correct / total if total else 0.0)


def score_prediction(
    question: Question,
    oracle: OracleAnswer,
    prediction: Prediction,
) -> QuestionResult:
    """Score one typed prediction against separately supplied private truth."""
    if len({question.question_id, oracle.question_id, prediction.question_id}) != 1:
        raise ValueError("question IDs must match for scoring")
    return QuestionResult(
        question_id=question.question_id,
        expected=oracle.expected,
        predicted=prediction.answer,
        correct=prediction.answer is oracle.expected,
    )
