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

"""Tests for candidate answer parsing and evidence-linked reporting."""

import pytest

from dimos.benchmark.spatiotemporal.models import (
    OracleAnswer,
    PredictionStatus,
    Question,
    QuestionKind,
    SpatialPredicate,
)
from dimos.benchmark.spatiotemporal.runner import (
    build_evaluation_report,
    parse_candidate_prediction,
)
from dimos.benchmark.spatiotemporal.utilities import stable_question_id

QUESTION_ID = stable_question_id(
    episode_id="episode_1",
    object_ids=("obj_red", "obj_blue"),
    predicate=SpatialPredicate.LEFT_OF.value,
    question_kind=QuestionKind.SPATIAL.value,
)


@pytest.mark.parametrize(
    ("raw_answer", "expected", "expected_status", "expected_prediction"),
    (
        (True, True, PredictionStatus.CORRECT, True),
        (False, True, PredictionStatus.INCORRECT, False),
        ("yes", True, PredictionStatus.CORRECT, True),
        ("no", False, PredictionStatus.CORRECT, False),
        (None, True, PredictionStatus.MISSING, None),
        ("true", True, PredictionStatus.INVALID, None),
        ("yes, definitely", True, PredictionStatus.INVALID, None),
    ),
)
def test_parses_only_boolean_and_explicit_yes_no_answers(
    raw_answer: str | bool | None,
    expected: bool,
    expected_status: PredictionStatus,
    expected_prediction: bool | None,
) -> None:
    status, prediction = parse_candidate_prediction(QUESTION_ID, raw_answer, expected)

    assert status is expected_status
    if expected_prediction is None:
        assert prediction is None
    else:
        assert prediction is not None
        assert prediction.question_id == QUESTION_ID
        assert prediction.answer is expected_prediction


def test_builds_evidence_linked_aggregate_report_with_filtered_diagnostics() -> None:
    questions = tuple(
        Question(
            question_id=stable_question_id(
                episode_id="episode_1",
                object_ids=("obj_red", "obj_blue"),
                predicate=predicate.value,
                question_kind=QuestionKind.SPATIAL.value,
            ),
            episode_id="episode_1",
            text=f"Is the red object {predicate.value} the blue object?",
            question_kind=QuestionKind.SPATIAL,
            predicate=predicate,
            object_ids=("obj_red", "obj_blue"),
        )
        for predicate in (SpatialPredicate.LEFT_OF, SpatialPredicate.ABOVE)
    )
    oracles = {
        questions[0].question_id: OracleAnswer(
            question_id=questions[0].question_id,
            expected=True,
            evidence_frame_ids=(4,),
        ),
        questions[1].question_id: OracleAnswer(
            question_id=questions[1].question_id,
            expected=False,
            evidence_frame_ids=(9,),
        ),
    }

    report = build_evaluation_report(
        questions,
        oracles,
        {questions[0].question_id: "yes"},
        source_video_sha256="a" * 64,
        diagnostic_statuses={PredictionStatus.MISSING},
    )

    assert report.source_video_sha256 == "a" * 64
    assert report.overall.total == 2
    assert report.overall.correct == 1
    assert report.overall.accuracy == 0.5
    assert report.status_counts == {
        PredictionStatus.CORRECT: 1,
        PredictionStatus.INCORRECT: 0,
        PredictionStatus.MISSING: 1,
        PredictionStatus.INVALID: 0,
    }
    assert report.by_family[QuestionKind.SPATIAL].total == 2
    assert report.by_predicate[SpatialPredicate.LEFT_OF].correct == 1
    assert report.by_predicate[SpatialPredicate.ABOVE].correct == 0
    assert len(report.diagnostics) == 1
    diagnostic = report.diagnostics[0]
    assert diagnostic.question_id == questions[1].question_id
    assert diagnostic.status is PredictionStatus.MISSING
    assert diagnostic.question_kind is QuestionKind.SPATIAL
    assert diagnostic.predicate is SpatialPredicate.ABOVE
    assert diagnostic.expected is False
    assert diagnostic.predicted is None
    assert diagnostic.evidence_frame_ids == (9,)
    assert diagnostic.evidence_interval_ids == ()
