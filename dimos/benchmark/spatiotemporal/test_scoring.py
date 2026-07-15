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

"""Tests for exact typed prediction scoring."""

import pytest

from dimos.benchmark.spatiotemporal.models import (
    OracleAnswer,
    Prediction,
    Question,
    QuestionKind,
    SpatialPredicate,
)
from dimos.benchmark.spatiotemporal.scoring import score_prediction
from dimos.benchmark.spatiotemporal.utilities import stable_question_id

QUESTION_ID = stable_question_id(
    episode_id="episode_1",
    object_ids=("obj_red", "obj_blue"),
    predicate=SpatialPredicate.LEFT_OF.value,
    question_kind=QuestionKind.SPATIAL.value,
)


def test_scores_exact_boolean_prediction_with_separate_oracle() -> None:
    question = Question(
        question_id=QUESTION_ID,
        episode_id="episode_1",
        text="Is the mug left of the laptop at the end?",
        question_kind=QuestionKind.SPATIAL,
        predicate=SpatialPredicate.LEFT_OF,
        object_ids=("obj_red", "obj_blue"),
    )
    oracle = OracleAnswer(
        question_id=question.question_id,
        expected=True,
        evidence_frame_ids=(12,),
    )
    prediction = Prediction(question_id=question.question_id, answer=True)

    result = score_prediction(question, oracle, prediction)

    assert result.question_id == question.question_id
    assert result.expected is True
    assert result.predicted is True
    assert result.correct is True


def test_scores_always_yes_incorrect_against_false_private_truth() -> None:
    question = Question(
        question_id=QUESTION_ID,
        episode_id="episode_1",
        text="Is the mug left of the laptop at the end?",
        question_kind=QuestionKind.SPATIAL,
        predicate=SpatialPredicate.LEFT_OF,
        object_ids=("obj_red", "obj_blue"),
    )
    oracle = OracleAnswer(
        question_id=question.question_id,
        expected=False,
        evidence_frame_ids=(12,),
    )

    result = score_prediction(
        question,
        oracle,
        Prediction(question_id=question.question_id, answer=True),
    )

    assert result.correct is False


def test_rejects_oracle_or_prediction_for_another_question() -> None:
    question_id = QUESTION_ID
    other_id = f"question_{'0' * 64}"
    question = Question(
        question_id=question_id,
        episode_id="episode_1",
        text="Is the mug left of the laptop at the end?",
        question_kind=QuestionKind.SPATIAL,
        predicate=SpatialPredicate.LEFT_OF,
        object_ids=("obj_red", "obj_blue"),
    )

    mismatched_records = (
        (
            OracleAnswer(question_id=other_id, expected=True, evidence_frame_ids=(12,)),
            Prediction(question_id=question_id, answer=True),
        ),
        (
            OracleAnswer(question_id=question_id, expected=True, evidence_frame_ids=(12,)),
            Prediction(question_id=other_id, answer=True),
        ),
    )
    for oracle, prediction in mismatched_records:
        with pytest.raises(ValueError, match="question IDs"):
            score_prediction(question, oracle, prediction)
