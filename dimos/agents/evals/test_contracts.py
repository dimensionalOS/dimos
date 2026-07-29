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

"""Serialization contract of the three eval records.

The records are the join between a live run and everything that reads its
output later (the scorer, the renderer, a reviewer opening a shard months
after the run), and they travel as JSONL. These tests pin the three
properties that makes that safe: a record survives a round trip unchanged,
one record is exactly one line, and the outcome enum has a single definition.
"""

from __future__ import annotations

import json
from typing import get_args

from dimos.agents.evals.contracts import OUTCOMES, AnswerRecord, Outcome, QuestionSpec, ScoreResult

QUESTION = QuestionSpec(
    question_id="go2-bigoffice-liquor-shelf",
    display_name="liquor shelf",
    raw_label="bookstore",
    question_text="Where is the liquor shelf? Use your navigation tools to go to it.",
    ref_x=-0.825,
    ref_y=-1.85,
    ref_z=0.3,
    n_views=2,
    spread_m=0.0707,
    human_review="renamed",
    threshold_m=1.5,
)

ANSWER = AnswerRecord(
    question_id="go2-bigoffice-liquor-shelf",
    outcome="predicted",
    goal_x=-1.4,
    goal_y=-1.2,
    goal_yaw=0.31,
    n_goals=1,
    tool_invoked=True,
    tool_queries=["liquor shelf", "shelf with bottles"],
    model_id="gpt-5.6-luna",
    prompt_id="spatial",
    prompt_sha256="a" * 64,
    wall_time_s=3.25,
    error=None,
)

RESULT = ScoreResult(
    question_id="go2-bigoffice-liquor-shelf",
    passed=True,
    reason="map-frame XY error 0.879 m vs threshold 1.500 m",
    score=1.0,
    error_m=0.8791,
    outcome="predicted",
)


def test_outcome_literal_and_outcomes_tuple_agree() -> None:
    """The enum has one definition: the tuple is the ``Literal``, spelled out.

    ``OUTCOMES`` is what runtime code iterates and ``Outcome`` is what mypy
    checks, so a state added to one and not the other would type-check while
    silently never being handled.
    """
    assert get_args(Outcome) == OUTCOMES


def test_question_spec_round_trip() -> None:
    assert QuestionSpec.from_json(QUESTION.to_json()) == QUESTION


def test_answer_record_round_trip() -> None:
    assert AnswerRecord.from_json(ANSWER.to_json()) == ANSWER


def test_answer_record_round_trip_without_a_goal() -> None:
    """The no-prediction shape: every goal field is null and must stay null."""
    answer = AnswerRecord(
        question_id="go2-bigoffice-houseplant",
        outcome="no_prediction",
        goal_x=None,
        goal_y=None,
        goal_yaw=None,
        n_goals=0,
        tool_invoked=True,
        tool_queries=["houseplant"],
        model_id="openai:gpt-4o",
        prompt_id="plain",
        prompt_sha256="b" * 64,
        wall_time_s=2.5,
    )
    assert AnswerRecord.from_json(answer.to_json()) == answer


def test_score_result_round_trip() -> None:
    assert ScoreResult.from_json(RESULT.to_json()) == RESULT


def test_records_serialize_to_a_single_line() -> None:
    """Shards are JSONL, so a newline inside a record would corrupt the file."""
    question = QuestionSpec(
        question_id="q-multiline",
        display_name="two\nlines",
        raw_label="two\nlines",
        question_text="Where is the two\nlines?",
        ref_x=0.0,
        ref_y=0.0,
        ref_z=0.0,
        n_views=2,
        spread_m=0.0,
        human_review="verified",
    )
    assert "\n" not in question.to_json()
    assert QuestionSpec.from_json(question.to_json()) == question


def test_records_serialize_with_sorted_keys() -> None:
    """Key order is fixed so re-running a config produces a diffable shard."""
    for record in (QUESTION, ANSWER, RESULT):
        keys = list(json.loads(record.to_json()))
        assert keys == sorted(keys)
