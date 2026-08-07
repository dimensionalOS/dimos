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

"""Serialization contract of the eval records.

The records are the join between a live run and everything that reads its
output later (the scorer, the renderer, a reviewer opening a shard months
after the run), and they travel as JSONL. These tests pin the three
properties that makes that safe: a record survives a round trip unchanged,
one record is exactly one line, and the outcome enum has a single definition.

``AnswerRecord`` is the one with a nested member (``retrievals``), so the round
trip is asserted to rebuild it as records rather than dicts, and to still read a
line written before the field existed.
"""

from __future__ import annotations

import json
from typing import get_args

import pytest

from dimos.agents.evals.contracts import (
    OUTCOMES,
    AnswerRecord,
    Outcome,
    QuestionSpec,
    RetrievalRecord,
    ScoreResult,
)

#: The committed "stack of storage boxes" question: asked under its reviewed
#: name, but identified by the detector label the reviewer cannot move.
QUESTION = QuestionSpec(
    question_id="go2-bigoffice-organization",
    display_name="stack of storage boxes",
    raw_label="organization",
    question_text="Where is the stack of storage boxes? Use your navigation tools to go to it.",
    ref_x=-3.875,
    ref_y=0.425,
    ref_z=0.375,
    n_views=2,
    spread_m=0.4924,
    human_review="renamed",
    threshold_m=1.9961,
)

ANSWER = AnswerRecord(
    question_id="go2-bigoffice-organization",
    outcome="predicted",
    goal_x=-3.4899,
    goal_y=-1.3669,
    goal_yaw=0.31,
    n_goals=1,
    tool_invoked=True,
    tool_queries=["stack of storage boxes", "storage boxes"],
    model_id="gpt-5.6-luna",
    prompt_id="spatial",
    prompt_sha256="a" * 64,
    run_id="20260730T091500-4242",
    wall_time_s=3.25,
    error=None,
    retrievals=[
        RetrievalRecord(query="stack of storage boxes", x=-3.4899, y=-1.3669, distance=0.7412),
        RetrievalRecord(query="storage boxes", x=-3.4899, y=-1.3669, distance=0.7508),
    ],
)

RESULT = ScoreResult(
    question_id="go2-bigoffice-organization",
    passed=True,
    reason="map-frame XY error 1.833 m vs threshold 1.996 m",
    score=1.0,
    error_m=1.8328,
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


def test_a_question_cannot_be_built_without_its_threshold() -> None:
    """There is no default pass radius, because there is no default viewing distance.

    Every threshold is derived from the question's own nearest teacher viewpoint
    (``questions.threshold_for``). A class default would let a spec assembled by
    hand -- in a test, or in an experiment someone runs next year -- be scored
    under a radius nobody derived for it, and nothing downstream could tell.
    """
    fields = json.loads(QUESTION.to_json())
    del fields["threshold_m"]
    with pytest.raises(TypeError, match="threshold_m"):
        QuestionSpec(**fields)


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
        model_id="openai:gpt-5.6-sol",
        prompt_id="shipping",
        prompt_sha256="b" * 64,
        run_id="20260730T091500-4242",
        wall_time_s=2.5,
    )
    assert AnswerRecord.from_json(answer.to_json()) == answer


def test_answer_record_carries_the_run_it_was_measured_in() -> None:
    """Two runs of one configuration are two measurements, not a duplicate.

    ``run_id`` is what tells them apart in a shard directory: without it the
    scorer cannot distinguish a deliberately repeated sweep -- the way a
    stochastic agent gets measured -- from the same run read twice, and it has to
    reject one of the two cases.
    """
    repeat = AnswerRecord.from_dict(
        {**json.loads(ANSWER.to_json()), "run_id": "20260730T104500-4243"}
    )
    assert repeat.run_id != ANSWER.run_id
    assert AnswerRecord.from_json(repeat.to_json()) == repeat


def test_answer_record_rebuilds_its_retrievals_instead_of_leaving_them_dicts() -> None:
    """The nested list is why there is a ``from_dict`` at all.

    ``asdict`` flattens ``retrievals`` into plain dicts, so the naive
    ``AnswerRecord(**decoded)`` produces a record that *looks* fine, compares
    unequal to the one that was written, and hands a reader a dict where the
    schema promises a ``RetrievalRecord``. Both read paths (``from_json`` here,
    ``scorer.read_shard``) go through ``from_dict`` so that cannot happen on only
    the path nobody tested.
    """
    decoded = json.loads(ANSWER.to_json())
    assert decoded["retrievals"] == [
        {"distance": 0.7412, "query": "stack of storage boxes", "x": -3.4899, "y": -1.3669},
        {"distance": 0.7508, "query": "storage boxes", "x": -3.4899, "y": -1.3669},
    ]

    rebuilt = AnswerRecord.from_dict(decoded)
    assert rebuilt == ANSWER
    assert all(isinstance(item, RetrievalRecord) for item in rebuilt.retrievals)
    assert rebuilt.retrievals[0].query == "stack of storage boxes"
    # The mistake `from_dict` exists to prevent, spelled out.
    assert AnswerRecord(**decoded) != ANSWER


def test_answer_record_from_a_line_written_before_retrievals_existed() -> None:
    """Shards predating the field stay readable, and read as "nothing recorded".

    Back-compat is by omission rather than by migration: ``retrievals`` defaults
    to empty, and an absent key and an empty list mean the same thing -- this run
    recorded no retrieval -- so there is nothing a stricter reader could usefully
    reject. An empty list is likewise not evidence the agent retrieved nothing:
    it may never have called the tool.
    """
    fields = json.loads(ANSWER.to_json())
    del fields["retrievals"]
    old_line = json.dumps(fields, sort_keys=True)

    restored = AnswerRecord.from_json(old_line)
    assert restored.retrievals == []
    assert restored == AnswerRecord.from_dict({**fields, "retrievals": []})


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
        threshold_m=1.4,
    )
    assert "\n" not in question.to_json()
    assert QuestionSpec.from_json(question.to_json()) == question


def test_records_serialize_with_sorted_keys() -> None:
    """Key order is fixed so re-running a config produces a diffable shard."""
    for record in (QUESTION, ANSWER, RESULT):
        keys = list(json.loads(record.to_json()))
        assert keys == sorted(keys)
