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

"""Scoring semantics, on synthetic observations.

The scorer is where the eval's judgement lives, and it is pure, so all of it
is exercised here without a robot, a model or a store: the six-state
classification ladder, the map-frame XY distance, what stays in the pass-rate
denominator, and the shard format the renderer reads back.
"""

from __future__ import annotations

from dataclasses import replace
import hashlib
import json
import math
from pathlib import Path
from typing import Any

import pytest

from dimos.agents.evals.contracts import (
    AnswerRecord,
    Outcome,
    QuestionSpec,
    RetrievalRecord,
    ScoreResult,
)
from dimos.agents.evals.scorer import (
    GoalRecord,
    RunObservation,
    append_shard,
    broken_count,
    build_answer_record,
    classify_outcome,
    errors_m,
    no_prediction_rate,
    pass_rate,
    prompt_sha256,
    read_shard,
    read_shards,
    score,
)

PROMPT = "You are a mobile robot assistant."

#: One sweep's stamp. Every record of one run carries it; a *second* run of the
#: same configuration carries a different one, which is what keeps the two apart.
RUN_ID = "20260730T091500-4242"

#: What the memory returned for the agent's query, re-asked after the turn. Pure
#: observability: the tests below assert it survives into the shard *and* that
#: nothing about the verdict moves when it is there.
RETRIEVALS = [
    RetrievalRecord(query="liquor shelf", x=1.0, y=0.0, distance=0.7412),
    RetrievalRecord(query="shelf with bottles", x=1.2, y=-0.3, distance=0.8004),
]


def make_question(question_id: str = "q1", **overrides: Any) -> QuestionSpec:
    """A question at the origin, 1.5 m threshold, unless overridden."""
    fields: dict[str, Any] = {
        "question_id": question_id,
        "display_name": "liquor shelf",
        "raw_label": "bookstore",
        "question_text": "Where is the liquor shelf? Use your navigation tools to go to it.",
        "ref_x": 0.0,
        "ref_y": 0.0,
        "ref_z": 0.0,
        "n_views": 2,
        "spread_m": 0.1,
        "human_review": "renamed",
        "threshold_m": 1.5,
    }
    fields.update(overrides)
    return QuestionSpec(**fields)


def make_observation(**overrides: Any) -> RunObservation:
    """A finished run that set no goal, unless overridden."""
    fields: dict[str, Any] = {
        "question_id": "q1",
        "model_id": "gpt-5.6-luna",
        "prompt_id": "spatial",
        "prompt_sha256": prompt_sha256(PROMPT),
        "run_id": RUN_ID,
        "finished": True,
        "agent_wall_time_s": 3.0,
        "total_wall_time_s": 27.0,
    }
    fields.update(overrides)
    return RunObservation(**fields)


def goal(x: float, y: float, yaw: float = 0.0) -> GoalRecord:
    """A map-frame goal as ``navigate_with_text`` emits them: ``z`` always 0."""
    return GoalRecord(x=x, y=y, z=0.0, yaw=yaw)


def make_answer(question_id: str = "q1", outcome: Outcome = "predicted", **kw: Any) -> AnswerRecord:
    fields: dict[str, Any] = {
        "question_id": question_id,
        "outcome": outcome,
        "goal_x": 1.0,
        "goal_y": 0.0,
        "goal_yaw": 0.0,
        "n_goals": 1,
        "tool_invoked": True,
        "tool_queries": ["liquor shelf"],
        "model_id": "gpt-5.6-luna",
        "prompt_id": "spatial",
        "prompt_sha256": prompt_sha256(PROMPT),
        "run_id": RUN_ID,
        "wall_time_s": 3.0,
        "error": None,
    }
    fields.update(kw)
    return AnswerRecord(**fields)


# --- classification ---------------------------------------------------------


def test_goal_record_from_the_rpc_mapping() -> None:
    """`RecordingStubNavigation.get_goals()` hands back plain dicts over RPC."""
    record = GoalRecord.from_mapping(
        {"x": 1.5, "y": -2.0, "z": 0.0, "yaw": 0.25, "frame_id": "map"}
    )
    assert record == GoalRecord(x=1.5, y=-2.0, z=0.0, yaw=0.25, frame_id="map")
    assert GoalRecord.from_mapping({"x": 0, "y": 0, "z": 0, "yaw": 0}).frame_id is None


@pytest.mark.parametrize(
    ("observation", "expected"),
    [
        (make_observation(), "no_prediction"),
        (make_observation(goals=[goal(1.0, 0.0)]), "predicted"),
        (make_observation(goals=[goal(1.0, 0.0), goal(2.0, 0.0)]), "multiple_predictions"),
        (
            make_observation(goals=[goal(1.0, 0.0)], tool_errors=["RuntimeError: boom"]),
            "tool_error",
        ),
        (make_observation(finished=False), "answer_timeout"),
        (make_observation(harness_error="RuntimeError: build failed"), "harness_error"),
    ],
    ids=["none", "one", "two", "tool_raised", "unfinished", "harness_broke"],
)
def test_classify_covers_every_outcome(observation: RunObservation, expected: Outcome) -> None:
    assert classify_outcome(observation) == expected


def test_classification_ladder_puts_broken_measurements_first() -> None:
    """A run that is several things at once is reported as the most invalidating.

    Peeling the reasons off one at a time walks the whole ladder: as long as
    the harness is broken nothing about the agent is known, an unfinished turn
    outranks a raising tool, and a raising tool outranks the goals it left
    behind.
    """
    broken = dict(
        goals=[goal(1.0, 0.0), goal(2.0, 0.0)],
        tool_errors=["RuntimeError: boom"],
        finished=False,
        harness_error="RuntimeError: teardown failed",
    )
    assert classify_outcome(make_observation(**broken)) == "harness_error"
    assert classify_outcome(make_observation(**{**broken, "harness_error": None})) == (
        "answer_timeout"
    )
    assert (
        classify_outcome(make_observation(**{**broken, "harness_error": None, "finished": True}))
        == "tool_error"
    )
    assert (
        classify_outcome(
            make_observation(
                **{**broken, "harness_error": None, "finished": True, "tool_errors": []}
            )
        )
        == "multiple_predictions"
    )


# --- answer records ---------------------------------------------------------


def test_answer_record_carries_the_first_goal_and_the_run_configuration() -> None:
    answer = build_answer_record(
        make_observation(goals=[goal(1.25, -0.5, yaw=0.75)], tool_queries=["liquor shelf"])
    )
    assert (answer.goal_x, answer.goal_y, answer.goal_yaw) == (1.25, -0.5, 0.75)
    assert answer.n_goals == 1
    assert answer.tool_invoked is True
    assert answer.tool_queries == ["liquor shelf"]
    assert answer.model_id == "gpt-5.6-luna"
    assert answer.prompt_sha256 == prompt_sha256(PROMPT)
    assert answer.error is None


def test_answer_record_keeps_the_evidence_for_multiple_predictions() -> None:
    """Both goals have to survive into the shard, not just the count."""
    answer = build_answer_record(
        make_observation(goals=[goal(1.0, 2.0), goal(-3.5, 4.25)], tool_queries=["shelf", "shelf"])
    )
    assert answer.outcome == "multiple_predictions"
    assert answer.n_goals == 2
    assert answer.goal_x == 1.0
    assert answer.error is not None
    assert "1.000, 2.000" in answer.error
    assert "-3.500, 4.250" in answer.error


def test_answer_record_reports_the_turn_not_the_harness_cost() -> None:
    """Building the coordinator loads CLIP; that is not model latency."""
    answer = build_answer_record(
        make_observation(goals=[goal(1.0, 0.0)], agent_wall_time_s=3.0, total_wall_time_s=27.0)
    )
    assert answer.wall_time_s == 3.0


def test_answer_record_records_a_tool_call_that_produced_no_goal() -> None:
    """The similarity gate rejecting a query is the expected no-prediction path."""
    answer = build_answer_record(make_observation(tool_queries=["a place that is not here"]))
    assert answer.outcome == "no_prediction"
    assert answer.tool_invoked is True
    assert (answer.goal_x, answer.goal_y, answer.n_goals) == (None, None, 0)
    assert answer.error is None


def test_answer_record_carries_the_retrievals_through_untouched() -> None:
    """What the memory returned is observability, and the shard has to hold it.

    The scored answer channel is ``(x, y, yaw)``, which cannot tell "found the
    right object" from "found a neighbour inside the pass radius" -- the failure
    the confusability gate in ``questions.py`` has to prevent by dropping
    questions. Recording the retrieval is what would make identity-level scoring
    possible later, so it has to reach the shard now.
    """
    answer = build_answer_record(
        make_observation(
            goals=[goal(1.0, 0.0)],
            tool_queries=["liquor shelf", "shelf with bottles"],
            retrievals=RETRIEVALS,
        )
    )
    assert answer.retrievals == RETRIEVALS
    assert [record.query for record in answer.retrievals] == answer.tool_queries


def test_retrievals_change_nothing_about_the_outcome_or_the_score() -> None:
    """The diagnostic is inert: same observation, same verdict, retrievals or not.

    Asserted as an equality between two whole records rather than field by field,
    so a future scoring rule that started reading ``retrievals`` -- turning a
    diagnostic into a pass criterion by accident -- fails here.
    """
    question = make_question()
    bare = make_observation(goals=[goal(0.3, 0.4)], tool_queries=["liquor shelf"])
    observed = make_observation(
        goals=[goal(0.3, 0.4)], tool_queries=["liquor shelf"], retrievals=RETRIEVALS
    )

    assert classify_outcome(bare) == classify_outcome(observed)

    bare_answer, observed_answer = build_answer_record(bare), build_answer_record(observed)
    assert observed_answer.retrievals == RETRIEVALS
    assert bare_answer == replace(observed_answer, retrievals=[])
    assert score(question, bare_answer) == score(question, observed_answer)

    with_retrievals = [score(question, observed_answer)]
    without = [score(question, bare_answer)]
    assert (pass_rate(with_retrievals), no_prediction_rate(with_retrievals)) == (
        pass_rate(without),
        no_prediction_rate(without),
    )
    assert errors_m(with_retrievals) == errors_m(without)


def test_answer_record_explains_a_timeout_and_a_tool_error() -> None:
    timed_out = build_answer_record(make_observation(finished=False, agent_wall_time_s=180.0))
    assert timed_out.error is not None
    assert "180.0s" in timed_out.error

    raised = build_answer_record(make_observation(tool_errors=["RuntimeError: boom", "OSError: x"]))
    assert raised.error == "RuntimeError: boom; OSError: x"


# --- scoring ----------------------------------------------------------------


def test_score_measures_map_frame_xy_distance() -> None:
    result = score(make_question(), make_answer(goal_x=0.3, goal_y=0.4))
    assert result.passed is True
    assert result.score == 1.0
    assert result.error_m == pytest.approx(0.5)
    assert result.outcome == "predicted"
    assert "0.500" in result.reason
    assert "1.500" in result.reason


def test_score_ignores_the_reference_height() -> None:
    """Goals always have ``z == 0``, so a 3-D distance would punish shelves.

    The goal here is 1.0 m away in XY and would be 3.16 m away in 3-D: only
    the XY reading is inside the 1.5 m threshold.
    """
    question = make_question(ref_z=3.0)
    result = score(question, make_answer(goal_x=1.0, goal_y=0.0))
    assert result.error_m == pytest.approx(1.0)
    assert result.passed is True
    assert math.dist((1.0, 0.0, 0.0), (0.0, 0.0, 3.0)) > question.threshold_m


def test_score_threshold_is_inclusive() -> None:
    result = score(make_question(threshold_m=1.5), make_answer(goal_x=1.5, goal_y=0.0))
    assert result.error_m == pytest.approx(1.5)
    assert result.passed is True


def test_score_fails_beyond_the_threshold_but_keeps_the_measurement() -> None:
    result = score(make_question(threshold_m=1.5), make_answer(goal_x=4.0, goal_y=3.0))
    assert result.passed is False
    assert result.score == 0.0
    assert result.error_m == pytest.approx(5.0)


@pytest.mark.parametrize(
    "outcome",
    ["no_prediction", "multiple_predictions", "answer_timeout", "tool_error", "harness_error"],
)
def test_score_of_anything_but_a_single_prediction_has_no_error(outcome: Outcome) -> None:
    """Not measurable is not zero and not infinite -- it is ``None``."""
    result = score(make_question(), make_answer(outcome=outcome, goal_x=None, goal_y=None))
    assert result.passed is False
    assert result.score == 0.0
    assert result.error_m is None
    assert result.outcome == outcome
    assert result.reason == outcome


def test_score_reason_repeats_the_answer_record_detail() -> None:
    answer = make_answer(
        outcome="multiple_predictions",
        goal_x=None,
        goal_y=None,
        error="2 goals recorded: (1.000, 2.000); (-3.500, 4.250)",
    )
    result = score(make_question(), answer)
    assert (
        result.reason == "multiple_predictions: 2 goals recorded: (1.000, 2.000); (-3.500, 4.250)"
    )


def test_score_refuses_to_pair_records_from_different_questions() -> None:
    with pytest.raises(ValueError, match="paired by question_id"):
        score(make_question("q1"), make_answer("q2"))


def test_score_refuses_a_prediction_without_a_goal() -> None:
    """A 'predicted' answer with no goal means the record was hand-assembled."""
    with pytest.raises(ValueError, match="carries no goal"):
        score(make_question(), make_answer(outcome="predicted", goal_x=None, goal_y=None))


# --- aggregates -------------------------------------------------------------


def scored(outcome: Outcome, passed: bool = False, error_m: float | None = None) -> ScoreResult:
    return ScoreResult(
        question_id="q",
        passed=passed,
        reason=outcome,
        score=1.0 if passed else 0.0,
        error_m=error_m,
        outcome=outcome,
    )


def test_pass_rate_keeps_unanswered_questions_in_the_denominator() -> None:
    """An agent that answers nothing is not thereby perfect."""
    results = [
        scored("predicted", passed=True, error_m=0.4),
        scored("no_prediction"),
        scored("no_prediction"),
        scored("predicted", error_m=6.1),
    ]
    assert pass_rate(results) == pytest.approx(0.25)
    assert no_prediction_rate(results) == pytest.approx(0.5)
    assert broken_count(results) == 0


def test_broken_measurements_leave_the_denominator_entirely() -> None:
    """A harness that died is not a model that failed.

    The same two agent-attributable results are scored twice: once alone, once
    with three broken measurements appended. If the broken ones stayed in the
    denominator, a run whose worker crashed would report half the pass rate of
    an identical run that happened not to crash -- and the shard would look
    like evidence about the model.
    """
    agent = [scored("predicted", passed=True, error_m=0.4), scored("no_prediction")]
    with_broken = [
        *agent,
        scored("harness_error"),
        scored("answer_timeout"),
        scored("tool_error"),
    ]

    assert pass_rate(agent) == pytest.approx(0.5)
    assert pass_rate(with_broken) == pytest.approx(0.5)
    assert no_prediction_rate(with_broken) == pytest.approx(0.5)
    assert broken_count(with_broken) == 3


def test_multiple_predictions_stays_on_the_agents_side_of_the_split() -> None:
    """Setting two goals is a choice the agent made, not a broken measurement."""
    results = [scored("predicted", passed=True, error_m=0.4), scored("multiple_predictions")]
    assert pass_rate(results) == pytest.approx(0.5)
    assert broken_count(results) == 0


def test_rates_of_nothing_are_zero() -> None:
    assert pass_rate([]) == 0.0
    assert no_prediction_rate([]) == 0.0
    assert broken_count([]) == 0


def test_rates_of_nothing_but_broken_measurements_are_zero() -> None:
    """An empty denominator is 0.0, not a crash and not an invented number."""
    results = [scored("harness_error"), scored("tool_error")]
    assert pass_rate(results) == 0.0
    assert no_prediction_rate(results) == 0.0
    assert broken_count(results) == 2


def test_errors_m_is_the_plotted_sample_in_input_order() -> None:
    results = [
        scored("predicted", passed=True, error_m=0.4),
        scored("harness_error"),
        scored("predicted", error_m=6.1),
        scored("no_prediction"),
    ]
    assert errors_m(results) == [0.4, 6.1]


def test_prompt_sha256_hashes_the_text_and_treats_no_prompt_as_empty() -> None:
    assert prompt_sha256(PROMPT) == hashlib.sha256(PROMPT.encode("utf-8")).hexdigest()
    assert prompt_sha256(None) == hashlib.sha256(b"").hexdigest()


# --- shards -----------------------------------------------------------------


def test_shard_round_trips_pairs_in_file_order(tmp_path: Path) -> None:
    path = tmp_path / "nested" / "gpt-5-6-luna__spatial.jsonl"
    first = make_answer("q1", goal_x=0.3, goal_y=0.4)
    second = make_answer("q2", outcome="no_prediction", goal_x=None, goal_y=None, n_goals=0)
    append_shard(path, first, score(make_question("q1"), first))
    append_shard(path, second, score(make_question("q2"), second))

    cases = read_shard(path)
    assert [case.answer.question_id for case in cases] == ["q1", "q2"]
    assert cases[0].answer == first
    assert cases[0].result.error_m == pytest.approx(0.5)
    assert cases[1].result.outcome == "no_prediction"


def test_shard_round_trips_the_retrievals_as_records(tmp_path: Path) -> None:
    """``read_shard`` has to rebuild the nested list, not hand back dicts.

    ``append_shard`` writes ``retrievals`` as JSON objects, and the read path
    constructs the record from that dict; without ``AnswerRecord.from_dict`` the
    round trip would come back with dicts inside -- unequal to what was written,
    and a ``.query`` attribute short of what the schema promises. The smoke lane
    asserts ``cases[0].answer == answer``, so this is exactly the equality it
    rests on.
    """
    path = tmp_path / "gpt-5-6-luna__spatial.jsonl"
    answer = make_answer("q1", goal_x=0.3, goal_y=0.4, retrievals=RETRIEVALS)
    append_shard(path, answer, score(make_question("q1"), answer))

    (case,) = read_shard(path)
    assert case.answer == answer
    assert case.answer.retrievals == RETRIEVALS
    assert all(isinstance(record, RetrievalRecord) for record in case.answer.retrievals)


def test_append_shard_refuses_to_pair_records_from_different_questions(tmp_path: Path) -> None:
    path = tmp_path / "shard.jsonl"
    with pytest.raises(ValueError, match="different questions"):
        append_shard(path, make_answer("q1"), score(make_question("q2"), make_answer("q2")))
    assert not path.exists()


def test_read_shard_rejects_an_unknown_record_kind(tmp_path: Path) -> None:
    path = tmp_path / "shard.jsonl"
    path.write_text(json.dumps({"kind": "note", "data": {}}) + "\n")
    with pytest.raises(ValueError, match="unknown record kind 'note'"):
        read_shard(path)


def test_read_shard_rejects_a_half_written_pair(tmp_path: Path) -> None:
    """A crashed run leaves an answer with no score; a plot with holes hides it."""
    path = tmp_path / "shard.jsonl"
    append_shard(path, make_answer("q1"), score(make_question("q1"), make_answer("q1")))
    path.write_text(
        path.read_text()
        + json.dumps({"kind": "answer", "data": json.loads(make_answer("q2").to_json())})
        + "\n"
    )
    with pytest.raises(ValueError, match=r"unpaired records for question\(s\) \['q2'\]"):
        read_shard(path)


def test_read_shard_rejects_a_duplicated_question(tmp_path: Path) -> None:
    """Two runs appended to one file must fail loudly, not silently keep the last."""
    path = tmp_path / "shard.jsonl"
    answer = make_answer("q1")
    append_shard(path, answer, score(make_question("q1"), answer))
    append_shard(path, answer, score(make_question("q1"), answer))
    with pytest.raises(ValueError, match="duplicate answer for question 'q1'"):
        read_shard(path)


def test_read_shards_concatenates_in_the_order_given(tmp_path: Path) -> None:
    paths = []
    for name, question_id in (("a.jsonl", "q1"), ("b.jsonl", "q2")):
        path = tmp_path / name
        answer = make_answer(question_id)
        append_shard(path, answer, score(make_question(question_id), answer))
        paths.append(path)

    assert [case.answer.question_id for case in read_shards(reversed(paths))] == ["q2", "q1"]


def test_read_shards_rejects_the_same_run_twice_across_files(tmp_path: Path) -> None:
    """One run's records may appear once, however many files they are spread over.

    A case that died and was restarted leaves a half-written shard next to the
    new one, both tagged with the same ``run_id``. Concatenating them would count
    those questions twice inside one run -- a figure that is wrong in the one way
    nobody can see by looking at it -- so it is refused.
    """
    first = tmp_path / f"gpt-5-6-luna__spatial__{RUN_ID}.jsonl"
    second = tmp_path / f"gpt-5-6-luna__spatial__{RUN_ID}-retry.jsonl"
    for path in (first, second):
        answer = make_answer("q1")
        append_shard(path, answer, score(make_question("q1"), answer))

    with pytest.raises(ValueError, match="both hold question 'q1'"):
        read_shards([first, second])


def test_read_shards_keeps_two_runs_of_one_configuration(tmp_path: Path) -> None:
    """Repeating a configuration is a measurement, not a mistake.

    The same question under the same (model, prompt) is exactly what a second
    sweep produces, and averaging over repeats is how a stochastic agent gets
    measured. Only the ``run_id`` distinguishes the records, so it is what the
    duplicate key rests on -- and the renderer aggregates both into one row.
    """
    paths = []
    for run_id in (RUN_ID, "20260730T104500-4243"):
        path = tmp_path / f"gpt-5-6-luna__spatial__{run_id}.jsonl"
        answer = make_answer("q1", run_id=run_id)
        append_shard(path, answer, score(make_question("q1"), answer))
        paths.append(path)

    assert [case.answer.run_id for case in read_shards(paths)] == [
        RUN_ID,
        "20260730T104500-4243",
    ]


def test_read_shards_keeps_configurations_apart(tmp_path: Path) -> None:
    """The same question under two configurations is two measurements, not a clash."""
    paths = []
    for name, prompt_id in (("a.jsonl", "shipping"), ("b.jsonl", "spatial")):
        path = tmp_path / name
        answer = make_answer("q1", prompt_id=prompt_id)
        append_shard(path, answer, score(make_question("q1"), answer))
        paths.append(path)

    assert [case.answer.prompt_id for case in read_shards(paths)] == ["shipping", "spatial"]
