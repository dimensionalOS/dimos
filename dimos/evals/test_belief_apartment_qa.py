# Copyright 2026 Dimensional Inc.
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

"""Grader smoke for belief_apartment_qa (no live DimSim)."""

import pytest

from dimos.evals.suites.belief_apartment_qa import SUITE
from dimos.evals.types import AgentInfo, FinalMetrics, Outcome, RunExtra, Step, Trajectory


def _outcome(answer: str) -> Outcome:
    return Outcome(
        trajectory=Trajectory(
            agent=AgentInfo(name="test", version="1", model_name="test"),
            steps=(Step(step_id=1, timestamp="", source="agent", message=answer),),
            final_metrics=FinalMetrics(
                total_prompt_tokens=0,
                total_completion_tokens=0,
                total_cached_tokens=0,
                total_cost_usd=0,
                total_steps=1,
            ),
            extra=RunExtra(ended_by="answer"),
        ),
        artifacts={},
    )


def _grade(case_id: str, answer: str) -> float:
    case = next(c for c in SUITE if c.id == case_id)
    return case.grade(_outcome(answer))


@pytest.mark.parametrize(
    "answer,score",
    [
        ("yes", 1),
        (" YES ", 1),
        ("Yes, there is a microwave.", 1),
        ("no", 0),
        ("unknown", 0),
        ("true", 0),
    ],
)
def test_boolean_yes_case(answer: str, score: float) -> None:
    assert _grade("belief_q051_microwave_in_kitchen", answer) == score


@pytest.mark.parametrize(
    "answer,score",
    [("no", 1), ("NO", 1), ("yes", 0), ("unknown", 0)],
)
def test_boolean_no_case(answer: str, score: float) -> None:
    assert _grade("belief_q118_television_off", answer) == score


@pytest.mark.parametrize(
    "answer,score",
    [("D", 1), ("d", 0), ("A", 0), ("Bedroom", 0), ("", 0)],
)
def test_letter_choice(answer: str, score: float) -> None:
    assert _grade("belief_q049_laptop_room", answer) == score


@pytest.mark.parametrize(
    "answer,score",
    [("4", 1), ("There are 4 chairs.", 1), ("3", 0), ("unknown", 0)],
)
def test_count_case(answer: str, score: float) -> None:
    assert _grade("belief_q074_chairs_near_table", answer) == score


@pytest.mark.parametrize(
    "answer,score",
    [("different", 1), ("DIFFERENT", 1), ("same", 0), ("no", 0)],
)
def test_same_different(answer: str, score: float) -> None:
    assert _grade("belief_q081_chair_vs_bedside_count", answer) == score


def test_suite_conventions() -> None:
    assert len(SUITE) >= 30
    ids = [c.id for c in SUITE]
    assert len(ids) == len(set(ids))
    assert not any(c.id.endswith("table_not_blocking") for c in SUITE)
    for case in SUITE:
        assert case.environment.config.scene == "apartment"
        assert "Return only" in case.inputs or "return only" in case.inputs.lower()
        assert "Move around to gather the evidence" in case.inputs
        assert case.timeout_s >= 60.0
