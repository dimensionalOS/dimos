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

"""Handoff validation: when the decision-makers ask for help, and the ticket's numbers."""

from __future__ import annotations

import threading

from langchain_core.messages import AIMessage, ToolMessage
import pytest

from dimos.agents.skill_result import SkillResult
from dimos.manipulation.evals.handoff_validation import (
    TRIAL_PLAN,
    AgentDecisionMaker,
    Attempt,
    Case,
    ScriptedDecisionMaker,
    Trial,
    hold_reason,
    read_agent_messages,
    render_csv,
    render_markdown,
    summarize,
)


class FakeSkills:
    def __init__(self, pick: str | None, *, found: bool = True) -> None:
        self._pick = pick  # None: the pick never returns
        self._found = found
        self.reasons: list[str] = []
        self.release = threading.Event()

    def scan_objects(self, prompts: list[str]) -> SkillResult:
        objects = [{"object_id": "7", "name": "apple"}] if self._found else []
        return SkillResult(f"Detected {len(objects)} object(s)", metadata={"objects": objects})

    def pick_object(self, object_id: str) -> SkillResult:
        assert object_id == "7"
        if self._pick is None:
            self.release.wait(5.0)
            return SkillResult(
                "Move to grasp candidate 0 did not complete; execution returned ABORTED"
            )
        return SkillResult(self._pick)

    def request_operator(self, reason: str) -> SkillResult:
        self.reasons.append(reason)
        self.release.set()
        return SkillResult("Operator hold is on")


def test_trial_plan_is_eight_clean_and_four_of_each_failure() -> None:
    assert len(TRIAL_PLAN) == 20
    assert {case: TRIAL_PLAN.count(case) for case in Case} == {
        Case.CLEAN: 8,
        Case.MOVED: 4,
        Case.REMOVED: 4,
        Case.MISSED_GRASP: 4,
    }


def test_hold_reason_is_none_only_for_a_complete_pick() -> None:
    assert hold_reason("apple", "Pick complete", 180.0) is None
    assert hold_reason("apple", "Closed the gripper on object 7.", 180.0) == (
        "pick of apple failed: Closed the gripper on object 7."
    )
    assert hold_reason("apple", None, 180.0) == "pick of apple did not finish within 180 s"


def test_scripted_asks_for_help_when_the_pick_fails() -> None:
    skills = FakeSkills("Closed the gripper on object 7. Final gripper position 0.02.")

    attempt = ScriptedDecisionMaker(skills, skills).attempt("apple")

    assert attempt.requested
    assert skills.reasons == [
        "pick of apple failed: Closed the gripper on object 7. Final gripper position 0.02."
    ]
    assert attempt.t_pick_result is not None


def test_scripted_never_asks_for_help_after_a_complete_pick() -> None:
    skills = FakeSkills("Pick complete")

    attempt = ScriptedDecisionMaker(skills, skills).attempt("apple")

    assert not attempt.requested
    assert skills.reasons == []
    assert attempt.pick_message == "Pick complete"


def test_scripted_asks_for_help_when_the_scan_finds_nothing() -> None:
    skills = FakeSkills("Pick complete", found=False)

    attempt = ScriptedDecisionMaker(skills, skills).attempt("apple")

    assert skills.reasons == [
        "pick of apple failed: the scan found no apple (Detected 0 object(s))"
    ]
    assert attempt.pick_message.startswith("the scan found no apple")


def test_scripted_asks_for_help_when_the_pick_runs_too_long() -> None:
    skills = FakeSkills(None)

    attempt = ScriptedDecisionMaker(skills, skills, pick_timeout_s=0.1, grace_s=5.0).attempt(
        "apple"
    )

    assert skills.reasons == ["pick of apple did not finish within 0 s"]
    # The hold stops the arm; the pick's own failure is still recorded.
    assert attempt.pick_message.endswith("ABORTED")


def test_agent_messages_give_the_pick_result_and_the_hold_request() -> None:
    messages = [
        (
            1.0,
            AIMessage(
                content="",
                tool_calls=[{"name": "pick_object", "args": {"object_id": "7"}, "id": "a"}],
            ),
        ),
        (
            2.0,
            ToolMessage(content='{"message": "Closed the gripper on object 7."}', tool_call_id="a"),
        ),
        (
            3.0,
            AIMessage(
                content="",
                tool_calls=[
                    {"name": "request_operator", "args": {"reason": "grasp missed"}, "id": "b"}
                ],
            ),
        ),
        (4.0, ToolMessage(content='{"message": "Operator hold is on"}', tool_call_id="b")),
        (5.0, AIMessage(content="I asked a person for help.")),
    ]

    attempt = read_agent_messages(messages)

    assert attempt.pick_message == "Closed the gripper on object 7."
    assert attempt.t_pick_result == 2.0
    assert attempt.requested
    assert attempt.reason == "grasp missed"
    assert attempt.log[-1] == "agent says: I asked a person for help."


def test_agent_attempt_waits_for_the_agent_to_go_idle() -> None:
    def send(text: str) -> None:
        agent.on_idle(False)
        agent.on_idle(True)
        # The reply can trail the idle flag; it still counts.
        threading.Timer(0.05, agent.on_message, [AIMessage(content="done")]).start()

    agent = AgentDecisionMaker(send, timeout_s=1.0)
    agent.straggler_s = 0.3

    assert agent.ask("Reply with ready") == "done"
    assert not agent.attempt("apple").requested


def _trial(
    number: int,
    *,
    held: bool,
    t_inject: float | None = None,
    t_pick: float | None = None,
    t_hold: float | None = None,
) -> Trial:
    return Trial(
        number=number,
        case=Case.CLEAN if t_inject is None else Case.MOVED,
        decision_maker="scripted",
        held=held,
        attempt=Attempt(pick_message="Pick complete" if held else "missed", t_pick_result=t_pick),
        t_start=0.0,
        t_inject=t_inject,
        t_hold=t_hold,
        route="agent" if t_hold is not None else "",
    )


def test_time_to_hold_counts_from_the_later_of_injection_and_failure() -> None:
    trial = _trial(1, held=False, t_inject=10.0, t_pick=25.0, t_hold=27.0)

    assert trial.stuck_to_hold_s == pytest.approx(2.0)
    assert trial.inject_to_hold_s == pytest.approx(17.0)
    assert _trial(2, held=True, t_pick=5.0).stuck_to_hold_s is None


def test_summarize_reports_the_ticket_checks() -> None:
    trials = [
        _trial(1, held=False, t_inject=0.0, t_pick=10.0, t_hold=11.0),
        _trial(2, held=False, t_inject=0.0, t_pick=10.0, t_hold=13.0),
        _trial(3, held=False, t_inject=0.0, t_pick=10.0, t_hold=50.0),
        _trial(4, held=False, t_inject=0.0, t_pick=10.0),  # stuck, no hold
        _trial(5, held=True, t_pick=10.0),
        _trial(6, held=True, t_pick=10.0, t_hold=12.0),  # a hold on a good pick
    ]

    checks = {check.name: check for check in summarize(trials)}

    injected = checks["At least half the trials had a failure injected"]
    assert (injected.value, injected.passed) == ("4 of 6", True)
    stuck = checks["At least 90% of stuck trials ended in a hold request"]
    assert (stuck.value, stuck.passed) == ("3 of 4 (75%)", False)
    good = checks["At most 10% of successful trials got a hold request"]
    assert (good.value, good.passed) == ("1 of 2 (50%)", False)
    median = checks["Median time from stuck to hold is 30 s or less"]
    assert (median.value, median.passed) == ("3.0 s", True)
    from_inject = checks["Same, counted from the moment the failure was injected"]
    assert (from_inject.value, from_inject.passed) == ("13.0 s", True)


def test_reports_have_one_row_per_trial() -> None:
    trials = [_trial(1, held=False, t_inject=0.0, t_pick=10.0, t_hold=11.0), _trial(2, held=True)]

    csv_text = render_csv(trials)
    markdown = render_markdown(trials, {"Commit": "abc123"})

    assert csv_text.splitlines()[1].startswith("1,moved mid-pick,,scripted,stuck,missed,yes,agent")
    assert len(csv_text.splitlines()) == 3
    assert "- Commit: abc123" in markdown
    assert "| 2 | clean | held the object | Pick complete | no |" in markdown
