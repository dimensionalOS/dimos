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

from pathlib import Path
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
    read_agent_messages,
    summarize,
    write_report,
)


class FakeSkills:
    def __init__(self, pick: str | None, *, found: bool = True) -> None:
        self._pick = pick  # None: the pick returns only once the operator is asked
        self._found = found
        self.reasons: list[str] = []
        self.release = threading.Event()

    def scan_objects(self, prompts: list[str]) -> SkillResult:
        objects = [{"object_id": "7", "name": "apple"}] if self._found else []
        return SkillResult(f"Found {len(objects)}", metadata={"objects": objects})

    def pick_object(self, object_id: str) -> SkillResult:
        if self._pick is None:
            self.release.wait(5.0)
        return SkillResult(self._pick or "Move to grasp candidate 0 did not complete; ABORTED")

    def request_operator(self, reason: str) -> SkillResult:
        self.reasons.append(reason)
        self.release.set()
        return SkillResult("Operator hold is on")


def test_trial_plan_is_eight_clean_and_four_of_each_failure() -> None:
    counts = {case: TRIAL_PLAN.count(case) for case in Case}
    assert counts == {Case.CLEAN: 8, Case.MOVED: 4, Case.REMOVED: 4, Case.MISSED_GRASP: 4}


@pytest.mark.parametrize(
    ("pick", "found", "reason"),
    [
        ("Pick complete", True, None),
        ("Closed on air.", True, "pick of apple failed: Closed on air."),
        ("Pick complete", False, "pick of apple failed: the scan found no apple (Found 0)"),
        (None, True, "pick of apple did not finish within 0 s"),  # the hold stops the stuck pick
    ],
)
def test_scripted_asks_for_help_only_when_the_pick_fails(
    pick: str | None, found: bool, reason: str | None
) -> None:
    skills = FakeSkills(pick, found=found)

    attempt = ScriptedDecisionMaker(skills, pick_timeout_s=0.3, grace_s=5.0).attempt("apple")

    assert skills.reasons == ([reason] if reason else [])
    assert attempt.requested == bool(reason)
    assert attempt.pick_message and attempt.t_pick_result is not None


def test_agent_messages_give_the_pick_result_and_the_hold_request() -> None:
    pick = {"name": "pick_object", "args": {"object_id": "7"}, "id": "a"}
    ask = {"name": "request_operator", "args": {"reason": "grasp missed"}, "id": "b"}
    messages = [
        (1.0, AIMessage(content="", tool_calls=[pick])),
        (2.0, ToolMessage(content='{"message": "Closed on air."}', tool_call_id="a")),
        (3.0, AIMessage(content="", tool_calls=[ask])),
        (4.0, ToolMessage(content='{"message": "Operator hold is on"}', tool_call_id="b")),
        (5.0, AIMessage(content="I asked a person for help.")),
    ]

    attempt = read_agent_messages(messages)

    assert (attempt.pick_message, attempt.t_pick_result) == ("Closed on air.", 2.0)
    assert attempt.requested
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


def _trial(n: int, held: bool, inject: float | None, pick: float, hold: float | None) -> Trial:
    attempt = Attempt(pick_message="Pick complete" if held else "missed", t_pick_result=pick)
    case = Case.CLEAN if inject is None else Case.MOVED
    return Trial(n, case, "scripted", held, attempt, t_inject=inject, t_hold=hold)


def test_summarize_and_report_give_the_ticket_checks(tmp_path: Path) -> None:
    trials = [
        _trial(1, False, 0.0, 10.0, 11.0),  # stuck from the later of injection and pick: 10
        _trial(2, False, 12.0, 10.0, 13.0),  # stuck from the injection at 12
        _trial(3, False, 0.0, 10.0, 50.0),
        _trial(4, False, 0.0, 10.0, None),  # stuck, no hold
        _trial(5, True, None, 10.0, None),
        _trial(6, True, None, 10.0, 12.0),  # a hold on a good pick
    ]

    checks = [(c.value, c.passed) for c in summarize(trials)]
    markdown = write_report(trials, tmp_path / "report", "Commit abc123")

    assert checks == [
        ("4 of 6", True),  # failures injected
        ("3 of 4 (75%)", False),  # stuck trials that got a hold
        ("1 of 2 (50%)", False),  # good trials that got a hold
        ("1.0 s", True),  # median stuck to hold: 1, 1, 40
        ("11.0 s", True),  # median injection to hold: 11, 1, 50
    ]
    rows = (tmp_path / "report.csv").read_text().splitlines()
    assert len(rows) == 7
    assert rows[1].startswith("1,moved mid-pick,,scripted,stuck,missed,yes,,,1.0,11.0")
    assert "Commit abc123" in markdown
    assert "| 3 of 4 (75%) | no |" in markdown
