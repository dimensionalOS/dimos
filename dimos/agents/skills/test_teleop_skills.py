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

"""TeleopSkills: what request_operator asks the coordinator for, and what never asks for the hold."""

from __future__ import annotations

import asyncio
from collections.abc import Iterator
from typing import Any
from unittest.mock import MagicMock

import pytest

from dimos.agents.capabilities import CapabilityRegistry
from dimos.agents.mcp.mcp_server import app, handle_request
from dimos.agents.skill_result import SkillResult
from dimos.agents.skills.teleop_skills import TeleopSkills
from dimos.control.coordinator import ControlCoordinator
from dimos.control.tasks.operator_hold_task.operator_hold_spec import OperatorHoldSpec
from dimos.control.tasks.operator_hold_task.operator_hold_task import OperatorHoldStatus
from dimos.core.module import Module
from dimos.manipulation.manipulation_skills import ManipulationSkills
from dimos.manipulation.manipulation_spec import (
    ExecutionResult,
    ExecutionStatus,
    PlanningGroupInfo,
    PlanResult,
    PlanStatus,
)
from dimos.spec.utils import spec_annotation_compliance


class FakeCoordinator:
    """Records every task_invoke call and answers a hold request like the real task."""

    def __init__(self) -> None:
        self.calls: list[tuple[str, str, dict[str, Any] | None]] = []
        self.has_hold_task = True

    def task_invoke(self, task_name: str, method: str, kwargs: dict[str, Any] | None = None) -> Any:
        self.calls.append((task_name, method, kwargs))
        if not self.has_hold_task:
            return None  # what the coordinator returns for an unknown task
        return OperatorHoldStatus(on=True, route="agent", reason=(kwargs or {}).get("reason", ""))


@pytest.fixture
def coordinator() -> FakeCoordinator:
    return FakeCoordinator()


@pytest.fixture
def teleop(coordinator: FakeCoordinator) -> Iterator[TeleopSkills]:
    skills = TeleopSkills()
    skills._coordinator = coordinator
    yield skills
    skills.stop()


def test_the_spec_matches_the_coordinator() -> None:
    assert spec_annotation_compliance(ControlCoordinator, OperatorHoldSpec)


def test_request_operator_asks_for_the_hold_with_the_agent_route(
    teleop: TeleopSkills, coordinator: FakeCoordinator
) -> None:
    result = teleop.request_operator("box slipped")

    assert coordinator.calls == [
        ("operator_hold", "request", {"route": "agent", "reason": "box slipped"})
    ]
    assert isinstance(result, SkillResult)
    assert "hold is on" in result.message
    assert "box slipped" in result.message


def test_request_operator_raises_when_the_robot_has_no_hold_task(
    teleop: TeleopSkills, coordinator: FakeCoordinator
) -> None:
    coordinator.has_hold_task = False

    with pytest.raises(RuntimeError, match="no operator hold task"):
        teleop.request_operator("box slipped")


def test_request_operator_is_the_only_skill_and_waits_on_nothing(teleop: TeleopSkills) -> None:
    # No acknowledge or release skill: only the operator ends a hold. No
    # capability either, so a running move cannot make the request wait.
    assert [(s.func_name, s.uses) for s in teleop.get_skills()] == [("request_operator", ())]


def _failing_manipulation(failure: str) -> MagicMock:
    manipulation = MagicMock()
    manipulation.list_planning_groups.return_value = (
        PlanningGroupInfo(
            id="arm", joint_names=("j1",), base_frame="world", tip_frame="tcp", has_gripper=False
        ),
    )
    manipulation.plan_to_joints.return_value = PlanResult(PlanStatus.SUCCEEDED)
    manipulation.execute.return_value = ExecutionResult(ExecutionStatus.COMPLETED)
    if failure == "planner raises":
        manipulation.plan_to_joints.side_effect = RuntimeError("planner crashed")
    elif failure == "no plan":
        manipulation.plan_to_joints.return_value = PlanResult(PlanStatus.FAILED, "no path")
    else:
        manipulation.execute.return_value = ExecutionResult(ExecutionStatus.FAULT, "arm fault")
    return manipulation


def _call_tool(
    modules: list[Module], name: str, arguments: dict[str, Any], monkeypatch: pytest.MonkeyPatch
) -> dict[str, Any]:
    """Call one skill through the MCP server, as an agent would."""
    skills = [info for module in modules for info in module.get_skills()]
    rpc_calls = {
        info.func_name: getattr(module, info.func_name)
        for module in modules
        for info in module.get_skills()
    }
    monkeypatch.setattr(app.state, "skills_by_name", {s.func_name: s for s in skills})
    monkeypatch.setattr(app.state, "cap_registry", CapabilityRegistry())
    request = {"method": "tools/call", "id": 1, "params": {"name": name, "arguments": arguments}}
    response = asyncio.run(handle_request(request, skills, rpc_calls))
    assert response is not None
    return response


@pytest.mark.parametrize("failure", ["planner raises", "no plan", "execution fault"])
def test_a_failing_manipulation_skill_does_not_request_the_hold(
    teleop: TeleopSkills,
    coordinator: FakeCoordinator,
    monkeypatch: pytest.MonkeyPatch,
    failure: str,
) -> None:
    manipulation = ManipulationSkills()
    manipulation.manipulation = _failing_manipulation(failure)
    try:
        response = _call_tool(
            [manipulation, teleop], "move_to_joints", {"joints": "0.5"}, monkeypatch
        )
    finally:
        manipulation.stop()

    text = response["result"]["content"][0]["text"]
    assert any(word in text for word in ("planner crashed", "no path", "arm fault"))
    assert coordinator.calls == []
