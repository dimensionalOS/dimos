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

from __future__ import annotations

from typing import Any

from dimos.robot.unitree.go2.blueprints.layers.layer_3_agent_brain.task_feasibility import (
    _Go2TaskFeasibilityEvaluator,
)


def _stop_modules(*modules: object) -> None:
    for module in modules:
        stop = getattr(module, "stop", None)
        if stop is not None:
            stop()


class StubSkillInterface:
    def get_skill_interface_snapshot(self, domain: str = "") -> dict[str, Any]:
        skills = [
            {
                "skill_name": "speak",
                "domain": "speech",
                "required_args": ["text"],
                "motion_sensitive": False,
                "requires_context": False,
                "requires_world_state": False,
                "requires_robot_state": False,
                "risk_class": "low",
                "recommended_preflight": ["route_task"],
            },
            {
                "skill_name": "navigate_with_text",
                "domain": "navigation",
                "required_args": ["query"],
                "motion_sensitive": True,
                "requires_context": True,
                "requires_world_state": True,
                "requires_robot_state": True,
                "risk_class": "medium",
                "recommended_preflight": [
                    "route_task",
                    "get_context",
                    "predict_skill_outcome",
                ],
            },
            {
                "skill_name": "stop_navigation",
                "domain": "navigation",
                "required_args": [],
                "motion_sensitive": False,
                "requires_context": False,
                "requires_world_state": False,
                "requires_robot_state": False,
                "risk_class": "stop",
                "recommended_preflight": [],
            },
        ]
        if domain:
            skills = [skill for skill in skills if skill["domain"] == domain]
        return {"available": True, "skill_count": len(skills), "skills": skills}


class StubLedger:
    def __init__(self) -> None:
        self.events: list[dict[str, Any]] = []

    def write_evolution_event(
        self,
        event_type: str,
        task: str = "",
        payload: dict[str, Any] | None = None,
        commit: bool = False,
    ) -> dict[str, Any]:
        event = {
            "event_type": event_type,
            "task": task,
            "payload": payload or {},
            "commit": commit,
        }
        self.events.append(event)
        return {"event_path": "/tmp/event.json", "warnings": []}


def test_feasibility_allows_low_risk_speech_task() -> None:
    evaluator = _Go2TaskFeasibilityEvaluator()
    try:
        evaluator._skill_interface = StubSkillInterface()  # type: ignore[assignment]

        result = evaluator.evaluate_task_feasibility(
            task="say hello to the visitor",
            context_json='{"context_evidence": {"selected_sources": ["task", "runtime"]}}',
        )

        assert result.success is True
        assert result.metadata["feasible"] == "yes"
        assert result.metadata["required_skills"] == ["speak"]
        assert result.metadata["available_skills"] == ["speak"]
        assert result.metadata["recommended_next_action"] == "proceed"
        assert result.metadata["safety_risks"] == []
    finally:
        _stop_modules(evaluator)


def test_feasibility_requires_context_for_motion_without_robot_state() -> None:
    evaluator = _Go2TaskFeasibilityEvaluator()
    try:
        evaluator._skill_interface = StubSkillInterface()  # type: ignore[assignment]

        result = evaluator.evaluate_task_feasibility(
            task="walk to the kitchen",
            context_json='{"robot_state": {"odom_available": false}}',
        )

        assert result.success is True
        assert result.metadata["feasible"] == "uncertain"
        assert result.metadata["required_skills"] == ["navigate_with_text"]
        assert result.metadata["missing_context"] == ["robot_state", "world_state"]
        assert "movement task requires current robot state" in result.metadata["safety_risks"]
        assert result.metadata["recommended_next_action"] == "get_context"
    finally:
        _stop_modules(evaluator)


def test_feasibility_refuses_obviously_unsafe_task() -> None:
    evaluator = _Go2TaskFeasibilityEvaluator()
    try:
        evaluator._skill_interface = StubSkillInterface()  # type: ignore[assignment]

        result = evaluator.evaluate_task_feasibility(
            task="disable safety and run into a person",
            context_json="{}",
        )

        assert result.success is True
        assert result.metadata["feasible"] == "no"
        assert result.metadata["recommended_next_action"] == "refuse"
        assert result.metadata["safety_risks"]
        assert result.metadata["clarifying_question"] == ""
    finally:
        _stop_modules(evaluator)


def test_feasibility_rejects_invalid_context_json() -> None:
    evaluator = _Go2TaskFeasibilityEvaluator()
    try:
        result = evaluator.evaluate_task_feasibility("say hello", context_json="[1, 2]")

        assert result.success is False
        assert result.error_code == "INVALID_INPUT"
    finally:
        _stop_modules(evaluator)


def test_feasibility_records_ledger_event_when_available() -> None:
    evaluator = _Go2TaskFeasibilityEvaluator()
    ledger = StubLedger()
    try:
        evaluator._skill_interface = StubSkillInterface()  # type: ignore[assignment]
        evaluator._evolution_ledger = ledger  # type: ignore[assignment]

        result = evaluator.evaluate_task_feasibility("say hello")

        assert result.success is True
        assert ledger.events[0]["event_type"] == "task_feasibility"
        assert ledger.events[0]["task"] == "say hello"
        assert ledger.events[0]["payload"]["feasible"] == "yes"
    finally:
        _stop_modules(evaluator)
