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

import json
from typing import Any

from dimos.robot.unitree.go2.blueprints.layers.layer_3_agent_brain.evolution_proposal import (
    SKILL_PROPOSAL_SCHEMA,
)
from dimos.robot.unitree.go2.blueprints.layers.layer_3_agent_brain.skill_proposal import (
    _Go2SkillProposalGenerator,
)


def _stop_modules(*modules: object) -> None:
    for module in modules:
        stop = getattr(module, "stop", None)
        if stop is not None:
            stop()


class StubSkillInterface:
    def __init__(self, skills: list[dict[str, Any]] | None = None) -> None:
        self._skills = skills or [
            {
                "skill_name": "navigate_with_text",
                "domain": "navigation",
                "summary": "Navigate to a tagged or visible location.",
                "required_args": ["query"],
                "optional_args": [],
                "risk_class": "medium",
                "recommended_preflight": ["get_context", "predict_skill_outcome"],
            }
        ]

    def get_skill_interface_snapshot(self, domain: str = "") -> dict[str, Any]:
        skills = self._skills
        if domain:
            skills = [skill for skill in skills if skill["domain"] == domain]
        return {"available": True, "skill_count": len(skills), "skills": skills}


class StubSkillOutcomes:
    def get_recent_outcomes(
        self, limit: int = 5, skill_name: str = "", domain: str = ""
    ) -> list[dict[str, Any]]:
        return [
            {
                "skill_name": "navigate_with_text",
                "success": False,
                "error_code": "INVALID_INPUT",
                "message": "missing required argument: query",
            },
            {
                "skill_name": "navigate_with_text",
                "success": False,
                "error_code": "INVALID_INPUT",
                "message": "missing required argument: query",
            },
        ][:limit]


class StubLedger:
    def __init__(self) -> None:
        self.proposals: list[dict[str, Any]] = []

    def record_skill_proposal(self, proposal_json: str, commit: bool = False) -> Any:
        proposal = json.loads(proposal_json)
        self.proposals.append(proposal)
        return type(
            "Result",
            (),
            {
                "success": True,
                "metadata": {"proposal_path": "/tmp/proposal.json"},
                "message": "ok",
            },
        )()


def test_missing_argument_failures_do_not_create_new_skill_proposal() -> None:
    generator = _Go2SkillProposalGenerator()
    try:
        generator._skill_interface = StubSkillInterface()  # type: ignore[assignment]
        generator._skill_outcomes = StubSkillOutcomes()  # type: ignore[assignment]

        result = generator.propose_skill_interface(
            task="go to the kitchen",
            failure_context_json='{"failure_type": "missing_argument"}',
        )

        assert result.success is True
        assert result.metadata["proposal_created"] is False
        assert result.metadata["recommended_next_action"] == "use_existing_skill_correctly"
        assert result.metadata["proposal"] == {}
    finally:
        _stop_modules(generator)


def test_missing_capability_feedback_creates_reviewable_proposal() -> None:
    generator = _Go2SkillProposalGenerator()
    ledger = StubLedger()
    try:
        generator._skill_interface = StubSkillInterface()  # type: ignore[assignment]
        generator._evolution_ledger = ledger  # type: ignore[assignment]

        result = generator.propose_skill_interface(
            task="open the lab door",
            failure_context_json=(
                '{"failure_type": "skill_missing_capability", '
                '"capability": "open doors", "domain": "manipulation"}'
            ),
        )

        assert result.success is True
        assert result.metadata["proposal_created"] is True
        proposal = result.metadata["proposal"]
        assert proposal["schema"] == SKILL_PROPOSAL_SCHEMA
        assert proposal["requires_human_review"] is True
        assert proposal["proposed_interface"]["skill_name"] == "open_doors"
        assert ledger.proposals[0]["proposal_id"] == proposal["proposal_id"]
        assert result.metadata["proposal_path"] == "/tmp/proposal.json"
    finally:
        _stop_modules(generator)


def test_skill_proposal_always_requires_human_review() -> None:
    generator = _Go2SkillProposalGenerator()
    try:
        result = generator.propose_skill_interface(
            task="open the lab door",
            failure_context_json='{"failure_type": "skill_missing_capability", "capability": "open doors"}',
        )

        assert result.success is True
        assert result.metadata["proposal"]["requires_human_review"] is True
    finally:
        _stop_modules(generator)


def test_existing_contract_prevents_duplicate_skill_proposal() -> None:
    generator = _Go2SkillProposalGenerator()
    try:
        generator._skill_interface = StubSkillInterface(
            [
                {
                    "skill_name": "open_door",
                    "domain": "manipulation",
                    "summary": "Open doors with the robot manipulator.",
                    "required_args": ["door_id"],
                    "risk_class": "high",
                }
            ]
        )  # type: ignore[assignment]

        result = generator.propose_skill_interface(
            task="open the lab door",
            failure_context_json='{"failure_type": "skill_missing_capability", "capability": "open doors"}',
        )

        assert result.success is True
        assert result.metadata["proposal_created"] is False
        assert result.metadata["recommended_next_action"] == "use_existing_skill_contract"
        assert result.metadata["existing_skill_matches"] == ["open_door"]
    finally:
        _stop_modules(generator)
