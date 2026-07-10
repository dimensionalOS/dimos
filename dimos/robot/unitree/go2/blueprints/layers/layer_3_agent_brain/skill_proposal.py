#!/usr/bin/env python3
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
import re
from typing import Any

from dimos.agents.annotation import skill
from dimos.agents.skill_result import SkillResult
from dimos.core.module import Module
from dimos.robot.unitree.go2.blueprints.layers.layer_3_agent_brain.evolution_ledger import (
    EvolutionLedgerSpec,
)
from dimos.robot.unitree.go2.blueprints.layers.layer_3_agent_brain.evolution_proposal import (
    SKILL_PROPOSAL_SCHEMA,
    validate_skill_proposal,
)
from dimos.robot.unitree.go2.blueprints.layers.layer_3_agent_brain.skill_outcome_store import (
    SkillOutcomeStoreSpec,
)
from dimos.robot.unitree.go2.blueprints.layers.layer_5_skill_interface.skill_interface_spec import (
    SkillInterfaceSpec,
)

_SKILL_INTERFACE_TARGET = (
    "dimos/robot/unitree/go2/blueprints/layers/layer_5_skill_interface/skill_interface_registry.py"
)


class _Go2SkillProposalGenerator(Module):
    """Proposal-only generator for Go2 skill-interface improvements."""

    _skill_interface: SkillInterfaceSpec | None = None
    _skill_outcomes: SkillOutcomeStoreSpec | None = None
    _evolution_ledger: EvolutionLedgerSpec | None = None

    @skill
    def propose_skill_interface(
        self,
        task: str,
        failure_context_json: str = "{}",
    ) -> SkillResult:
        """Propose a new or revised skill interface without changing executable code.

        Args:
            task: User task or repeated failure scenario.
            failure_context_json: JSON object describing the observed skill gap.
        """
        task = task.strip()
        if not task:
            return SkillResult.fail("INVALID_INPUT", "task is required")

        failure_context = _parse_context(failure_context_json)
        if isinstance(failure_context, str):
            return SkillResult.fail("INVALID_INPUT", failure_context)

        contracts = self._skill_contracts()
        existing_matches = _existing_skill_matches(task, failure_context, contracts)
        if existing_matches:
            return SkillResult.ok(
                "Existing skill contract appears to cover this capability",
                proposal_created=False,
                proposal={},
                existing_skill_matches=existing_matches,
                recommended_next_action="use_existing_skill_contract",
            )

        misuse_reason = self._existing_skill_misuse_reason(failure_context)
        if misuse_reason:
            return SkillResult.ok(
                "No new skill proposal created; use the existing skill contract correctly",
                proposal_created=False,
                proposal={},
                existing_skill_matches=[],
                recommended_next_action="use_existing_skill_correctly",
                reason=misuse_reason,
            )

        if not _is_missing_capability(failure_context):
            return SkillResult.ok(
                "No skill proposal created; missing capability evidence is insufficient",
                proposal_created=False,
                proposal={},
                existing_skill_matches=[],
                recommended_next_action="collect_more_feedback",
            )

        proposal = _build_skill_proposal(task=task, failure_context=failure_context)
        errors = validate_skill_proposal(proposal)
        if errors:
            return SkillResult.fail("INVALID_INPUT", "; ".join(errors))

        metadata: dict[str, Any] = {
            "proposal_created": True,
            "proposal": proposal,
            "existing_skill_matches": [],
            "recommended_next_action": "review_proposal",
        }
        ledger_warning = self._record_proposal(proposal, metadata)
        if ledger_warning:
            metadata["warnings"] = [ledger_warning]
        return SkillResult.ok(f"Proposed skill interface {proposal['proposal_id']}", **metadata)

    def _skill_contracts(self) -> list[dict[str, Any]]:
        if self._skill_interface is None:
            return []
        try:
            snapshot = self._skill_interface.get_skill_interface_snapshot()
        except Exception:
            return []
        return [skill for skill in snapshot.get("skills") or [] if isinstance(skill, dict)]

    def _existing_skill_misuse_reason(self, failure_context: dict[str, Any]) -> str:
        failure_type = str(failure_context.get("failure_type") or "").strip()
        if failure_type in {"missing_argument", "missing_context", "invalid_argument"}:
            return failure_type
        if self._skill_outcomes is None:
            return ""
        try:
            outcomes = self._skill_outcomes.get_recent_outcomes(limit=5)
        except Exception:
            return ""
        misuse_markers = ("missing required argument", "invalid argument", "context required")
        repeated = [
            outcome
            for outcome in outcomes
            if any(
                marker in str(outcome.get("message") or "").casefold() for marker in misuse_markers
            )
            or outcome.get("error_code") == "INVALID_INPUT"
        ]
        if len(repeated) >= 2:
            return "repeated_existing_skill_misuse"
        return ""

    def _record_proposal(self, proposal: dict[str, Any], metadata: dict[str, Any]) -> str:
        if self._evolution_ledger is None:
            return ""
        record_skill_proposal = getattr(self._evolution_ledger, "record_skill_proposal", None)
        if record_skill_proposal is None:
            return "evolution_ledger: record_skill_proposal is unavailable"
        try:
            result = record_skill_proposal(proposal_json=json.dumps(proposal), commit=False)
        except Exception as exc:
            return f"evolution_ledger: {exc}"
        if not getattr(result, "success", False):
            return f"evolution_ledger: {getattr(result, 'message', 'proposal write failed')}"
        result_metadata = getattr(result, "metadata", {})
        if isinstance(result_metadata, dict) and result_metadata.get("proposal_path"):
            metadata["proposal_path"] = result_metadata["proposal_path"]
        return ""


def _parse_context(failure_context_json: str) -> dict[str, Any] | str:
    try:
        parsed = json.loads(failure_context_json or "{}")
    except json.JSONDecodeError as exc:
        return f"failure_context_json must be a JSON object: {exc.msg}"
    if not isinstance(parsed, dict):
        return "failure_context_json must be a JSON object"
    return parsed


def _is_missing_capability(failure_context: dict[str, Any]) -> bool:
    failure_type = str(failure_context.get("failure_type") or "").strip()
    if failure_type in {"skill_missing_capability", "missing_capability", "unknown_skill"}:
        return True
    message = str(failure_context.get("message") or "").casefold()
    return "missing capability" in message or "no skill" in message or "unknown skill" in message


def _existing_skill_matches(
    task: str,
    failure_context: dict[str, Any],
    contracts: list[dict[str, Any]],
) -> list[str]:
    capability = _capability_text(task, failure_context)
    capability_tokens = _tokens(capability)
    if not capability_tokens:
        return []
    matches: list[str] = []
    for contract in contracts:
        name = str(contract.get("skill_name") or "")
        searchable = " ".join(
            [
                name.replace("_", " "),
                str(contract.get("summary") or ""),
                str(contract.get("domain") or ""),
            ]
        )
        contract_tokens = _tokens(searchable)
        if capability_tokens.issubset(contract_tokens):
            matches.append(name)
    return sorted(match for match in matches if match)


def _build_skill_proposal(task: str, failure_context: dict[str, Any]) -> dict[str, Any]:
    capability = _capability_text(task, failure_context)
    skill_name = _skill_name_from_capability(capability)
    domain = str(failure_context.get("domain") or _infer_domain(task, capability)).strip()
    risk_class = _risk_class(domain)
    required_args = failure_context.get("required_args")
    if not isinstance(required_args, list) or not all(
        isinstance(arg, str) for arg in required_args
    ):
        required_args = ["target"]

    proposal = {
        "schema": SKILL_PROPOSAL_SCHEMA,
        "proposal_id": f"go2-skill-{skill_name}",
        "title": f"Add Go2 skill interface for {capability}",
        "summary": f"Repeated evidence suggests the current Go2 skill set cannot {capability}.",
        "risk_level": risk_class,
        "target_files": [_SKILL_INTERFACE_TARGET],
        "requires_human_review": True,
        "generated_patch": "",
        "task": task,
        "problem": str(
            failure_context.get("problem")
            or failure_context.get("message")
            or f"Missing capability: {capability}"
        ),
        "existing_skill_gaps": _existing_skill_gaps(failure_context, capability),
        "proposed_interface": {
            "skill_name": skill_name,
            "domain": domain,
            "required_args": required_args,
            "optional_args": failure_context.get("optional_args")
            if isinstance(failure_context.get("optional_args"), list)
            else [],
            "risk_class": risk_class,
            "recommended_preflight": _recommended_preflight(risk_class),
        },
        "validation_plan": [
            "Add a Layer 5 static contract first.",
            "Add unit tests for argument validation and MCP tool exposure.",
            "Validate in replay or simulation before hardware use.",
            "Require human review before adding executable robot code.",
        ],
    }
    return proposal


def _capability_text(task: str, failure_context: dict[str, Any]) -> str:
    return str(failure_context.get("capability") or task).strip()


def _skill_name_from_capability(capability: str) -> str:
    normalized = re.sub(r"[^a-zA-Z0-9]+", "_", capability.casefold()).strip("_")
    return normalized or "proposed_skill"


def _infer_domain(task: str, capability: str) -> str:
    text = f"{task} {capability}".casefold()
    if any(word in text for word in ("door", "grab", "pick", "place", "open")):
        return "manipulation"
    if any(word in text for word in ("walk", "navigate", "move", "follow")):
        return "navigation"
    if any(word in text for word in ("look", "detect", "recognize", "inspect")):
        return "perception"
    return "general"


def _risk_class(domain: str) -> str:
    if domain in {"manipulation", "robot_motion", "navigation", "security"}:
        return "high"
    if domain in {"perception", "person_follow"}:
        return "medium"
    return "low"


def _recommended_preflight(risk_class: str) -> list[str]:
    if risk_class == "high":
        return ["evaluate_task_feasibility", "get_context", "predict_skill_outcome"]
    if risk_class == "medium":
        return ["get_context", "predict_skill_outcome"]
    return ["route_task"]


def _existing_skill_gaps(failure_context: dict[str, Any], capability: str) -> list[str]:
    gaps = failure_context.get("existing_skill_gaps")
    if isinstance(gaps, list) and all(isinstance(gap, str) for gap in gaps):
        return gaps
    evidence = failure_context.get("evidence")
    if isinstance(evidence, list) and all(isinstance(item, str) for item in evidence):
        return evidence
    return [f"No existing skill contract covers capability: {capability}"]


def _tokens(value: str) -> set[str]:
    return {
        token[:-1] if token.endswith("s") and len(token) > 3 else token
        for token in re.findall(r"[a-z0-9]+", value.casefold())
    }


__all__ = ["_Go2SkillProposalGenerator"]
