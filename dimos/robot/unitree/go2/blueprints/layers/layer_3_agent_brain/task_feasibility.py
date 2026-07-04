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
from typing import Any

from dimos.agents.annotation import skill
from dimos.agents.skill_result import SkillResult
from dimos.core.module import Module
from dimos.robot.unitree.go2.blueprints.layers.layer_3_agent_brain.evolution_ledger import (
    EvolutionLedgerSpec,
)
from dimos.robot.unitree.go2.blueprints.layers.layer_5_skill_interface.skill_interface_spec import (
    SkillInterfaceSpec,
)


class _Go2TaskFeasibilityEvaluator(Module):
    """Rule-based Layer 3 preflight for user task feasibility."""

    _skill_interface: SkillInterfaceSpec | None = None
    _evolution_ledger: EvolutionLedgerSpec | None = None

    @skill
    def evaluate_task_feasibility(
        self,
        task: str,
        context_json: str = "{}",
    ) -> SkillResult:
        """Evaluate whether the current task is feasible before selecting physical skills.

        Args:
            task: User task or goal to evaluate.
            context_json: Optional JSON object from get_context metadata.
        """
        task = task.strip()
        if not task:
            return SkillResult.fail("INVALID_INPUT", "task is required")

        parsed_context = _parse_context_json(context_json)
        if isinstance(parsed_context, str):
            return SkillResult.fail("INVALID_INPUT", parsed_context)

        contracts, contract_warnings = self._skill_contracts()
        required_skills = _required_skills_for_task(task)
        available_skills = _available_required_skills(required_skills, contracts)
        missing_skills = [skill for skill in required_skills if skill not in available_skills]
        safety_risks = _safety_risks(task)
        missing_context = _missing_context(required_skills, contracts, parsed_context)

        if _is_motion_task(required_skills, contracts) and "robot_state" in missing_context:
            safety_risks.append("movement task requires current robot state")

        feasible = _feasible_value(required_skills, missing_skills, missing_context, safety_risks)
        recommended_next_action = _recommended_next_action(
            feasible=feasible,
            missing_context=missing_context,
            missing_skills=missing_skills,
            safety_risks=safety_risks,
        )
        clarifying_question = _clarifying_question(
            recommended_next_action=recommended_next_action,
            required_skills=required_skills,
            missing_skills=missing_skills,
        )
        evidence_sources = _evidence_sources(parsed_context)
        if contracts:
            evidence_sources.append("skill_interface")
            evidence_sources = _unique(evidence_sources)

        metadata: dict[str, Any] = {
            "feasible": feasible,
            "missing_context": missing_context,
            "required_skills": required_skills,
            "available_skills": available_skills,
            "safety_risks": safety_risks,
            "recommended_next_action": recommended_next_action,
            "clarifying_question": clarifying_question,
            "evidence_sources": evidence_sources,
            "missing_skills": missing_skills,
            "skill_interface_available": self._skill_interface is not None,
        }
        if contract_warnings:
            metadata["warnings"] = contract_warnings

        ledger_warning = self._record_feasibility_event(task=task, payload=metadata)
        if ledger_warning:
            metadata.setdefault("warnings", []).append(ledger_warning)

        message = (
            f"Task feasibility: {feasible}; next_action={recommended_next_action}; "
            f"required_skills={required_skills or ['unknown']}"
        )
        return SkillResult.ok(message, **metadata)

    def _skill_contracts(self) -> tuple[dict[str, dict[str, Any]], list[str]]:
        if self._skill_interface is None:
            return {}, []
        try:
            snapshot = self._skill_interface.get_skill_interface_snapshot()
        except Exception as exc:
            return {}, [f"skill_interface: {exc}"]
        contracts: dict[str, dict[str, Any]] = {}
        for item in snapshot.get("skills") or []:
            if isinstance(item, dict) and isinstance(item.get("skill_name"), str):
                contracts[item["skill_name"]] = item
        return contracts, []

    def _record_feasibility_event(self, task: str, payload: dict[str, Any]) -> str:
        if self._evolution_ledger is None:
            return ""
        try:
            self._evolution_ledger.write_evolution_event(
                event_type="task_feasibility",
                task=task,
                payload=payload,
                commit=False,
            )
        except Exception as exc:
            return f"evolution_ledger: {exc}"
        return ""


def _parse_context_json(context_json: str) -> dict[str, Any] | str:
    if not context_json.strip():
        return {}
    try:
        parsed = json.loads(context_json)
    except json.JSONDecodeError as exc:
        return f"context_json must be a JSON object: {exc.msg}"
    if not isinstance(parsed, dict):
        return "context_json must be a JSON object"
    metadata = parsed.get("metadata")
    if isinstance(metadata, dict):
        return metadata
    return parsed


def _required_skills_for_task(task: str) -> list[str]:
    text = task.casefold()
    if any(word in text for word in ("stop", "cancel", "halt")):
        return ["stop_navigation"]
    if any(word in text for word in ("say", "speak", "tell", "announce")):
        return ["speak"]
    if "follow" in text:
        return ["follow_person"]
    if any(word in text for word in ("patrol", "security")):
        return ["start_security_patrol" if "security" in text else "start_patrol"]
    if any(word in text for word in ("look for", "look out", "find", "search", "detect")):
        return ["look_out_for"]
    if any(word in text for word in ("walk to", "go to", "navigate", "kitchen", "room")):
        return ["navigate_with_text"]
    if any(word in text for word in ("move", "turn", "forward", "backward", "left", "right")):
        return ["relative_move"]
    if "wait" in text:
        return ["wait"]
    if "time" in text:
        return ["current_time"]
    return []


def _available_required_skills(
    required_skills: list[str],
    contracts: dict[str, dict[str, Any]],
) -> list[str]:
    if not contracts:
        return []
    return [skill for skill in required_skills if skill in contracts]


def _missing_context(
    required_skills: list[str],
    contracts: dict[str, dict[str, Any]],
    context: dict[str, Any],
) -> list[str]:
    missing: list[str] = []
    for skill_name in required_skills:
        contract = contracts.get(skill_name, {})
        if contract.get("requires_robot_state") and not _robot_state_available(context):
            missing.append("robot_state")
        if contract.get("requires_world_state") and not _world_state_available(context):
            missing.append("world_state")
        if contract.get("requires_context") and not context:
            missing.append("context_evidence")
    return _unique(missing)


def _robot_state_available(context: dict[str, Any]) -> bool:
    robot_state = context.get("robot_state")
    if not isinstance(robot_state, dict):
        return False
    for key in ("odom_available", "pose_available"):
        value = robot_state.get(key)
        if isinstance(value, bool):
            return value
    if robot_state.get("pose") and robot_state.get("pose") != "unavailable":
        return True
    if robot_state.get("odom"):
        return True
    return False


def _world_state_available(context: dict[str, Any]) -> bool:
    world_state = context.get("world_state")
    if isinstance(world_state, dict) and any(world_state.values()):
        return True
    sources = context.get("sources")
    if isinstance(sources, dict):
        return any(
            bool(sources.get(key))
            for key in ("spatial_memory", "temporal_memory", "semantic_temporal_map")
        )
    evidence = context.get("context_evidence")
    if isinstance(evidence, dict):
        selected = evidence.get("selected_sources")
        if isinstance(selected, list):
            return any(
                source in selected
                for source in ("spatial_memory", "temporal_memory", "structured_world_state")
            )
    return False


def _safety_risks(task: str) -> list[str]:
    text = task.casefold()
    unsafe_markers = (
        "disable safety",
        "turn off safety",
        "run into",
        "hit ",
        "harm",
        "crash into",
        "jump off",
        "attack",
    )
    if any(marker in text for marker in unsafe_markers):
        return ["task requests unsafe or harmful robot behavior"]
    return []


def _is_motion_task(
    required_skills: list[str],
    contracts: dict[str, dict[str, Any]],
) -> bool:
    for skill_name in required_skills:
        contract = contracts.get(skill_name, {})
        if contract.get("motion_sensitive"):
            return True
        if skill_name in {
            "navigate_with_text",
            "relative_move",
            "follow_person",
            "start_patrol",
            "start_security_patrol",
        }:
            return True
    return False


def _feasible_value(
    required_skills: list[str],
    missing_skills: list[str],
    missing_context: list[str],
    safety_risks: list[str],
) -> str:
    if any("unsafe or harmful" in risk for risk in safety_risks):
        return "no"
    if missing_skills:
        return "no"
    if not required_skills or missing_context or safety_risks:
        return "uncertain"
    return "yes"


def _recommended_next_action(
    feasible: str,
    missing_context: list[str],
    missing_skills: list[str],
    safety_risks: list[str],
) -> str:
    if any("unsafe or harmful" in risk for risk in safety_risks):
        return "refuse"
    if missing_context:
        return "get_context"
    if missing_skills or feasible == "uncertain":
        return "ask_clarifying_question"
    if feasible == "no":
        return "refuse"
    return "proceed"


def _clarifying_question(
    recommended_next_action: str,
    required_skills: list[str],
    missing_skills: list[str],
) -> str:
    if recommended_next_action != "ask_clarifying_question":
        return ""
    if not required_skills:
        return "Which concrete robot action or observation should I perform?"
    if missing_skills:
        return f"No current skill contract covers {missing_skills[0]}; what outcome do you need?"
    return "What extra context should I use before choosing a robot skill?"


def _evidence_sources(context: dict[str, Any]) -> list[str]:
    evidence = context.get("context_evidence")
    sources: list[str] = []
    if isinstance(evidence, dict):
        selected = evidence.get("selected_sources")
        if isinstance(selected, list):
            sources.extend(str(source) for source in selected if source)
        entries = evidence.get("entries")
        if isinstance(entries, list):
            for entry in entries:
                if isinstance(entry, dict) and entry.get("source"):
                    sources.append(str(entry["source"]))
    return _unique(sources)


def _unique(items: list[str]) -> list[str]:
    seen: set[str] = set()
    result: list[str] = []
    for item in items:
        if item and item not in seen:
            seen.add(item)
            result.append(item)
    return result


__all__ = ["_Go2TaskFeasibilityEvaluator"]
