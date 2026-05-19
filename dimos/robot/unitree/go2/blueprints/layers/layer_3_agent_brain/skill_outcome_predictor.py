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
from dimos.robot.unitree.go2.blueprints.layers.layer_3_agent_brain.skill_outcome_store import (
    SkillOutcomeStoreSpec,
)


class _Go2SkillOutcomePredictor(Module):
    """Rule-based Layer 3 predictor for pre-skill risk checks.

    What "prediction" means in this version:
        This is not a trained model and does not estimate a numeric success
        probability. It is a deterministic risk gate. It checks planned skill
        arguments, text context from ContextProvider, and recent outcomes from
        SkillOutcomeStore, then returns low/medium/high risk plus reasons.

    Why this is in Layer 3:
        It helps the LLM agent decide whether to proceed, gather more context,
        or choose a recovery tool before calling a physical skill.
    """

    # Optional Spec injection. If the store is present in the blueprint,
    # autoconnect wires this ref to _Go2SkillOutcomeStore. If not present, the
    # predictor still works using only args_json and context.
    _skill_outcomes: SkillOutcomeStoreSpec | None = None

    @skill
    def predict_skill_outcome(
        self, skill_name: str, args_json: str = "", context: str = ""
    ) -> SkillResult:
        """Predict whether a skill is likely to succeed in the current context.

        This tool performs a conservative preflight risk check. It does not
        execute robot actions. Use it before navigation, perception, following,
        or recovery-sensitive tools when the current context is uncertain.

        Args:
            skill_name: Name of the skill or MCP tool the agent is considering.
            args_json: Optional JSON object string containing planned arguments.
            context: Optional context summary from get_context or route_task.
        """
        skill_name = skill_name.strip()
        context = context.strip()
        if not skill_name:
            return SkillResult.fail("INVALID_INPUT", "skill_name is required")

        # MCP skill schemas do not handle arbitrary nested dict parameters as
        # consistently as strings. Accept JSON text, parse it here, and reject
        # non-object payloads. Example: '{"query": "kitchen"}'.
        parsed_args = _parse_args_json(args_json)
        if isinstance(parsed_args, str):
            return SkillResult.fail("INVALID_INPUT", parsed_args)

        # Inputs to the V2 predictor:
        # 1. skill_name: which tool the agent wants to call.
        # 2. parsed_args: planned tool arguments.
        # 3. context: text summary from get_context/route_task/dialogue.
        # 4. recent: same-skill outcomes from SkillOutcomeStore.
        recent = self._recent_outcomes(skill_name)
        reasons = _risk_reasons(skill_name, parsed_args, context, recent)
        risk = _risk_level(reasons)
        predicted_success = risk != "high"
        suggestions = _recovery_suggestions(skill_name, reasons)

        metadata = {
            "skill_name": skill_name,
            "args": parsed_args,
            "risk": risk,
            "predicted_success": predicted_success,
            "failure_reasons": reasons,
            "recovery_suggestions": suggestions,
            "recent_outcomes": recent,
            "outcome_store_available": self._skill_outcomes is not None,
        }
        message = (
            f"Predicted risk={risk}, predicted_success={predicted_success}. "
            f"Reasons: {reasons or ['no blocking risk detected']}"
        )
        return SkillResult(success=True, message=message, metadata=metadata)

    def _recent_outcomes(self, skill_name: str) -> list[dict[str, Any]]:
        """Fetch recent same-skill outcomes if the outcome store is wired."""
        if self._skill_outcomes is None:
            return []
        return self._skill_outcomes.get_recent_outcomes(limit=5, skill_name=skill_name)


def _parse_args_json(args_json: str) -> dict[str, Any] | str:
    """Parse the planned skill arguments.

    Returns a dict on success and an error string on failure. Keeping errors as
    strings avoids exceptions bubbling through the MCP skill response path.
    """
    args_json = args_json.strip()
    if not args_json:
        return {}
    try:
        parsed = json.loads(args_json)
    except json.JSONDecodeError as exc:
        return f"args_json must be a JSON object: {exc}"
    if not isinstance(parsed, dict):
        return "args_json must be a JSON object"
    return parsed


def _risk_reasons(
    skill_name: str,
    args: dict[str, Any],
    context: str,
    recent: list[dict[str, Any]],
) -> list[str]:
    """Collect human-readable risk reasons.

    The rules are intentionally conservative:
    - repeated recent failures increase risk even if arguments look valid;
    - movement-sensitive skills need context/odom;
    - semantic navigation needs a query and benefits from spatial memory;
    - person following needs a query and benefits from memory/perception context.
    """
    name = skill_name.casefold()
    context_text = context.casefold()
    reasons: list[str] = []

    # Recent history is the only "learned" signal in this version. It is still
    # rule-based: one failure is medium concern; repeated failures become high
    # risk in _risk_level.
    failures = [outcome for outcome in recent if not outcome.get("success", False)]
    if len(failures) >= 2:
        reasons.append("same skill failed repeatedly in recent outcomes")
    elif len(failures) == 1:
        reasons.append("same skill has one recent failure")

    if name in {"navigate_with_text", "relative_move", "follow_person"}:
        # ContextProvider formats missing odom as "Robot pose: unavailable".
        # Some structured summaries may instead include an odom=false-style
        # marker, so keep the broader fallback check.
        odom_unavailable = "odom" in context_text and "false" in context_text
        if "robot pose: unavailable" in context_text or odom_unavailable:
            reasons.append("robot pose or odometry may be unavailable")
        if not context:
            reasons.append("no context provided for movement-sensitive skill")

    if name == "navigate_with_text":
        # Navigation without a target query is a hard blocker. Missing or empty
        # spatial memory is not always fatal, but it should push the agent to
        # gather context or use visual perception first.
        query = str(args.get("query", "")).strip()
        if not query:
            reasons.append("navigation query is missing")
        if "spatial memory: unavailable" in context_text:
            reasons.append("spatial memory is unavailable for semantic navigation")
        if "no relevant matches" in context_text:
            reasons.append("spatial memory has no relevant matches")

    if name == "follow_person":
        # Person following is visual, but a concrete query still matters. Memory
        # availability is used as a weak risk signal when the target identity is
        # likely ambiguous.
        query = str(args.get("query", "")).strip()
        if not query:
            reasons.append("person-follow query is missing")
        no_memory_context = (
            "temporal memory: unavailable" in context_text
            and "spatial memory: unavailable" in context_text
        )
        if no_memory_context:
            reasons.append("no memory source is available to help identify the target")

    if name == "look_out_for" and not args.get("description_of_things"):
        reasons.append("look_out_for descriptions are missing")

    # Stop/utility/speech tools should remain easy to call. Do not penalize them
    # for missing robot context; only preserve recent-failure concerns.
    if name.startswith("stop_") or name in {"current_time", "wait", "speak"}:
        return [reason for reason in reasons if "recent failure" in reason]

    return reasons


def _risk_level(reasons: list[str]) -> str:
    """Map reasons to coarse risk.

    V2 uses three buckets rather than a fake numeric probability:
    - high: missing required args or repeated same-skill failures;
    - medium: context/history warnings;
    - low: no blocking risk detected.
    """
    high_markers = (
        "failed repeatedly",
        "navigation query is missing",
        "person-follow query is missing",
        "descriptions are missing",
    )
    if any(any(marker in reason for marker in high_markers) for reason in reasons):
        return "high"
    if reasons:
        return "medium"
    return "low"


def _recovery_suggestions(skill_name: str, reasons: list[str]) -> list[str]:
    """Translate risk reasons into actionable next steps for the LLM agent."""
    if not reasons:
        return []

    suggestions: list[str] = []
    name = skill_name.casefold()
    if "navigate" in name or name == "relative_move":
        suggestions.extend(
            [
                "call get_context with focus='navigation'",
                "consider look_out_for before navigation if the target is visual",
                "call stop_navigation before retrying if navigation is already active",
            ]
        )
    if "follow" in name:
        suggestions.extend(
            [
                "call look_out_for to detect the person first",
                "include a specific visual description in the query",
            ]
        )
    if not suggestions:
        suggestions.append("gather context and retry with more specific arguments")
    return suggestions


__all__ = ["_Go2SkillOutcomePredictor"]
