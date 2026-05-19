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

from collections import Counter, deque
from dataclasses import dataclass
import json
import time
from typing import Any, Protocol

from dimos.agents.annotation import skill
from dimos.agents.skill_result import SkillResult
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.robot.unitree.go2.blueprints.layers.layer_3_agent_brain.skill_outcome_store import (
    SkillOutcomeStoreSpec,
)
from dimos.spec.utils import Spec


class CausalWorldModelSpec(Spec, Protocol):
    """RPC surface for Layer 3 modules that need recent causal history."""

    def get_recent_transitions(
        self,
        limit: int = 5,
        skill_name: str = "",
        domain: str = "",
        cause: str = "",
    ) -> list[dict[str, Any]]: ...


@dataclass(frozen=True)
class _CausalTransition:
    """One observed action transition with inferred cause and recovery."""

    timestamp: float
    task: str
    domain: str
    skill_name: str
    args: dict[str, Any]
    before_context: str
    prediction_risk: str
    prediction_reasons: list[str]
    outcome_success: bool | None
    outcome_error_code: str
    outcome_message: str
    after_context: str
    inferred_cause: str
    recovery: str
    confidence: str

    def to_dict(self) -> dict[str, Any]:
        return {
            "timestamp": round(self.timestamp, 3),
            "task": self.task,
            "domain": self.domain,
            "skill_name": self.skill_name,
            "args": self.args,
            "before_context": self.before_context,
            "prediction_risk": self.prediction_risk,
            "prediction_reasons": self.prediction_reasons,
            "outcome_success": self.outcome_success,
            "outcome_error_code": self.outcome_error_code,
            "outcome_message": self.outcome_message,
            "after_context": self.after_context,
            "inferred_cause": self.inferred_cause,
            "recovery": self.recovery,
            "confidence": self.confidence,
        }


class _Go2CausalWorldModel(Module):
    """In-memory Layer 3 causal transition recorder for Go2."""

    _skill_outcomes: SkillOutcomeStoreSpec | None = None
    _max_transitions = 200

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._transitions: deque[_CausalTransition] = deque(maxlen=self._max_transitions)

    @skill
    def record_causal_transition(
        self,
        task: str,
        skill_name: str,
        args_json: str = "",
        before_context: str = "",
        after_context: str = "",
        prediction_json: str = "",
        outcome_json: str = "",
        domain: str = "",
    ) -> SkillResult:
        """Record one before/action/result/after causal transition.

        Use this after an important physical or recovery-sensitive tool call.
        The tool stores a compact event, infers a coarse cause, and returns a
        recovery suggestion. It does not execute or retry robot actions.

        Args:
            task: User goal or agent subtask that led to the action.
            skill_name: Tool or skill that was executed.
            args_json: Optional JSON object string with the executed arguments.
            before_context: Context summary captured before the action.
            after_context: Context summary captured after the action.
            prediction_json: Optional JSON object from predict_skill_outcome.
            outcome_json: Optional JSON object from the tool result or outcome
                store. If omitted, the latest same-skill outcome is used when
                SkillOutcomeStore is wired.
            domain: Optional expert domain override.
        """
        task = task.strip()
        skill_name = skill_name.strip()
        domain = domain.strip()
        before_context = before_context.strip()
        after_context = after_context.strip()

        if not task:
            return SkillResult.fail("INVALID_INPUT", "task is required")
        if not skill_name:
            return SkillResult.fail("INVALID_INPUT", "skill_name is required")

        parsed_args = _parse_json_object(args_json, "args_json")
        if isinstance(parsed_args, str):
            return SkillResult.fail("INVALID_INPUT", parsed_args)

        prediction = _parse_json_object(prediction_json, "prediction_json")
        if isinstance(prediction, str):
            return SkillResult.fail("INVALID_INPUT", prediction)

        outcome = _parse_json_object(outcome_json, "outcome_json")
        if isinstance(outcome, str):
            return SkillResult.fail("INVALID_INPUT", outcome)
        if not outcome:
            outcome = self._latest_outcome(skill_name)

        prediction_view = _metadata_view(prediction)
        outcome_view = _metadata_view(outcome)
        prediction_reasons = _prediction_reasons(prediction_view)
        outcome_success = _optional_bool(outcome_view.get("success"))
        outcome_error_code = str(outcome_view.get("error_code") or "")
        outcome_message = str(outcome_view.get("message") or "")
        prediction_risk = str(prediction_view.get("risk") or "unknown")
        resolved_domain = domain or str(prediction_view.get("domain") or _infer_domain(skill_name))

        cause, recovery, confidence = _infer_cause(
            skill_name=skill_name,
            before_context=before_context,
            after_context=after_context,
            prediction_reasons=prediction_reasons,
            outcome_success=outcome_success,
            outcome_message=outcome_message,
            outcome_error_code=outcome_error_code,
        )

        transition = _CausalTransition(
            timestamp=time.time(),
            task=task,
            domain=resolved_domain,
            skill_name=skill_name,
            args=parsed_args,
            before_context=before_context[:500],
            prediction_risk=prediction_risk,
            prediction_reasons=prediction_reasons,
            outcome_success=outcome_success,
            outcome_error_code=outcome_error_code,
            outcome_message=outcome_message[:500],
            after_context=after_context[:500],
            inferred_cause=cause,
            recovery=recovery,
            confidence=confidence,
        )
        self._transitions.append(transition)

        return SkillResult.ok(
            f"Recorded causal transition for {skill_name}: {cause}",
            transition=transition.to_dict(),
            total_transitions=len(self._transitions),
        )

    @skill
    def summarize_causal_patterns(
        self, skill_name: str = "", domain: str = "", limit: int = 10
    ) -> SkillResult:
        """Summarize recent causal failure patterns.

        Args:
            skill_name: Optional skill-name filter.
            domain: Optional expert-domain filter.
            limit: Maximum number of recent transitions to inspect.
        """
        transitions = self.get_recent_transitions(
            limit=limit,
            skill_name=skill_name,
            domain=domain,
        )
        if not transitions:
            return SkillResult.ok("No recorded causal transitions", patterns=[], transitions=[])

        patterns = _summarize_patterns(transitions)
        if not patterns:
            return SkillResult.ok(
                f"{len(transitions)} transition(s), no repeated failure pattern",
                patterns=[],
                transitions=transitions,
            )

        top = patterns[0]
        message = (
            f"{len(transitions)} transition(s), top failure cause "
            f"{top['cause']} occurred {top['count']} time(s)"
        )
        return SkillResult.ok(message, patterns=patterns, transitions=transitions)

    @rpc
    def get_recent_transitions(
        self,
        limit: int = 5,
        skill_name: str = "",
        domain: str = "",
        cause: str = "",
    ) -> list[dict[str, Any]]:
        """Return newest causal transitions first, optionally filtered."""
        limit = max(0, min(limit, self._max_transitions))
        skill_name = skill_name.strip()
        domain = domain.strip()
        cause = cause.strip()

        transitions = [
            transition
            for transition in reversed(self._transitions)
            if (not skill_name or transition.skill_name == skill_name)
            and (not domain or transition.domain == domain)
            and (not cause or transition.inferred_cause == cause)
        ]
        return [transition.to_dict() for transition in transitions[:limit]]

    def _latest_outcome(self, skill_name: str) -> dict[str, Any]:
        if self._skill_outcomes is None:
            return {}
        outcomes = self._skill_outcomes.get_recent_outcomes(limit=1, skill_name=skill_name)
        return outcomes[0] if outcomes else {}


def _parse_json_object(value: str, field_name: str) -> dict[str, Any] | str:
    value = value.strip()
    if not value:
        return {}
    try:
        parsed = json.loads(value)
    except json.JSONDecodeError as exc:
        return f"{field_name} must be a JSON object: {exc}"
    if not isinstance(parsed, dict):
        return f"{field_name} must be a JSON object"
    return parsed


def _metadata_view(value: dict[str, Any]) -> dict[str, Any]:
    metadata = value.get("metadata")
    if isinstance(metadata, dict):
        merged = dict(value)
        merged.update(metadata)
        return merged
    return value


def _prediction_reasons(prediction: dict[str, Any]) -> list[str]:
    reasons = prediction.get("failure_reasons") or prediction.get("reasons") or []
    if isinstance(reasons, list):
        return [str(reason)[:300] for reason in reasons]
    if isinstance(reasons, str):
        return [reasons[:300]]
    return []


def _optional_bool(value: Any) -> bool | None:
    if isinstance(value, bool):
        return value
    if value is None:
        return None
    if isinstance(value, str):
        lowered = value.casefold()
        if lowered == "true":
            return True
        if lowered == "false":
            return False
    return None


def _infer_cause(
    skill_name: str,
    before_context: str,
    after_context: str,
    prediction_reasons: list[str],
    outcome_success: bool | None,
    outcome_message: str,
    outcome_error_code: str,
) -> tuple[str, str, str]:
    text = " ".join([before_context, after_context, outcome_message]).casefold()
    reasons = " ".join(prediction_reasons).casefold()

    if outcome_success is True:
        return ("success_no_failure", "", "high")

    if "query is missing" in reasons or "descriptions are missing" in reasons:
        return (
            "invalid_or_missing_arguments",
            "Retry with complete, specific tool arguments.",
            "high",
        )
    if "failed repeatedly" in reasons:
        return (
            "repeated_failure_pattern",
            "Call summarize_causal_patterns before retrying this skill.",
            "high",
        )
    if "robot pose: unavailable" in text or "odometry may be unavailable" in reasons:
        return (
            "robot_state_unavailable",
            "Call get_context with focus='navigation' and wait for odometry.",
            "high",
        )
    if "no relevant matches" in text or "no matching location" in text:
        return (
            "semantic_map_missing_target",
            "Use perception, ask the user, or tag the location before navigating.",
            "medium",
        )
    if "spatial memory is unavailable" in reasons:
        return (
            "world_state_unavailable",
            "Wait for world-state modules or use a direct perception skill.",
            "medium",
        )
    if "could not find" in text or "no image available" in text:
        return (
            "visual_target_unavailable",
            "Call look_out_for or ask for a more specific visual description.",
            "medium",
        )
    if outcome_error_code:
        return (
            f"tool_error:{outcome_error_code}",
            "Inspect the tool error and gather context before retrying.",
            "medium",
        )
    if outcome_success is False:
        return (
            "unknown_failure",
            "Gather context and retry with more specific arguments.",
            "low",
        )
    return ("unknown_transition", "Record a clearer outcome before reasoning from it.", "low")


def _summarize_patterns(transitions: list[dict[str, Any]]) -> list[dict[str, Any]]:
    failures = [
        transition
        for transition in transitions
        if transition.get("outcome_success") is False
        and transition.get("inferred_cause") not in {"success_no_failure"}
    ]
    counts = Counter(str(transition.get("inferred_cause")) for transition in failures)
    patterns: list[dict[str, Any]] = []
    for cause, count in counts.most_common():
        examples = [
            transition for transition in failures if transition.get("inferred_cause") == cause
        ]
        skills = sorted({str(transition.get("skill_name")) for transition in examples})
        recovery = str(examples[0].get("recovery") or "")
        patterns.append(
            {
                "cause": cause,
                "count": count,
                "skill_names": skills,
                "latest_message": str(examples[0].get("outcome_message") or ""),
                "recovery": recovery,
            }
        )
    return patterns


def _infer_domain(skill_name: str) -> str:
    if skill_name in {"navigate_with_text", "stop_navigation"}:
        return "navigation"
    if skill_name in {"relative_move", "execute_sport_command"}:
        return "robot_motion"
    if skill_name in {"follow_person", "stop_following"}:
        return "person_follow"
    if skill_name in {"look_out_for", "stop_looking_out"}:
        return "perception"
    if skill_name in {"start_security_patrol", "stop_security_patrol"}:
        return "security"
    if skill_name == "speak":
        return "speech"
    return ""


__all__ = ["CausalWorldModelSpec", "_Go2CausalWorldModel"]
