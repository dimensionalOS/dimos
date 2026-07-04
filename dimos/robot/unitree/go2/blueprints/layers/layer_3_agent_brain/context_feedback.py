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
from dimos.robot.unitree.go2.blueprints.layers.layer_3_agent_brain.evolution_ledger import (
    EvolutionLedgerSpec,
)
from dimos.spec.utils import Spec

CONTEXT_FEEDBACK_SCHEMA = "go2_context_feedback.v1"
CONTEXT_EVIDENCE_SCHEMA = "context_evidence.v1"


class ContextFeedbackSpec(Spec, Protocol):
    """RPC surface for Layer 3 modules that read context feedback summaries."""

    def get_recent_context_feedback(
        self,
        limit: int = 5,
        source: str = "",
    ) -> list[dict[str, Any]]: ...

    def get_context_feedback_summary(
        self,
        limit: int = 20,
    ) -> dict[str, Any]: ...


@dataclass(frozen=True)
class _ContextFeedback:
    timestamp: float
    task: str
    selected_skill: str
    outcome_success: bool | None
    outcome_error_code: str
    evidence_sources: tuple[str, ...]
    helpful_sources: tuple[str, ...]
    ignored_risks: tuple[str, ...]
    helpful_source_counts: dict[str, int]
    harmful_source_counts: dict[str, int]

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema": CONTEXT_FEEDBACK_SCHEMA,
            "timestamp": round(self.timestamp, 3),
            "task": self.task,
            "selected_skill": self.selected_skill,
            "outcome_success": self.outcome_success,
            "outcome_error_code": self.outcome_error_code,
            "evidence_sources": list(self.evidence_sources),
            "helpful_sources": list(self.helpful_sources),
            "ignored_risks": list(self.ignored_risks),
            "helpful_source_counts": dict(self.helpful_source_counts),
            "harmful_source_counts": dict(self.harmful_source_counts),
        }


class _Go2ContextFeedbackStore(Module):
    """Bounded store for whether selected context evidence helped task outcomes."""

    _max_feedback = 100
    _evolution_ledger: EvolutionLedgerSpec | None = None

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._feedback: deque[_ContextFeedback] = deque(maxlen=self._max_feedback)

    @skill
    def record_context_feedback(
        self,
        task: str,
        context_evidence_json: str,
        selected_skill: str = "",
        outcome_json: str = "{}",
        helpful_sources_json: str = "[]",
        ignored_risks_json: str = "[]",
    ) -> SkillResult:
        """Record whether selected context evidence helped or hurt the task outcome.

        Args:
            task: User task or subtask that used the context.
            context_evidence_json: JSON object with version context_evidence.v1.
            selected_skill: Skill selected after reading the context.
            outcome_json: JSON object describing the observed outcome.
            helpful_sources_json: JSON list of evidence source names that helped.
            ignored_risks_json: JSON list of risk/source labels that were ignored.
        """
        task = task.strip()
        selected_skill = selected_skill.strip()
        if not task:
            return SkillResult.fail("INVALID_INPUT", "task is required")

        parsed = _parse_feedback_inputs(
            context_evidence_json=context_evidence_json,
            outcome_json=outcome_json,
            helpful_sources_json=helpful_sources_json,
            ignored_risks_json=ignored_risks_json,
        )
        if isinstance(parsed, str):
            return SkillResult.fail("INVALID_INPUT", parsed)
        evidence, outcome, helpful_sources, ignored_risks = parsed

        feedback = _build_context_feedback(
            task=task,
            selected_skill=selected_skill,
            evidence=evidence,
            outcome=outcome,
            helpful_sources=helpful_sources,
            ignored_risks=ignored_risks,
        )
        self._feedback.append(feedback)

        metadata: dict[str, Any] = {
            "feedback": feedback.to_dict(),
            "total_feedback": len(self._feedback),
        }
        ledger_warning = self._write_ledger_event(feedback)
        if ledger_warning:
            metadata["warnings"] = [ledger_warning]

        return SkillResult.ok(
            f"Recorded context feedback for {selected_skill or 'unknown skill'}",
            **metadata,
        )

    @rpc
    def get_recent_context_feedback(
        self,
        limit: int = 5,
        source: str = "",
    ) -> list[dict[str, Any]]:
        """Return newest context feedback records first, optionally filtered by source."""
        limit = max(0, min(limit, self._max_feedback))
        source = source.strip()
        records = [
            feedback
            for feedback in reversed(self._feedback)
            if not source or _feedback_mentions_source(feedback, source)
        ]
        return [feedback.to_dict() for feedback in records[:limit]]

    @rpc
    def get_context_feedback_summary(self, limit: int = 20) -> dict[str, Any]:
        """Return aggregate counts over recent context feedback records."""
        records = self.get_recent_context_feedback(limit=limit)
        helpful = Counter()
        harmful = Counter()
        successes = 0
        failures = 0
        unknown = 0
        for record in records:
            helpful.update(record.get("helpful_source_counts") or {})
            harmful.update(record.get("harmful_source_counts") or {})
            if record.get("outcome_success") is True:
                successes += 1
            elif record.get("outcome_success") is False:
                failures += 1
            else:
                unknown += 1
        return {
            "schema": CONTEXT_FEEDBACK_SCHEMA,
            "total_feedback": len(records),
            "success_count": successes,
            "failure_count": failures,
            "unknown_count": unknown,
            "helpful_source_counts": dict(helpful),
            "harmful_source_counts": dict(harmful),
        }

    def _write_ledger_event(self, feedback: _ContextFeedback) -> str:
        if self._evolution_ledger is None:
            return ""
        try:
            self._evolution_ledger.write_evolution_event(
                event_type="context_feedback",
                task=feedback.task,
                payload=feedback.to_dict(),
                commit=False,
            )
        except Exception as exc:
            return f"evolution_ledger: {exc}"
        return ""


def _parse_feedback_inputs(
    *,
    context_evidence_json: str,
    outcome_json: str,
    helpful_sources_json: str,
    ignored_risks_json: str,
) -> tuple[dict[str, Any], dict[str, Any], list[str], list[str]] | str:
    evidence = _parse_json_object(context_evidence_json, "context_evidence_json")
    if isinstance(evidence, str):
        return evidence
    if isinstance(evidence.get("context_evidence"), dict):
        evidence = evidence["context_evidence"]
    if evidence.get("version") != CONTEXT_EVIDENCE_SCHEMA:
        return "context_evidence_json must have version context_evidence.v1"

    outcome = _parse_json_object(outcome_json or "{}", "outcome_json")
    if isinstance(outcome, str):
        return outcome
    helpful_sources = _parse_json_string_list(helpful_sources_json, "helpful_sources_json")
    if isinstance(helpful_sources, str):
        return helpful_sources
    ignored_risks = _parse_json_string_list(ignored_risks_json, "ignored_risks_json")
    if isinstance(ignored_risks, str):
        return ignored_risks
    return evidence, outcome, helpful_sources, ignored_risks


def _parse_json_object(raw: str, field: str) -> dict[str, Any] | str:
    try:
        parsed = json.loads(raw or "{}")
    except json.JSONDecodeError as exc:
        return f"{field} must be a JSON object: {exc.msg}"
    if not isinstance(parsed, dict):
        return f"{field} must be a JSON object"
    return parsed


def _parse_json_string_list(raw: str, field: str) -> list[str] | str:
    try:
        parsed = json.loads(raw or "[]")
    except json.JSONDecodeError as exc:
        return f"{field} must be a JSON list of strings: {exc.msg}"
    if not isinstance(parsed, list) or not all(isinstance(item, str) for item in parsed):
        return f"{field} must be a JSON list of strings"
    return _unique([item.strip() for item in parsed if item.strip()])


def _build_context_feedback(
    *,
    task: str,
    selected_skill: str,
    evidence: dict[str, Any],
    outcome: dict[str, Any],
    helpful_sources: list[str],
    ignored_risks: list[str],
) -> _ContextFeedback:
    evidence_sources = _evidence_sources(evidence)
    outcome_success = outcome.get("success")
    if not isinstance(outcome_success, bool):
        outcome_success = None
    outcome_error_code = str(outcome.get("error_code") or "")
    helpful_counts = {source: 1 for source in helpful_sources}
    harmful_counts = _harmful_source_counts(
        evidence=evidence,
        outcome_success=outcome_success,
        helpful_sources=helpful_sources,
        ignored_risks=ignored_risks,
    )
    return _ContextFeedback(
        timestamp=time.time(),
        task=task,
        selected_skill=selected_skill,
        outcome_success=outcome_success,
        outcome_error_code=outcome_error_code,
        evidence_sources=tuple(evidence_sources),
        helpful_sources=tuple(helpful_sources),
        ignored_risks=tuple(ignored_risks),
        helpful_source_counts=helpful_counts,
        harmful_source_counts=harmful_counts,
    )


def _evidence_sources(evidence: dict[str, Any]) -> list[str]:
    sources: list[str] = []
    selected = evidence.get("selected_sources")
    if isinstance(selected, list):
        sources.extend(str(source) for source in selected if source)
    entries = evidence.get("entries")
    if isinstance(entries, list):
        for entry in entries:
            if isinstance(entry, dict) and entry.get("source"):
                sources.append(str(entry["source"]))
    return _unique(sources)


def _harmful_source_counts(
    *,
    evidence: dict[str, Any],
    outcome_success: bool | None,
    helpful_sources: list[str],
    ignored_risks: list[str],
) -> dict[str, int]:
    harmful = set(ignored_risks)
    if outcome_success is False:
        for entry in evidence.get("entries") or []:
            if not isinstance(entry, dict):
                continue
            source = str(entry.get("source") or "")
            if not source or source in helpful_sources:
                continue
            if entry.get("risk_impact") == "high":
                harmful.add(source)
    return {source: 1 for source in _unique(list(harmful))}


def _feedback_mentions_source(feedback: _ContextFeedback, source: str) -> bool:
    return (
        source in feedback.evidence_sources
        or source in feedback.helpful_sources
        or source in feedback.harmful_source_counts
    )


def _unique(items: list[str]) -> list[str]:
    seen: set[str] = set()
    result: list[str] = []
    for item in items:
        if item and item not in seen:
            seen.add(item)
            result.append(item)
    return result


__all__ = [
    "CONTEXT_EVIDENCE_SCHEMA",
    "CONTEXT_FEEDBACK_SCHEMA",
    "ContextFeedbackSpec",
    "_Go2ContextFeedbackStore",
]
