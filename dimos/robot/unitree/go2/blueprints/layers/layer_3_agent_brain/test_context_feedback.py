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

from dimos.robot.unitree.go2.blueprints.layers.layer_3_agent_brain.context_feedback import (
    _Go2ContextFeedbackStore,
)
from dimos.robot.unitree.go2.blueprints.layers.layer_3_agent_brain.context_provider import (
    _Go2ContextProvider,
)


def _stop_modules(*modules: object) -> None:
    for module in modules:
        stop = getattr(module, "stop", None)
        if stop is not None:
            stop()


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
        return {"event_path": "/tmp/context-feedback.json", "warnings": []}


def _context_evidence_json() -> str:
    return json.dumps(
        {
            "version": "context_evidence.v1",
            "selected_sources": ["task", "spatial_memory", "robot_state"],
            "entries": [
                {
                    "source": "spatial_memory",
                    "risk_impact": "low",
                    "summary": "matched kitchen tag",
                },
                {
                    "source": "robot_state",
                    "risk_impact": "high",
                    "summary": "pose unavailable",
                },
            ],
        }
    )


def test_context_feedback_records_helpful_and_harmful_source_counts() -> None:
    store = _Go2ContextFeedbackStore()
    try:
        result = store.record_context_feedback(
            task="walk to the kitchen",
            context_evidence_json=_context_evidence_json(),
            selected_skill="navigate_with_text",
            outcome_json='{"success": false, "error_code": "EXECUTION_FAILED"}',
            helpful_sources_json='["spatial_memory"]',
            ignored_risks_json='["robot_state"]',
        )

        assert result.success is True
        feedback = result.metadata["feedback"]
        assert feedback["helpful_source_counts"] == {"spatial_memory": 1}
        assert feedback["harmful_source_counts"] == {"robot_state": 1}
        assert feedback["outcome_success"] is False
        assert result.metadata["total_feedback"] == 1

        recent = store.get_recent_context_feedback(limit=5)
        assert recent[0]["selected_skill"] == "navigate_with_text"
    finally:
        _stop_modules(store)


def test_context_feedback_rejects_invalid_context_evidence() -> None:
    store = _Go2ContextFeedbackStore()
    try:
        result = store.record_context_feedback(
            task="walk",
            context_evidence_json='{"version": "wrong"}',
        )

        assert result.success is False
        assert result.error_code == "INVALID_INPUT"
    finally:
        _stop_modules(store)


def test_context_feedback_summary_is_bounded_and_newest_first() -> None:
    store = _Go2ContextFeedbackStore()
    try:
        for idx in range(105):
            store.record_context_feedback(
                task=f"task {idx}",
                context_evidence_json=_context_evidence_json(),
                selected_skill="navigate_with_text",
                outcome_json='{"success": true}',
                helpful_sources_json='["spatial_memory"]',
            )

        recent = store.get_recent_context_feedback(limit=200)
        assert len(recent) == 100
        assert recent[0]["task"] == "task 104"
        assert recent[-1]["task"] == "task 5"

        summary = store.get_context_feedback_summary(limit=200)
        assert summary["total_feedback"] == 100
        assert summary["helpful_source_counts"]["spatial_memory"] == 100
    finally:
        _stop_modules(store)


def test_context_feedback_writes_to_ledger_when_available() -> None:
    store = _Go2ContextFeedbackStore()
    ledger = StubLedger()
    try:
        store._evolution_ledger = ledger  # type: ignore[assignment]

        result = store.record_context_feedback(
            task="walk to the kitchen",
            context_evidence_json=_context_evidence_json(),
            selected_skill="navigate_with_text",
            outcome_json='{"success": false}',
            helpful_sources_json='["spatial_memory"]',
        )

        assert result.success is True
        assert ledger.events[0]["event_type"] == "context_feedback"
        assert ledger.events[0]["task"] == "walk to the kitchen"
        assert ledger.events[0]["payload"]["selected_skill"] == "navigate_with_text"
    finally:
        _stop_modules(store)


def test_context_provider_includes_recent_context_feedback() -> None:
    store = _Go2ContextFeedbackStore()
    provider = _Go2ContextProvider()
    try:
        store.record_context_feedback(
            task="walk to the kitchen",
            context_evidence_json=_context_evidence_json(),
            selected_skill="navigate_with_text",
            outcome_json='{"success": true}',
            helpful_sources_json='["spatial_memory"]',
        )
        provider._context_feedback = store  # type: ignore[assignment]

        result = provider.get_context("walk to the kitchen")

        assert result.success is True
        feedback = result.metadata["context_feedback"]
        assert feedback["available"] is True
        assert feedback["summary"]["helpful_source_counts"]["spatial_memory"] == 1
        assert feedback["recent_feedback"][0]["task"] == "walk to the kitchen"
        assert "Context feedback: 1 recent" in result.message
    finally:
        _stop_modules(provider, store)
