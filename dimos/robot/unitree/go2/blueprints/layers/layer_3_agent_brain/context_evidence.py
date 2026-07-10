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

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class ContextEvidencePolicy:
    min_relevance_score: float = 0.0
    max_entries: int = 12
    include_low_confidence: bool = True
    require_robot_state_for_motion: bool = True


def build_context_evidence(
    metadata: dict[str, Any],
    policy: ContextEvidencePolicy,
) -> dict[str, Any]:
    """Build the explicit context evidence ledger from ContextProvider metadata."""
    task = str(metadata.get("task") or "")
    focus = str(metadata.get("focus") or "")
    entries: list[dict[str, Any]] = [
        _evidence_entry(
            source="task",
            query=task,
            relevance_score=1.0,
            recency="current",
            confidence="high",
            risk_impact="unknown",
            cost="free",
            selected_reason="Current user goal anchors retrieval.",
            summary=task,
        )
    ]

    runtime = metadata.get("runtime") if isinstance(metadata.get("runtime"), dict) else {}
    entries.append(
        _evidence_entry(
            source="runtime_config",
            query=focus or task,
            relevance_score=0.8,
            recency="current",
            confidence="high",
            risk_impact="medium" if runtime.get("mode") == "hardware" else "low",
            cost="config_read",
            selected_reason="Runtime mode changes safety and execution assumptions.",
            summary=f"mode={runtime.get('mode', 'unknown')}",
        )
    )

    robot_state = (
        metadata.get("robot_state") if isinstance(metadata.get("robot_state"), dict) else {}
    )
    if any(robot_state.get(key) for key in ("odom", "navigation", "connection", "safety")):
        entries.append(
            _evidence_entry(
                source="robot_state",
                query=focus or task,
                relevance_score=0.85,
                recency="current",
                confidence="high",
                risk_impact=_robot_risk_impact(robot_state),
                cost="stream_cache",
                selected_reason="Current pose, navigation, connection, and safety state gate physical actions.",
                summary=_robot_state_summary(robot_state),
            )
        )

    world_state = (
        metadata.get("world_state") if isinstance(metadata.get("world_state"), dict) else {}
    )
    entries.extend(_world_evidence_entries(task, world_state))

    skill_state = (
        metadata.get("skill_state") if isinstance(metadata.get("skill_state"), dict) else {}
    )
    entries.extend(_skill_evidence_entries(task, skill_state))

    causal_state = (
        metadata.get("causal_state") if isinstance(metadata.get("causal_state"), dict) else {}
    )
    entries.extend(_causal_evidence_entries(task, causal_state))

    world_model_state = (
        metadata.get("world_model_state")
        if isinstance(metadata.get("world_model_state"), dict)
        else {}
    )
    entries.extend(_world_model_evidence_entries(task, world_model_state))
    entries = _apply_policy(entries=entries, task=task, policy=policy)

    return {
        "version": "context_evidence.v1",
        "selection_policy": "deterministic_source_coverage_v1",
        "query": task,
        "focus": focus,
        "entry_count": len(entries),
        "selected_sources": _ordered_sources(entries),
        "entries": entries,
    }


def _apply_policy(
    *,
    entries: list[dict[str, Any]],
    task: str,
    policy: ContextEvidencePolicy,
) -> list[dict[str, Any]]:
    min_score = max(0.0, min(1.0, policy.min_relevance_score))
    max_entries = max(1, policy.max_entries)
    protected = {"task", "runtime_config"}
    if policy.require_robot_state_for_motion and _looks_motion_task(task):
        protected.add("robot_state")

    filtered = [
        entry
        for entry in entries
        if entry["source"] in protected
        or (
            float(entry.get("relevance_score") or 0.0) >= min_score
            and (policy.include_low_confidence or entry.get("confidence") != "low")
        )
    ]
    protected_entries = [entry for entry in filtered if entry["source"] in protected]
    other_entries = [entry for entry in filtered if entry["source"] not in protected]
    return (protected_entries + other_entries)[:max_entries]


def _looks_motion_task(task: str) -> bool:
    text = task.casefold()
    return any(
        marker in text
        for marker in (
            "walk",
            "navigate",
            "move",
            "go to",
            "follow",
            "patrol",
            "turn",
            "security",
        )
    )


def _evidence_entry(
    *,
    source: str,
    query: str,
    relevance_score: float,
    recency: str,
    confidence: str,
    risk_impact: str,
    cost: str,
    selected_reason: str,
    summary: str,
) -> dict[str, Any]:
    return {
        "source": source,
        "query": query,
        "relevance_score": round(max(0.0, min(1.0, relevance_score)), 3),
        "recency": recency,
        "confidence": _normalize_label(confidence, {"low", "medium", "high"}, "medium"),
        "risk_impact": _normalize_label(
            risk_impact,
            {"low", "medium", "high", "unknown"},
            "unknown",
        ),
        "cost": cost,
        "selected_reason": selected_reason,
        "summary": summary[:300],
    }


def _world_evidence_entries(task: str, world_state: dict[str, Any]) -> list[dict[str, Any]]:
    entries: list[dict[str, Any]] = []
    spatial = world_state.get("spatial") if isinstance(world_state.get("spatial"), dict) else {}
    matches = spatial.get("matches") if isinstance(spatial.get("matches"), list) else []
    if matches:
        first_match = matches[0] if isinstance(matches[0], dict) else {}
        entries.append(
            _evidence_entry(
                source="spatial_memory",
                query=task,
                relevance_score=_spatial_relevance(first_match),
                recency=_match_recency(first_match),
                confidence=_spatial_confidence(first_match),
                risk_impact="medium",
                cost="rag_query",
                selected_reason="Top spatial/semantic RAG match for the current task.",
                summary=_spatial_summary(first_match),
            )
        )

    temporal = world_state.get("temporal") if isinstance(world_state.get("temporal"), dict) else {}
    temporal_summary = str(temporal.get("rolling_summary") or "")
    if temporal_summary or temporal.get("state") or temporal.get("answer"):
        entries.append(
            _evidence_entry(
                source="temporal_memory",
                query=task,
                relevance_score=0.75 if temporal_summary else 0.65,
                recency="recent_summary" if temporal_summary else "current_state",
                confidence="medium",
                risk_impact="medium",
                cost="vlm_graph_query" if temporal.get("answer") else "state_read",
                selected_reason="Temporal memory contributes recent entities, events, or rolling summary.",
                summary=temporal_summary or "temporal state available",
            )
        )

    semantic_temporal = world_state.get("semantic_temporal")
    if not isinstance(semantic_temporal, dict):
        semantic_temporal = world_state.get("semantic_temporal_map")
    if isinstance(semantic_temporal, dict):
        fused = semantic_temporal.get("fused")
        if isinstance(fused, dict) and fused.get("available"):
            entries.append(
                _evidence_entry(
                    source="semantic_temporal_map",
                    query=task,
                    relevance_score=0.7,
                    recency="current_fusion",
                    confidence="medium",
                    risk_impact="medium",
                    cost="rpc_read",
                    selected_reason="Layer 4 fused spatial and temporal evidence into normalized entries.",
                    summary=f"fused_entries={fused.get('entry_count', 0)}",
                )
            )
    return entries


def _skill_evidence_entries(task: str, skill_state: dict[str, Any]) -> list[dict[str, Any]]:
    entries: list[dict[str, Any]] = []
    recent_outcomes = (
        skill_state.get("recent_outcomes")
        if isinstance(skill_state.get("recent_outcomes"), list)
        else []
    )
    if recent_outcomes:
        failures = [outcome for outcome in recent_outcomes if not outcome.get("success")]
        entries.append(
            _evidence_entry(
                source="skill_outcome_store",
                query=task,
                relevance_score=0.9 if failures else 0.65,
                recency="recent_history",
                confidence="high",
                risk_impact="high" if failures else "low",
                cost="memory_read",
                selected_reason="Recent skill outcomes reveal repeated failures or successful recovery paths.",
                summary=f"{len(recent_outcomes)} outcome(s), {len(failures)} failure(s)",
            )
        )

    interface = (
        skill_state.get("interface") if isinstance(skill_state.get("interface"), dict) else {}
    )
    if interface.get("available"):
        entries.append(
            _evidence_entry(
                source="skill_interface_registry",
                query=task,
                relevance_score=0.8,
                recency="static_contract",
                confidence="high",
                risk_impact="medium",
                cost="rpc_read",
                selected_reason="Skill contracts constrain tool choice, required arguments, and preflight checks.",
                summary=f"{interface.get('skill_count', 0)} skill contract(s)",
            )
        )
    return entries


def _causal_evidence_entries(task: str, causal_state: dict[str, Any]) -> list[dict[str, Any]]:
    transitions = (
        causal_state.get("recent_transitions")
        if isinstance(causal_state.get("recent_transitions"), list)
        else []
    )
    if not transitions:
        return []
    failures = [
        transition for transition in transitions if transition.get("outcome_success") is False
    ]
    return [
        _evidence_entry(
            source="causal_world_model",
            query=task,
            relevance_score=0.88 if failures else 0.7,
            recency="recent_transition",
            confidence="medium",
            risk_impact="high" if failures else "low",
            cost="memory_read",
            selected_reason="Causal transitions explain whether prior actions changed state as expected.",
            summary=f"{len(transitions)} transition(s), {len(failures)} failure(s)",
        )
    ]


def _world_model_evidence_entries(
    task: str, world_model_state: dict[str, Any]
) -> list[dict[str, Any]]:
    if not world_model_state.get("available"):
        return []
    model = (
        world_model_state.get("model") if isinstance(world_model_state.get("model"), dict) else {}
    )
    failures = int(world_model_state.get("recent_failure_count") or 0)
    return [
        _evidence_entry(
            source="predictive_world_model",
            query=task,
            relevance_score=0.82 if model.get("sample_count") else 0.6,
            recency="online_model",
            confidence="medium" if model.get("sample_count") else "low",
            risk_impact="high" if failures else "medium",
            cost="model_state_read",
            selected_reason="Predictive world-model state exposes learned samples and recent failure modes.",
            summary=(
                f"samples={model.get('sample_count', 0)}, "
                f"recent_failures={failures}, "
                f"top_failure={world_model_state.get('top_failure_mode', '')}"
            ),
        )
    ]


def _robot_risk_impact(robot_state: dict[str, Any]) -> str:
    safety = robot_state.get("safety") if isinstance(robot_state.get("safety"), dict) else {}
    navigation = (
        robot_state.get("navigation") if isinstance(robot_state.get("navigation"), dict) else {}
    )
    if safety and not safety.get("body_pose_available", True):
        return "high"
    if navigation and navigation.get("goal_reached") is False:
        return "medium"
    return "low"


def _robot_state_summary(robot_state: dict[str, Any]) -> str:
    parts: list[str] = []
    odom = robot_state.get("odom")
    if isinstance(odom, dict):
        position = odom.get("position") if isinstance(odom.get("position"), dict) else {}
        parts.append(f"pose=({position.get('x')}, {position.get('y')})")
    navigation = robot_state.get("navigation")
    if isinstance(navigation, dict):
        parts.append(f"navigation={navigation.get('state')}")
    connection = robot_state.get("connection")
    if isinstance(connection, dict):
        parts.append(f"connection={connection.get('mode')}")
    return ", ".join(parts) or "robot state available"


def _spatial_relevance(match: dict[str, Any]) -> float:
    score = match.get("score")
    if isinstance(score, int | float) and not isinstance(score, bool):
        return float(score)
    distance = match.get("distance")
    if isinstance(distance, int | float) and not isinstance(distance, bool):
        return 1.0 - min(max(float(distance), 0.0), 1.0)
    return 0.6


def _spatial_confidence(match: dict[str, Any]) -> str:
    relevance = _spatial_relevance(match)
    if relevance >= 0.75:
        return "high"
    if relevance >= 0.45:
        return "medium"
    return "low"


def _spatial_summary(match: dict[str, Any]) -> str:
    metadata = match.get("metadata")
    if isinstance(metadata, list) and metadata and isinstance(metadata[0], dict):
        metadata = metadata[0]
    if isinstance(metadata, dict):
        label = metadata.get("label") or metadata.get("description") or metadata.get("frame_id")
        if label:
            return str(label)
        if "pos_x" in metadata and "pos_y" in metadata:
            return f"spatial match at x={metadata.get('pos_x')}, y={metadata.get('pos_y')}"
    if match.get("id"):
        return str(match["id"])
    return "spatial match"


def _match_recency(match: dict[str, Any]) -> str:
    metadata = match.get("metadata")
    if isinstance(metadata, list) and metadata and isinstance(metadata[0], dict):
        metadata = metadata[0]
    if isinstance(metadata, dict) and metadata.get("timestamp") is not None:
        return "timestamped_observation"
    return "stored_observation"


def _ordered_sources(entries: list[dict[str, Any]]) -> list[str]:
    sources: set[str] = set()
    for entry in entries:
        source = str(entry.get("source") or "")
        if source:
            sources.add(source)
    return sorted(sources)


def _normalize_label(value: str, allowed: set[str], fallback: str) -> str:
    normalized = value.strip().casefold()
    if normalized in allowed:
        return normalized
    return fallback


__all__ = ["ContextEvidencePolicy", "build_context_evidence"]
