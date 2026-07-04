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

from typing import Any

from dimos.agents.annotation import skill
from dimos.agents.skill_result import SkillResult
from dimos.core.module import Module
from dimos.perception.spatial_memory_spec import SpatialMemorySpec
from dimos.perception.temporal_memory_spec import TemporalMemorySpec
from dimos.robot.unitree.go2.blueprints.layers.layer_3_agent_brain.causal_world_model import (
    CausalWorldModelSpec,
)
from dimos.robot.unitree.go2.blueprints.layers.layer_3_agent_brain.skill_outcome_store import (
    SkillOutcomeStoreSpec,
)
from dimos.robot.unitree.go2.blueprints.layers.layer_4_world_state.world_state_spec import (
    WorldStateSpec,
)
from dimos.robot.unitree.go2.blueprints.layers.layer_5_skill_interface.skill_interface_spec import (
    SkillInterfaceSpec,
)

_STATUS_PROBE_QUERY = "__memory_status_probe__"


class _Go2MemoryBackendStatus(Module):
    """Read-only Layer 3 view of Go2 memory/backend readiness."""

    _world_state: WorldStateSpec | None = None
    _spatial_memory: SpatialMemorySpec | None = None
    _temporal_memory: TemporalMemorySpec | None = None
    _skill_outcomes: SkillOutcomeStoreSpec | None = None
    _causal_world_model: CausalWorldModelSpec | None = None
    _skill_interface: SkillInterfaceSpec | None = None

    @skill
    def memory_backend_status(self) -> SkillResult:
        """Report which DimOS memory backends are wired and how much data they expose."""
        warnings: list[str] = []
        metadata = {
            "schema": "go2_memory_backend_status.v1",
            "world_state": self._world_state_status(warnings),
            "spatial_memory": self._spatial_memory_status(warnings),
            "temporal_memory": self._temporal_memory_status(warnings),
            "skill_outcomes": self._skill_outcomes_status(warnings),
            "causal_world_model": self._causal_world_model_status(warnings),
            "skill_interface": self._skill_interface_status(warnings),
            "warnings": warnings,
        }
        wired_count = sum(
            1
            for key in (
                "world_state",
                "spatial_memory",
                "temporal_memory",
                "skill_outcomes",
                "causal_world_model",
                "skill_interface",
            )
            if metadata[key]["wired"]
        )
        return SkillResult.ok(
            f"Memory backends: {wired_count} wired, {len(warnings)} warning(s)",
            **metadata,
        )

    def _world_state_status(self, warnings: list[str]) -> dict[str, Any]:
        status: dict[str, Any] = {
            "wired": self._world_state is not None,
            "snapshot_available": False,
            "sources": {},
            "snapshot_storage": {},
        }
        if self._world_state is None:
            return status
        try:
            snapshot = self._world_state.get_world_snapshot(
                task="memory backend status",
                spatial_limit=0,
            )
        except Exception as exc:
            warnings.append(f"world_state: {exc}")
            return status
        status["snapshot_available"] = True
        status["sources"] = _dict_or_empty(snapshot.get("sources"))
        status["snapshot_storage"] = _dict_or_empty(snapshot.get("snapshot_storage"))
        return status

    def _spatial_memory_status(self, warnings: list[str]) -> dict[str, Any]:
        status: dict[str, Any] = {
            "wired": self._spatial_memory is not None,
            "query_available": False,
            "match_count_probe": 0,
            "backend_hint": "",
        }
        if self._spatial_memory is None:
            return status
        try:
            matches = self._spatial_memory.query_by_text(_STATUS_PROBE_QUERY, limit=1)
        except Exception as exc:
            warnings.append(f"spatial_memory: {exc}")
            return status
        status["query_available"] = True
        status["match_count_probe"] = len(matches) if isinstance(matches, list) else 0
        status["backend_hint"] = "SpatialMemorySpec.query_by_text"
        return status

    def _temporal_memory_status(self, warnings: list[str]) -> dict[str, Any]:
        status: dict[str, Any] = {
            "wired": self._temporal_memory is not None,
            "entity_count": 0,
            "graph_stats_available": False,
            "graph_stats": {},
        }
        if self._temporal_memory is None:
            return status
        try:
            state = self._temporal_memory.get_state()
            status["entity_count"] = _entity_count(state)
        except Exception as exc:
            warnings.append(f"temporal_memory.state: {exc}")
        try:
            graph_stats = self._temporal_memory.get_graph_db_stats()
        except Exception as exc:
            warnings.append(f"temporal_memory.graph: {exc}")
            return status
        status["graph_stats_available"] = True
        status["graph_stats"] = _dict_or_empty(graph_stats.get("stats", graph_stats))
        return status

    def _skill_outcomes_status(self, warnings: list[str]) -> dict[str, Any]:
        status: dict[str, Any] = {
            "wired": self._skill_outcomes is not None,
            "recent_count": 0,
        }
        if self._skill_outcomes is None:
            return status
        try:
            recent = self._skill_outcomes.get_recent_outcomes(limit=5)
        except Exception as exc:
            warnings.append(f"skill_outcomes: {exc}")
            return status
        status["recent_count"] = len(recent) if isinstance(recent, list) else 0
        return status

    def _causal_world_model_status(self, warnings: list[str]) -> dict[str, Any]:
        status: dict[str, Any] = {
            "wired": self._causal_world_model is not None,
            "transition_count": 0,
            "sample_count": 0,
            "persistence_path": "",
            "autosave": False,
        }
        if self._causal_world_model is None:
            return status
        try:
            transitions = self._causal_world_model.get_recent_transitions(limit=5)
        except Exception as exc:
            warnings.append(f"causal_world_model.transitions: {exc}")
            transitions = []
        status["transition_count"] = len(transitions) if isinstance(transitions, list) else 0

        get_model_state = getattr(self._causal_world_model, "get_model_state", None)
        if get_model_state is None:
            return status
        try:
            model_state = get_model_state()
        except Exception as exc:
            warnings.append(f"causal_world_model.state: {exc}")
            return status
        status["sample_count"] = int(model_state.get("sample_count") or 0)
        persistence = _dict_or_empty(model_state.get("persistence"))
        status["persistence_path"] = str(persistence.get("path") or "")
        status["autosave"] = bool(persistence.get("autosave"))
        return status

    def _skill_interface_status(self, warnings: list[str]) -> dict[str, Any]:
        status: dict[str, Any] = {
            "wired": self._skill_interface is not None,
            "skill_count": 0,
        }
        if self._skill_interface is None:
            return status
        try:
            snapshot = self._skill_interface.get_skill_interface_snapshot()
        except Exception as exc:
            warnings.append(f"skill_interface: {exc}")
            return status
        status["skill_count"] = int(snapshot.get("skill_count") or 0)
        return status


def _dict_or_empty(value: Any) -> dict[str, Any]:
    return value if isinstance(value, dict) else {}


def _entity_count(state: dict[str, Any]) -> int:
    count = state.get("entity_count")
    if isinstance(count, int):
        return count
    entities = state.get("entities")
    if isinstance(entities, list):
        return len(entities)
    entity_roster = state.get("entity_roster")
    if isinstance(entity_roster, list):
        return len(entity_roster)
    return 0


__all__ = ["_Go2MemoryBackendStatus"]
