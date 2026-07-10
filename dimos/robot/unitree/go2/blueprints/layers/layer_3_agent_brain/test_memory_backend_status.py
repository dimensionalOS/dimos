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

from typing import Any

from dimos.robot.unitree.go2.blueprints.layers.layer_3_agent_brain.memory_backend_status import (
    _Go2MemoryBackendStatus,
)


def _stop_modules(*modules: object) -> None:
    for module in modules:
        stop = getattr(module, "stop", None)
        if stop is not None:
            stop()


class StubWorldState:
    def get_world_snapshot(self, task: str = "", spatial_limit: int = 3) -> dict[str, Any]:
        return {
            "sources": {
                "spatial_memory": True,
                "temporal_memory": True,
                "semantic_temporal_map": True,
            },
            "snapshot_storage": {
                "durable": False,
                "backend": None,
                "policy": "ephemeral_read_through",
            },
        }


class StubSpatialMemory:
    def query_by_text(self, text: str, limit: int = 5) -> list[dict[str, Any]]:
        return [{"id": "frame_1", "metadata": {"frame_id": "frame_1"}}][:limit]


class FailingSpatialMemory:
    def query_by_text(self, text: str, limit: int = 5) -> list[dict[str, Any]]:
        raise RuntimeError("spatial query unavailable")


class StubTemporalMemory:
    def get_state(self) -> dict[str, Any]:
        return {"entity_count": 2, "entities": [{"id": "person_1"}, {"id": "chair_1"}]}

    def get_graph_db_stats(self) -> dict[str, Any]:
        return {"stats": {"entities": 2, "relations": 1}}


class StubSkillOutcomes:
    def get_recent_outcomes(
        self, limit: int = 5, skill_name: str = "", domain: str = ""
    ) -> list[dict[str, Any]]:
        return [{"skill_name": "navigate_with_text", "success": False}][:limit]


class StubCausalWorldModel:
    def get_recent_transitions(
        self,
        limit: int = 5,
        skill_name: str = "",
        domain: str = "",
        cause: str = "",
    ) -> list[dict[str, Any]]:
        return [{"skill_name": "navigate_with_text", "outcome_success": False}][:limit]

    def get_model_state(self) -> dict[str, Any]:
        return {
            "sample_count": 3,
            "persistence": {
                "autosave": True,
                "path": "/tmp/go2-world-model.json",
            },
        }


class StubSkillInterface:
    def get_skill_interface_snapshot(self, domain: str = "") -> dict[str, Any]:
        return {"available": True, "skill_count": 18, "skills": []}


def test_memory_backend_status_reports_wired_backends() -> None:
    status = _Go2MemoryBackendStatus()
    try:
        status._world_state = StubWorldState()  # type: ignore[assignment]
        status._spatial_memory = StubSpatialMemory()  # type: ignore[assignment]
        status._temporal_memory = StubTemporalMemory()  # type: ignore[assignment]
        status._skill_outcomes = StubSkillOutcomes()  # type: ignore[assignment]
        status._causal_world_model = StubCausalWorldModel()  # type: ignore[assignment]
        status._skill_interface = StubSkillInterface()  # type: ignore[assignment]

        result = status.memory_backend_status()

        assert result.success is True
        assert "Memory backends:" in result.message
        assert result.metadata["spatial_memory"]["wired"] is True
        assert result.metadata["spatial_memory"]["query_available"] is True
        assert result.metadata["spatial_memory"]["match_count_probe"] == 1
        assert result.metadata["temporal_memory"]["wired"] is True
        assert result.metadata["temporal_memory"]["entity_count"] == 2
        assert result.metadata["temporal_memory"]["graph_stats_available"] is True
        assert result.metadata["skill_outcomes"]["recent_count"] == 1
        assert result.metadata["causal_world_model"]["transition_count"] == 1
        assert result.metadata["causal_world_model"]["sample_count"] == 3
        assert result.metadata["causal_world_model"]["persistence_path"] == (
            "/tmp/go2-world-model.json"
        )
        assert result.metadata["skill_interface"]["skill_count"] == 18
        assert result.metadata["warnings"] == []
    finally:
        _stop_modules(status)


def test_memory_backend_status_reports_missing_backends() -> None:
    status = _Go2MemoryBackendStatus()
    try:
        result = status.memory_backend_status()

        assert result.success is True
        assert result.metadata["spatial_memory"]["wired"] is False
        assert result.metadata["temporal_memory"]["wired"] is False
        assert result.metadata["skill_outcomes"]["wired"] is False
        assert result.metadata["causal_world_model"]["wired"] is False
        assert result.metadata["skill_interface"]["wired"] is False
        assert result.metadata["warnings"] == []
    finally:
        _stop_modules(status)


def test_memory_backend_status_warns_without_failing_on_probe_errors() -> None:
    status = _Go2MemoryBackendStatus()
    try:
        status._spatial_memory = FailingSpatialMemory()  # type: ignore[assignment]

        result = status.memory_backend_status()

        assert result.success is True
        assert result.metadata["spatial_memory"]["wired"] is True
        assert result.metadata["spatial_memory"]["query_available"] is False
        assert result.metadata["warnings"] == ["spatial_memory: spatial query unavailable"]
    finally:
        _stop_modules(status)
