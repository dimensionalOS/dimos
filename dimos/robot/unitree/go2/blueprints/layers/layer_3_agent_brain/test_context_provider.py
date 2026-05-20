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

import json
from typing import Any

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.navigation.base import NavigationState
from dimos.robot.unitree.go2.blueprints.layers.layer_3_agent_brain.context_provider import (
    _Go2ContextProvider,
)


def _stop_modules(*modules: object) -> None:
    for module in modules:
        stop = getattr(module, "stop", None)
        if stop is not None:
            stop()


class StubSpatialMemory:
    def query_by_text(self, text: str, limit: int = 5) -> list[dict[str, Any]]:
        return [
            {
                "distance": 0.12,
                "metadata": [{"pos_x": 1.5, "pos_y": -0.2, "label": text, "_raw": b"bytes"}],
            }
        ][:limit]


class StubTemporalMemory:
    def query(self, question: str) -> str:
        return f"answer to {question}"

    def get_state(self) -> dict[str, Any]:
        return {"entity_count": 2, "entities": [{"id": "person_1"}]}

    def get_entity_roster(self) -> list[dict[str, Any]]:
        return [{"id": "person_1"}]

    def get_rolling_summary(self) -> str:
        return "A person was recently visible near the hallway."

    def get_graph_db_stats(self) -> dict[str, Any]:
        return {"stats": {"entities": 1}}


class StubNavigation:
    def get_state(self) -> NavigationState:
        return NavigationState.FOLLOWING_PATH

    def is_goal_reached(self) -> bool:
        return False


class StubSkillOutcomes:
    def get_recent_outcomes(
        self, limit: int = 5, skill_name: str = "", domain: str = ""
    ) -> list[dict[str, Any]]:
        return [
            {
                "skill_name": "navigate_with_text",
                "success": False,
                "domain": "navigation",
                "error_code": "EXECUTION_FAILED",
            }
        ][:limit]


class StubSkillInterface:
    def get_skill_interface_snapshot(self, domain: str = "") -> dict[str, Any]:
        return {
            "available": True,
            "version": "v1",
            "domain_filter": domain,
            "domains": ["navigation"],
            "skill_count": 1,
            "skills": [
                {
                    "skill_name": "navigate_with_text",
                    "domain": "navigation",
                    "required_args": ["query"],
                    "motion_sensitive": True,
                }
            ],
        }

    def get_skill_contract(self, skill_name: str) -> dict[str, Any] | None:
        if skill_name == "navigate_with_text":
            return self.get_skill_interface_snapshot()["skills"][0]
        return None

    def validate_skill_request(self, skill_name: str, args_json: str = "{}") -> dict[str, Any]:
        return {"valid": True, "skill_name": skill_name, "errors": [], "warnings": []}


class StubCausalWorldModel:
    def get_recent_transitions(
        self,
        limit: int = 5,
        skill_name: str = "",
        domain: str = "",
        cause: str = "",
    ) -> list[dict[str, Any]]:
        return [
            {
                "skill_name": "navigate_with_text",
                "domain": "navigation",
                "outcome_success": False,
                "inferred_cause": "semantic_map_missing_target",
            }
        ][:limit]


class StubWorldState:
    def get_world_snapshot(self, task: str = "", spatial_limit: int = 3) -> dict[str, Any]:
        return {
            "task": task,
            "sources": {
                "odom": True,
                "navigation": True,
                "spatial_memory": True,
                "temporal_memory": True,
                "semantic_temporal_map": True,
                "runtime": True,
            },
            "runtime": {"mode": "replay", "simulation": False, "replay": True},
            "robot_state": {
                "odom": {
                    "frame_id": "map",
                    "timestamp": 1.0,
                    "position": {"x": 3.0, "y": 4.0, "z": 0.0},
                    "yaw_degrees": 0.0,
                },
                "navigation": {"state": "following_path", "goal_reached": False},
            },
            "memory_state": {
                "spatial": {"available": True, "matches": [{"distance": 0.3}]},
                "temporal": {
                    "available": True,
                    "rolling_summary": "The hallway was recently observed.",
                },
            },
            "semantic_temporal_map": {
                "fused": {"available": True, "spatial_match_count": 1}
            },
        }

    def get_robot_state(self) -> dict[str, Any]:
        return {}

    def get_runtime_state(self) -> dict[str, Any]:
        return {}

    def get_memory_state(self, task: str = "", spatial_limit: int = 3) -> dict[str, Any]:
        return {}


def test_get_context_aggregates_available_sources() -> None:
    provider = _Go2ContextProvider()
    try:
        provider._spatial_memory = StubSpatialMemory()  # type: ignore[assignment]
        provider._temporal_memory = StubTemporalMemory()  # type: ignore[assignment]
        provider._navigation = StubNavigation()  # type: ignore[assignment]
        provider._skill_outcomes = StubSkillOutcomes()  # type: ignore[assignment]
        provider._skill_interface = StubSkillInterface()  # type: ignore[assignment]
        provider._causal_world_model = StubCausalWorldModel()  # type: ignore[assignment]
        provider._latest_odom = PoseStamped(position=[1.0, 2.0, 0.0], frame_id="map")

        result = provider.get_context("find the person", focus="navigation")

        assert result.success is True
        assert "Task: find the person" in result.message
        assert result.metadata["sources"]["spatial_memory"] is True
        assert result.metadata["sources"]["temporal_memory"] is True
        assert result.metadata["robot_state"]["navigation"]["state"] == "following_path"
        assert result.metadata["world_state"]["spatial"]["matches"][0]["distance"] == 0.12
        assert result.metadata["world_state"]["temporal"]["rolling_summary"]
        assert result.metadata["skill_state"]["recent_outcomes"][0]["success"] is False
        assert result.metadata["skill_state"]["interface"]["skill_count"] == 1
        assert "Skill interface: 1 contract(s)" in result.message
        assert result.metadata["causal_state"]["recent_transitions"][0]["outcome_success"] is False

        encoded = json.loads(result.agent_encode()[0]["text"])
        assert encoded["success"] is True
        assert encoded["metadata"]["task"] == "find the person"
    finally:
        _stop_modules(provider)


def test_get_context_handles_missing_optional_sources() -> None:
    provider = _Go2ContextProvider()
    try:
        result = provider.get_context("walk forward")

        assert result.success is True
        assert result.metadata["sources"]["spatial_memory"] is False
        assert result.metadata["sources"]["temporal_memory"] is False
        assert result.metadata["robot_state"]["odom"] is None
        assert "Spatial memory: unavailable" in result.message
    finally:
        _stop_modules(provider)


def test_get_context_requires_task() -> None:
    provider = _Go2ContextProvider()
    try:
        result = provider.get_context("   ")

        assert result.success is False
        assert result.error_code == "INVALID_INPUT"
    finally:
        _stop_modules(provider)


def test_get_context_prefers_layer4_structured_world_state() -> None:
    provider = _Go2ContextProvider()
    try:
        provider._world_state = StubWorldState()  # type: ignore[assignment]

        result = provider.get_context("find the hallway")

        assert result.success is True
        assert result.metadata["sources"]["structured_world_state"] is True
        assert result.metadata["sources"]["spatial_memory"] is True
        assert result.metadata["runtime"]["mode"] == "replay"
        assert result.metadata["robot_state"]["odom"]["position"]["x"] == 3.0
        assert result.metadata["world_state"]["source"] == "structured_world_state"
        assert result.metadata["world_state"]["spatial"]["matches"][0]["distance"] == 0.3
    finally:
        _stop_modules(provider)
