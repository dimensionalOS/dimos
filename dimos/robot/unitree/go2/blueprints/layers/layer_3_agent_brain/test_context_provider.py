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
import json

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.navigation.base import NavigationState
from dimos.robot.unitree.go2.blueprints.layers.layer_3_agent_brain.context_provider import (
    _Go2ContextProvider,
)


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


def test_get_context_aggregates_available_sources() -> None:
    provider = _Go2ContextProvider()
    provider._spatial_memory = StubSpatialMemory()  # type: ignore[assignment]
    provider._temporal_memory = StubTemporalMemory()  # type: ignore[assignment]
    provider._navigation = StubNavigation()  # type: ignore[assignment]
    provider._skill_outcomes = StubSkillOutcomes()  # type: ignore[assignment]
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

    encoded = json.loads(result.agent_encode()[0]["text"])
    assert encoded["success"] is True
    assert encoded["metadata"]["task"] == "find the person"


def test_get_context_handles_missing_optional_sources() -> None:
    provider = _Go2ContextProvider()

    result = provider.get_context("walk forward")

    assert result.success is True
    assert result.metadata["sources"]["spatial_memory"] is False
    assert result.metadata["sources"]["temporal_memory"] is False
    assert result.metadata["robot_state"]["odom"] is None
    assert "Spatial memory: unavailable" in result.message


def test_get_context_requires_task() -> None:
    provider = _Go2ContextProvider()

    result = provider.get_context("   ")

    assert result.success is False
    assert result.error_code == "INVALID_INPUT"
