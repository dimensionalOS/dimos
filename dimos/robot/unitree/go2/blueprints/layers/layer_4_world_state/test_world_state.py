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

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.navigation.base import NavigationState
from dimos.robot.unitree.go2.blueprints.layers.layer_4_world_state.semantic_temporal_map import (
    _Go2SemanticTemporalMap,
)
from dimos.robot.unitree.go2.blueprints.layers.layer_4_world_state.structured_world_state import (
    _Go2StructuredWorldState,
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
                "distance": 0.2,
                "metadata": [{"pos_x": 1.0, "pos_y": 2.0, "label": text}],
            }
        ][:limit]


class StubTemporalMemory:
    def query(self, question: str) -> str:
        return f"temporal answer for {question}"

    def get_state(self) -> dict[str, Any]:
        return {"entity_count": 1}

    def get_entity_roster(self) -> list[dict[str, Any]]:
        return [{"id": "person_1"}]

    def get_rolling_summary(self) -> str:
        return "A person was recently near the hallway."

    def get_graph_db_stats(self) -> dict[str, Any]:
        return {"stats": {"entities": 1}}


class StubNavigation:
    def get_state(self) -> NavigationState:
        return NavigationState.FOLLOWING_PATH

    def is_goal_reached(self) -> bool:
        return False


class StubSemanticTemporalMap:
    def query_semantic_temporal_map(
        self, query: str = "", spatial_limit: int = 3
    ) -> dict[str, Any]:
        return {
            "query": query,
            "sources": {"spatial_memory": True, "temporal_memory": True},
            "spatial": {
                "available": True,
                "matches": [{"distance": 0.2, "metadata": [{"label": query}]}],
            },
            "temporal": {
                "available": True,
                "rolling_summary": "A person was recently near the hallway.",
            },
            "fused": {
                "available": True,
                "spatial_match_count": 1,
                "has_temporal_answer": False,
                "has_temporal_summary": True,
            },
        }


def test_semantic_temporal_map_combines_memory_sources() -> None:
    semantic_map = _Go2SemanticTemporalMap()
    try:
        semantic_map._spatial_memory = StubSpatialMemory()  # type: ignore[assignment]
        semantic_map._temporal_memory = StubTemporalMemory()  # type: ignore[assignment]

        result = semantic_map.query_semantic_temporal_map("find the hallway", spatial_limit=3)

        assert result["sources"]["spatial_memory"] is True
        assert result["sources"]["temporal_memory"] is True
        assert result["spatial"]["matches"][0]["distance"] == 0.2
        assert result["temporal"]["answer"] == "temporal answer for find the hallway"
        assert result["fused"]["spatial_match_count"] == 1
        assert result["fused"]["has_temporal_summary"] is True
    finally:
        _stop_modules(semantic_map)


def test_structured_world_state_returns_snapshot() -> None:
    world_state = _Go2StructuredWorldState()
    try:
        world_state._semantic_temporal_map = StubSemanticTemporalMap()  # type: ignore[assignment]
        world_state._navigation = StubNavigation()  # type: ignore[assignment]
        world_state._latest_odom = PoseStamped(position=[1.0, 2.0, 0.0], frame_id="map")

        snapshot = world_state.get_world_snapshot("find the hallway", spatial_limit=2)

        assert snapshot["sources"]["semantic_temporal_map"] is True
        assert snapshot["sources"]["spatial_memory"] is True
        assert snapshot["robot_state"]["navigation"]["state"] == "following_path"
        assert snapshot["robot_state"]["odom"]["position"]["x"] == 1.0
        assert snapshot["memory_state"]["spatial"]["matches"][0]["distance"] == 0.2
        assert snapshot["semantic_temporal_map"]["fused"]["spatial_match_count"] == 1
    finally:
        _stop_modules(world_state)
