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

import pytest

from dimos.core.coordination.blueprints import Blueprint, BlueprintAtom, ModuleRef
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.navigation.base import NavigationState
from dimos.robot.unitree.go2.blueprints.layers.layer_3_agent_brain.context_provider import (
    _Go2ContextProvider,
)
from dimos.robot.unitree.go2.blueprints.layers.layer_4_world_state.semantic_temporal_map import (
    _Go2SemanticTemporalMap,
)
from dimos.robot.unitree.go2.blueprints.layers.layer_4_world_state.structured_world_state import (
    _Go2StructuredWorldState,
)
from dimos.robot.unitree.go2.blueprints.layers.layer_4_world_state.world_state_spec import (
    SemanticTemporalMapSpec,
    WorldStateSpec,
)
from dimos.robot.unitree.go2.blueprints.layers.layer_6_robot_body.robot_body_spec import (
    RobotBodyStateSpec,
)
from dimos.robot.unitree.go2.blueprints.layers.layer_6_robot_body.robot_body_state import (
    _Go2RobotBodyState,
)
from dimos.spec.utils import spec_annotation_compliance, spec_structural_compliance


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
        return {
            "entity_count": 1,
            "entities": [
                {
                    "id": "cup_1",
                    "type": "object",
                    "descriptor": "blue cup",
                    "confidence": 0.7,
                }
            ],
        }

    def get_entity_roster(self) -> list[dict[str, Any]]:
        return [{"id": "person_1", "type": "person", "descriptor": "near the hallway"}]

    def get_rolling_summary(self) -> str:
        return "A person was recently near the hallway."

    def get_graph_db_stats(self) -> dict[str, Any]:
        return {
            "stats": {"entities": 1},
            "entities": [
                {
                    "entity_id": "hallway_1",
                    "entity_type": "location",
                    "descriptor": "main hallway",
                    "last_seen_ts": 12.0,
                    "metadata": {"world_x": 1.0, "world_y": 2.0, "world_z": 0.0},
                }
            ],
        }


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
                "matches": [
                    {
                        "distance": 0.2,
                        "metadata": [
                            {"label": query, "pos_x": 1.0, "pos_y": 2.0, "pos_z": 0.0}
                        ],
                    }
                ],
            },
            "temporal": {
                "available": True,
                "rolling_summary": "A person was recently near the hallway.",
                "entity_roster": [
                    {
                        "id": "person_1",
                        "type": "person",
                        "descriptor": "near the hallway",
                    }
                ],
            },
            "fused": {
                "available": True,
                "spatial_match_count": 1,
                "has_temporal_answer": False,
                "has_temporal_summary": True,
                "entry_count": 2,
                "entries": [
                    {
                        "entity": {
                            "id": query,
                            "type": "spatial_observation",
                            "description": query,
                        },
                        "location": {"x": 1.0, "y": 2.0, "z": 0.0, "label": query},
                        "time": None,
                        "evidence_source": "spatial_memory",
                        "confidence": 0.8,
                    },
                    {
                        "entity": {
                            "id": "person_1",
                            "type": "person",
                            "description": "near the hallway",
                        },
                        "location": None,
                        "time": None,
                        "evidence_source": "temporal_memory.entity_roster",
                        "confidence": None,
                    },
                ],
            },
        }


class StubRobotBody:
    def get_robot_body_snapshot(self) -> dict[str, Any]:
        return {
            "available": True,
            "version": "v1",
            "connection": {"available": True, "mode": "replay"},
            "sensors": {"odom": {"available": True, "count": 3}},
            "local_policy": {"available": True, "obstacle_avoidance": True},
            "safety": {"available": True, "body_pose_available": True},
        }

    def get_connection_state(self) -> dict[str, Any]:
        return {"available": True, "mode": "replay"}

    def get_sensor_state(self) -> dict[str, Any]:
        return {"odom": {"available": True, "count": 3}}

    def get_local_policy_state(self) -> dict[str, Any]:
        return {"available": True, "obstacle_avoidance": True}


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
        assert result["fused"]["entry_count"] == 4
        assert result["fused"]["entries"][0]["entity"]["id"] == "find the hallway"
        assert result["fused"]["entries"][0]["location"]["x"] == 1.0
        assert result["fused"]["entries"][0]["evidence_source"] == "spatial_memory"
        assert result["fused"]["entries"][0]["confidence"] == 0.8
        assert any(
            entry["entity"]["id"] == "hallway_1"
            and entry["evidence_source"] == "temporal_memory.graph"
            for entry in result["fused"]["entries"]
        )
    finally:
        _stop_modules(semantic_map)


def test_structured_world_state_returns_snapshot() -> None:
    world_state = _Go2StructuredWorldState()
    try:
        world_state._semantic_temporal_map = StubSemanticTemporalMap()  # type: ignore[assignment]
        world_state._navigation = StubNavigation()  # type: ignore[assignment]
        world_state._robot_body = StubRobotBody()  # type: ignore[assignment]
        world_state._latest_odom = PoseStamped(position=[1.0, 2.0, 0.0], frame_id="map")

        snapshot = world_state.get_world_snapshot("find the hallway", spatial_limit=2)

        assert snapshot["sources"]["semantic_temporal_map"] is True
        assert snapshot["sources"]["robot_body"] is True
        assert snapshot["sources"]["spatial_memory"] is True
        assert snapshot["robot_state"]["navigation"]["state"] == "following_path"
        assert snapshot["robot_state"]["odom"]["position"]["x"] == 1.0
        assert snapshot["robot_state"]["connection"]["mode"] == "replay"
        assert snapshot["robot_state"]["sensors"]["odom"]["count"] == 3
        assert snapshot["robot_state"]["safety"]["body_pose_available"] is True
        assert snapshot["memory_state"]["spatial"]["matches"][0]["distance"] == 0.2
        assert snapshot["memory_state"]["named_objects"][0]["id"] == "person_1"
        assert snapshot["memory_state"]["named_locations"][0]["name"] == "find the hallway"
        assert snapshot["memory_state"]["summary"]["named_object_count"] == 1
        assert snapshot["memory_state"]["summary"]["named_location_count"] == 1
        assert snapshot["semantic_temporal_map"]["fused"]["spatial_match_count"] == 1
        assert snapshot["semantic_temporal_map"]["fused"]["entries"][0]["location"]["x"] == 1.0
        assert snapshot["snapshot_storage"]["durable"] is False
        assert snapshot["snapshot_storage"]["policy"] == "ephemeral_read_through"
    finally:
        _stop_modules(world_state)


def test_go2_agentic_blueprint_wires_layer3_to_layer4_rpc_path() -> None:
    try:
        from dimos.robot.unitree.go2.blueprints.agentic.unitree_go2_agentic import (
            unitree_go2_agentic,
        )
    except (ImportError, ModuleNotFoundError, OSError) as exc:
        pytest.skip(f"Full Go2 agentic blueprint dependencies are unavailable: {exc}")

    context_atom = _atom_for(unitree_go2_agentic, _Go2ContextProvider)
    world_ref = _module_ref(context_atom, "_world_state")
    assert world_ref.spec is WorldStateSpec
    assert world_ref.optional is True
    assert _single_valid_provider(unitree_go2_agentic, context_atom, world_ref) is (
        _Go2StructuredWorldState
    )

    world_atom = _atom_for(unitree_go2_agentic, _Go2StructuredWorldState)
    semantic_ref = _module_ref(world_atom, "_semantic_temporal_map")
    assert semantic_ref.spec is SemanticTemporalMapSpec
    assert semantic_ref.optional is True
    assert _single_valid_provider(unitree_go2_agentic, world_atom, semantic_ref) is (
        _Go2SemanticTemporalMap
    )

    body_ref = _module_ref(world_atom, "_robot_body")
    assert body_ref.spec is RobotBodyStateSpec
    assert body_ref.optional is True
    assert _single_valid_provider(unitree_go2_agentic, world_atom, body_ref) is (
        _Go2RobotBodyState
    )


def _atom_for(blueprint: Blueprint, module: type[object]) -> BlueprintAtom:
    matches = [atom for atom in blueprint.active_blueprints if atom.module is module]
    assert len(matches) == 1
    return matches[0]


def _module_ref(atom: BlueprintAtom, name: str) -> ModuleRef:
    matches = [ref for ref in atom.module_refs if ref.name == name]
    assert len(matches) == 1
    return matches[0]


def _single_valid_provider(
    blueprint: Blueprint,
    consumer_atom: BlueprintAtom,
    module_ref: ModuleRef,
) -> type[object]:
    providers = [
        atom.module
        for atom in blueprint.active_blueprints
        if atom != consumer_atom
        and spec_structural_compliance(atom.module, module_ref.spec)
        and spec_annotation_compliance(atom.module, module_ref.spec)
    ]
    assert len(providers) == 1
    return providers[0]
