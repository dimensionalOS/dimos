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


from dimos.agents.mcp.mcp_client import McpClient
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.replanning_a_star.module import ReplanningAStarPlanner
from dimos.robot.pollen.microduck.blueprints.cockpit import (
    microduck_agentic_cockpit,
    microduck_cockpit,
)
from dimos.robot.pollen.microduck.skills import MicroduckSkills
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule
from dimos.web.relay_bridge.manifest import parse_manifest
from dimos.web.relay_bridge.relay_bridge_module import RelayBridgeModule


def _atoms(blueprint):  # type: ignore[no-untyped-def]
    return {atom.module.__name__: atom for atom in blueprint.active_blueprints}


def test_cockpit_blueprint_composes_sim_nav_and_three_panels() -> None:
    atoms = _atoms(microduck_cockpit)
    assert atoms["MujocoSimModule"].module is MujocoSimModule
    assert atoms["MujocoSimModule"].kwargs["enable_pointcloud"] is True
    for name in ("_MicroDuckCoordinator", "ReplanningAStarPlanner", "MovementManager"):
        assert name in atoms, name
    assert atoms["ReplanningAStarPlanner"].module is ReplanningAStarPlanner
    assert atoms["MovementManager"].module is MovementManager
    bridge = next(
        a for a in microduck_cockpit.active_blueprints if issubclass(a.module, RelayBridgeModule)
    )
    manifest = parse_manifest(bridge.kwargs["manifest"])
    assert [p.kind for p in manifest.panels] == ["video", "map2d", "teleop"]
    goal = next(c for c in manifest.channels if c.ch == "goal_request")
    assert (goal.dir, goal.encoding, goal.publish) == ("tx", "goal.json.v1", "shared")
    # The map click lands on the planner's goal_request by name + type.
    assert {(s.name, s.direction): s.type for s in bridge.streams}[
        ("goal_request", "out")
    ] is PoseStamped


def test_agentic_cockpit_adds_chat_and_skills() -> None:
    atoms = _atoms(microduck_agentic_cockpit)
    assert atoms["McpClient"].module is McpClient
    assert atoms["MicroduckSkills"].module is MicroduckSkills
    bridge = next(
        a
        for a in microduck_agentic_cockpit.active_blueprints
        if issubclass(a.module, RelayBridgeModule)
    )
    manifest = parse_manifest(bridge.kwargs["manifest"])
    assert [p.kind for p in manifest.panels] == ["video", "map2d", "teleop", "chat"]


def test_skills_are_declared() -> None:
    names = {n for n, f in vars(MicroduckSkills).items() if getattr(f, "__skill__", False)}
    assert {
        "go_to",
        "stop_moving",
        "where_am_i",
        "list_tricks",
        "perform",
        "sit",
        "stand_up",
    } <= names
