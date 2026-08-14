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

import pytest

from dimos.core.global_config import global_config

if global_config.simulation != "mujoco":
    pytest.skip("sim watering graph requires MuJoCo configuration", allow_module_level=True)

from dimos.robot.unitree.g1.blueprints.basic.unitree_g1_water_demo_sim import (
    unitree_g1_water_demo_sim,
)


def _atom(name: str) -> Any:
    (atom,) = [item for item in unitree_g1_water_demo_sim.active_blueprints if item.name == name]
    return atom


def _stream(instance: str, port: str) -> str:
    return unitree_g1_water_demo_sim.remapping_map.get((instance, port), port)


def test_demo_graph_has_one_task_and_no_navigation_modules() -> None:
    names = {atom.name for atom in unitree_g1_water_demo_sim.active_blueprints}
    required = {
        "mujocosimmodule",
        "ControlCoordinator",
        "simbodypose",
        "posetargetobservationmodule",
        "manipulationmodule",
        "basicpathfollower",
        "wateringtaskmodule",
        "websocketvismodule",
    }
    assert required <= names
    assert names - required <= {"rerunbridgemodule", "rerunwebsocketserver"}
    assert not names & {
        "movementmanager",
        "costmapper",
        "replanningastarplanner",
        "voxelgridmapper",
    }


def test_demo_uses_the_canonical_wbc_scene_and_waits_for_the_controller() -> None:
    backend = _atom("mujocosimmodule")

    assert backend.kwargs["address"]._lfs_filename == "mujoco_sim/g1_gear_wbc.xml"
    assert len(backend.kwargs["extra_mjcf"]) == 1
    assert backend.kwargs["headless"] is (global_config.viewer == "none")
    assert backend.kwargs["wait_for_control_command"] is True
    assert backend.kwargs["enable_color"] is False
    assert backend.kwargs["enable_depth"] is False
    assert backend.kwargs["enable_pointcloud"] is False


def test_sim_ground_truth_is_adapted_to_the_typed_target_contract() -> None:
    provider = _atom("simbodypose")
    adapter = _atom("posetargetobservationmodule")
    task = _atom("wateringtaskmodule")

    assert provider.kwargs["body_name"] == "plant_pot_1"
    assert adapter.kwargs == {
        "object_id": "plant_pot_1",
        "label": "plant pot",
        "source": "sim_ground_truth",
    }
    assert task.kwargs == {
        "target_id": "plant_pot_1",
        "motion_enabled": False,
        "approach_motion_enabled": True,
        "pour_motion_enabled": True,
    }
    assert _stream("simbodypose", "object_pose") == "object_pose"
    assert _stream("posetargetobservationmodule", "object_pose") == "object_pose"
    assert _stream("posetargetobservationmodule", "target_observation") == ("target_observation")
    assert _stream("wateringtaskmodule", "target_observation") == "target_observation"
    assert _stream("wateringtaskmodule", "base_pose") == "odom"
    assert _stream("wateringtaskmodule", "operator_command") == "tele_cmd_vel"
    assert _stream("wateringtaskmodule", "approach_path") == "path"
    assert _stream("wateringtaskmodule", "approach_goal") == "goal_request"
    assert _stream("wateringtaskmodule", "approach_command_path") == ("approach_command_path")
    assert _stream("basicpathfollower", "path") == "approach_command_path"
    assert _stream("basicpathfollower", "base_pose") == "odom"


def test_coordinator_arbitrates_teleop_and_autonomy_before_one_groot_task() -> None:
    assert _stream("ControlCoordinator", "operator_twist_command") == "tele_cmd_vel"
    assert _stream("ControlCoordinator", "autonomy_twist_command") == "autonomy_cmd_vel"
    assert _stream("basicpathfollower", "nav_cmd_vel") == "autonomy_cmd_vel"

    coordinator = _atom("ControlCoordinator")
    tasks = {task.name: task for task in coordinator.kwargs["tasks"]}
    watering = tasks["groot_wbc"]

    assert watering.priority == 50
    assert watering.stream_bind == {}
    assert watering.params["timeout"] == 0.25
    assert "teleop_groot_wbc" not in tasks

    sources = {source.name: source for source in coordinator.kwargs["twist_sources"]}
    assert sources["operator"].priority == 100
    assert sources["operator"].timeout == 0.35
    assert sources["autonomy"].priority == 50
    assert sources["autonomy"].timeout == 0.25
