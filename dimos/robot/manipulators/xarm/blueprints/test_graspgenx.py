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

from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.core.coordination.blueprints import Blueprint
from dimos.manipulation.grasping.grasp_gen_x import GraspGenXModule
from dimos.manipulation.pick_and_place_module import (
    PickAndPlaceModule,
    PickAndPlaceModuleConfig,
)
from dimos.perception.experimental.object_scene_registration import (
    ObjectSceneRegistrationModule,
)
from dimos.robot.manipulators.xarm.blueprints.agentic import xarm_graspgenx_agent
from dimos.robot.manipulators.xarm.blueprints.graspgenx import xarm_graspgenx
from dimos.robot.manipulators.xarm.blueprints.perception import xarm_perception
from dimos.robot.manipulators.xarm.grasp_config import (
    XARM_GRASP_FRAME_TO_TCP,
    XARM_GRIPPER_SWEEP,
    make_xarm_graspgenx_config,
)


def _module_kwargs(blueprint: Blueprint, module_type: type) -> dict[str, Any]:
    return next(atom.kwargs for atom in blueprint.blueprints if atom.module is module_type)


def _module_count(blueprint: Blueprint, module_type: type) -> int:
    return sum(atom.module is module_type for atom in blueprint.active_blueprints)


def test_xarm_graspgenx_geometry_is_explicit_and_import_safe() -> None:
    config = make_xarm_graspgenx_config()

    assert config.gripper == XARM_GRIPPER_SWEEP
    assert config.grasp_frame_to_tcp == XARM_GRASP_FRAME_TO_TCP
    assert config.gripper.extents_open == (0.085, 0.032, 0.067)
    assert config.gripper.extents_half_open == (0.0425, 0.032, 0.067)
    assert config.gripper.fingertip_depth == 0.162
    assert config.grasp_frame_to_tcp[2][3] == 0.172


def test_existing_xarm_perception_keeps_explicit_heuristic_fallback() -> None:
    config = PickAndPlaceModuleConfig(**_module_kwargs(xarm_perception, PickAndPlaceModule))

    assert config.heuristic_grasp_fallback is True
    assert _module_count(xarm_perception, GraspGenXModule) == 0


def test_xarm_graspgenx_composes_one_provider_of_each_kind() -> None:
    config = PickAndPlaceModuleConfig(**_module_kwargs(xarm_graspgenx, PickAndPlaceModule))

    assert _module_count(xarm_graspgenx, ObjectSceneRegistrationModule) == 1
    assert _module_count(xarm_graspgenx, GraspGenXModule) == 1
    assert _module_count(xarm_graspgenx, PickAndPlaceModule) == 1
    assert config.heuristic_grasp_fallback is False
    assert config.grasp_approach_vector == (0.0, 0.0, -1.0)
    assert config.grasp_verification.enabled is False


def test_xarm_graspgenx_agent_composes_one_mcp_pair() -> None:
    assert _module_count(xarm_graspgenx_agent, McpServer) == 1
    assert _module_count(xarm_graspgenx_agent, McpClient) == 1
    assert _module_count(xarm_graspgenx_agent, ObjectSceneRegistrationModule) == 1
    assert _module_count(xarm_graspgenx_agent, GraspGenXModule) == 1
    assert _module_count(xarm_graspgenx_agent, PickAndPlaceModule) == 1
