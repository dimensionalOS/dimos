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

from typing import get_args, get_type_hints

from dimos.agents.code_policy import CodePolicyModule
from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.agents.skills.speak_skill import SpeakSkill
from dimos.agents.web_human_input import WebInput
from dimos.benchmark.agent_eval.blueprint import (
    unitree_go2_dimsim_external_pi_eval,
)
from dimos.benchmark.agent_eval.observation_recorder import (
    DEFAULT_AGENT_EVAL_RECORDING_PATH,
    AgentEvalObservationRecorder,
)
from dimos.navigation.replanning_a_star.module import ReplanningAStarPlanner
from dimos.perception.experimental.perceive_loop_skill import PerceiveLoopSkill
from dimos.perception.experimental.spatial_perception import SpatialMemory


def test_observation_recorder_has_exact_agent_visible_allowlist() -> None:
    assert set(AgentEvalObservationRecorder.__annotations__) == {
        "color_image",
        "odom",
        "global_map",
        "goal_reached",
        "config",
        "_last_odom_pose",
    }
    public_inputs = {
        name
        for name in AgentEvalObservationRecorder.__annotations__
        if not name.startswith("_") and name != "config"
    }
    assert public_inputs == {
        "color_image",
        "odom",
        "global_map",
        "goal_reached",
    }
    assert public_inputs.isdisjoint(
        {
            "oracle",
            "scene_oracle_view",
            "task_contract",
            "expected_outcome",
            "reset_request",
            "reset_acknowledgement",
            "native_result",
            "rubric",
        }
    )
    recorder_goal_reached_type = get_args(
        get_type_hints(AgentEvalObservationRecorder)["goal_reached"]
    )[0]
    planner_goal_reached_type = get_args(get_type_hints(ReplanningAStarPlanner)["goal_reached"])[0]
    assert recorder_goal_reached_type is planner_goal_reached_type


def test_external_pi_blueprint_has_one_code_policy_and_no_internal_agent() -> None:
    modules = [atom.module for atom in unitree_go2_dimsim_external_pi_eval.blueprints]

    assert modules.count(CodePolicyModule) == 1
    assert modules.count(AgentEvalObservationRecorder) == 1
    assert modules.count(SpatialMemory) == 1
    assert PerceiveLoopSkill not in modules
    assert modules.count(McpServer) == 1
    assert McpClient not in modules
    assert SpeakSkill not in modules
    assert WebInput not in modules
    assert all(not module.__module__.startswith("dimos.manipulation") for module in modules)
    assert all("viser" not in module.__module__ for module in modules)


def test_blueprint_binds_code_policy_to_allowlisted_recording() -> None:
    recorder = next(
        atom
        for atom in unitree_go2_dimsim_external_pi_eval.blueprints
        if atom.module is AgentEvalObservationRecorder
    )
    code_policy = next(
        atom
        for atom in unitree_go2_dimsim_external_pi_eval.blueprints
        if atom.module is CodePolicyModule
    )

    assert recorder.kwargs["db_path"] == DEFAULT_AGENT_EVAL_RECORDING_PATH
    assert recorder.kwargs["on_existing"] == "overwrite"
    assert code_policy.kwargs["recording_path"] == DEFAULT_AGENT_EVAL_RECORDING_PATH
    assert unitree_go2_dimsim_external_pi_eval.global_config_overrides["simulation"] == "dimsim"
