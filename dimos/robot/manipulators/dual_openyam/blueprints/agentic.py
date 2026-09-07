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

"""Agentic Dual OpenYAM simulation blueprints."""

from __future__ import annotations

from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.manipulators.common.agent_prompts import (
    BIMANUAL_MANIPULATION_AGENT_SYSTEM_PROMPT,
)
from dimos.robot.manipulators.dual_openyam.blueprints.sim_learning import (
    build_dual_openyam_sim_rollout,
)
from dimos.robot.manipulators.dual_openyam.blueprints.simulation import (
    dual_openyam_sim_pick_place,
)
from dimos.robot.manipulators.dual_openyam.learning import DUAL_OPENYAM_SIM_TASK
from dimos.robot.manipulators.dual_openyam.visualization import sim_camera_layout
from dimos.visualization.rerun.bridge import RerunBridgeModule

dual_openyam_sim_agent = autoconnect(
    dual_openyam_sim_pick_place,
    RerunBridgeModule.blueprint(blueprint=sim_camera_layout),
    McpServer.blueprint(),
    McpClient.blueprint(system_prompt=BIMANUAL_MANIPULATION_AGENT_SYSTEM_PROMPT),
)


dual_openyam_sim_policy_agent = autoconnect(
    build_dual_openyam_sim_rollout(
        artifact="outputs/dual-openyam-act/checkpoints/last/pretrained_model",
        task=DUAL_OPENYAM_SIM_TASK,
        device="cuda",
    ),
    RerunBridgeModule.blueprint(blueprint=sim_camera_layout),
    McpServer.blueprint(),
    McpClient.blueprint(
        system_prompt=BIMANUAL_MANIPULATION_AGENT_SYSTEM_PROMPT
        + """
Learned policy controls: run_policy, stop_policy, policy_status.
ACT performs only the configured task; run_policy accepts no language goal.
Call stop_policy and confirm active=false before any classical motion or scene reset.
A started policy is not a completed task. Inspect the scene before reporting success.
"""
    ),
)
