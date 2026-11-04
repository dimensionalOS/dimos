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

"""The Go2's agent skills behind an MCP server: observe, person following, sport commands and move_to,
speech, web text input, and spatial memory with its perceive loop, driven by the MCP client's LLM, as the old
unitree-go2-agentic was.

Everything is imported when the blueprint is built, so the Go2 runs without the agent extras installed."""

from __future__ import annotations

from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.robot.unitree.go2.config import front_camera_info
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def agentic() -> Blueprint:
    """The skills, the MCP server and its client, or nothing (logged) when their dependencies aren't installed."""
    try:
        from dimos.agents.mcp.mcp_client import McpClient
        from dimos.agents.mcp.mcp_server import McpServer
        from dimos.agents.skills.observe_skill import ObserveSkill
        from dimos.agents.skills.person_follow import PersonFollowSkillContainer
        from dimos.agents.skills.speak_skill import SpeakSkill
        from dimos.agents.web_human_input import WebInput
        from dimos.perception.experimental.perceive_loop_skill import PerceiveLoopSkill
        from dimos.perception.experimental.spatial_perception import SpatialMemory
        from dimos.robot.unitree.unitree_skill_container import UnitreeSkillContainer
    except ImportError as error:
        logger.warning(f"Go2 agent skills off, their dependencies aren't installed: {error}")
        return autoconnect()
    return autoconnect(
        McpServer.blueprint(),
        McpClient.blueprint(),
        ObserveSkill.blueprint(),
        PersonFollowSkillContainer.blueprint(camera_info=front_camera_info()),
        UnitreeSkillContainer.blueprint(),
        WebInput.blueprint(),
        SpeakSkill.blueprint(),
        SpatialMemory.blueprint(),
        PerceiveLoopSkill.blueprint(),
    )
