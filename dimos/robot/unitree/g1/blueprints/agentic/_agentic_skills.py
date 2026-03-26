#!/usr/bin/env python3
# Copyright 2025-2026 Dimensional Inc.
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

"""Agentic skills used by higher-level G1 blueprints."""

from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.agents.skills.navigation import NavigationSkillContainer
from dimos.agents.skills.speak_skill import SpeakSkill
from dimos.core.blueprints import autoconnect
from dimos.robot.unitree.g1.skill_container import UnitreeG1SkillContainer
from dimos.robot.unitree.g1.system_prompt import G1_SYSTEM_PROMPT

_agentic_skills = autoconnect(
    McpServer.blueprint(),
    McpClient.blueprint(system_prompt=G1_SYSTEM_PROMPT),
    NavigationSkillContainer.blueprint(),
    SpeakSkill.blueprint(),
    UnitreeG1SkillContainer.blueprint(),
)

__all__ = ["_agentic_skills"]
