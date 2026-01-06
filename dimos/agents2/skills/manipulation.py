#!/usr/bin/env python3
# Copyright 2025 Dimensional Inc.
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

"""
Manipulation Skills Blueprints

Provides manipulation skill systems:
1. Skills-only blueprints (without hardware)
2. Complete system blueprints (hardware + skills + agent)

Skills layer includes:
- MoveAxisModule (directional movement commands)
- ManipulationSkillContainer (high-level skills for LLM agents)
- RobotCapabilities (basic robot functions like speech)

Complete systems combine hardware stack with skills and agent for
immediate interactive use.
"""

from dimos.agents2.agent import llm_agent
from dimos.agents2.cli.human import human_input
from dimos.agents2.skills.manipulation_skill_container import ManipulationSkillContainer
from dimos.core.blueprints import autoconnect
from dimos.manipulation.manipulation_blueprints import xarm7_manipulation
from dimos.manipulation.modules.move_axis import MoveAxisModule
from dimos.manipulation.modules.robot_capabilities import RobotCapabilities

# ============================================================================
# Main Manipulation Skills Blueprint
# ============================================================================

# This is the primary manipulation blueprint with full arm control capabilities
# and LLM agent for interactive use. Use this when you want agent-controlled
# manipulation with directional movement commands.
#
# To use with actual hardware, compose with manipulation hardware blueprints:
#   from dimos.manipulation.manipulation_blueprints import xarm7_manipulation
#   full_system = autoconnect(xarm7_manipulation, manipulation_skills)
manipulation_skills = autoconnect(
    RobotCapabilities.blueprint(),
    MoveAxisModule.blueprint(),
    ManipulationSkillContainer.blueprint(),
    llm_agent(
        system_prompt=(
            "You are a helpful robotic manipulation assistant. You can control a robot arm "
            "to move in different directions (up, down, left, right, forward, backward) and "
            "perform manipulation tasks. You can also greet people by name. When someone tells "
            "you their name, remember to use it when greeting them."
        )
    ),
    human_input(),
)

# ============================================================================
# Greetings Skill Blueprint
# ============================================================================

# Simple blueprint for just greeting capabilities. Useful for testing
# robot speech or as a minimal example of skill integration.
#
# The robot can greet people by name and remembers names when told.
greetings_skill_blueprint = autoconnect(
    RobotCapabilities.blueprint(),
    ManipulationSkillContainer.blueprint(),
    llm_agent(
        system_prompt=(
            "You are a friendly robot that can greet people. When someone tells you their name, "
            "use the greet skill with their name. For example, if they say 'My name is Alice', "
            "call greet('Alice'). Be warm and personable in your interactions."
        )
    ),
    human_input(),
)

# ============================================================================
# Complete System Blueprint (Hardware + Skills + Agent)
# ============================================================================

# XArm7 complete system: hardware + skills + agent
# This is the primary blueprint for full manipulation capabilities
xarm7_manipulation_agent = autoconnect(
    xarm7_manipulation,      # Hardware: driver + planner + controller
    manipulation_skills,     # Skills: directional movement + agent
)

# ============================================================================
# Exported Blueprints
# ============================================================================

__all__ = [
    "manipulation_skills",
    "greetings_skill_blueprint",
    "xarm7_manipulation_agent",
]
