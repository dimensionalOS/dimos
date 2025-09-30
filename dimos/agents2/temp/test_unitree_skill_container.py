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
Test file for UnitreeSkillContainer with agents2 framework.
Tests skill registration and basic functionality.
"""

import sys
import time
from pathlib import Path

# Add parent directories to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent.parent))

from dimos.agents2 import Agent
from dimos.agents2.spec import Model, Provider
from dimos.robot.unitree_webrtc.unitree_skill_container import UnitreeSkillContainer
from dimos.utils.logging_config import setup_logger

logger = setup_logger("test_unitree_skills")


def test_skill_container_creation():
    """Test that the skill container can be created and skills are registered."""
    print("\n=== Testing UnitreeSkillContainer Creation ===")

    # Create container without robot (for testing)
    container = UnitreeSkillContainer(robot=None)

    try:
        # Get available skills from the container
        skills = container.skills()

        print(f"Number of skills registered: {len(skills)}")
        print("\nAvailable skills:")
        for name, skill_config in list(skills.items())[:10]:  # Show first 10
            print(
                f"  - {name}: {skill_config.description if hasattr(skill_config, 'description') else 'No description'}"
            )
        if len(skills) > 10:
            print(f"  ... and {len(skills) - 10} more skills")

        return container, skills
    finally:
        # Ensure proper cleanup
        container._close_module()
        # Small delay to allow threads to finish cleanup
        time.sleep(0.1)


def test_agent_with_skills():
    """Test that an agent can be created with the skill container."""
    print("\n=== Testing Agent with Skills ===")

    # Create skill container
    container = UnitreeSkillContainer(robot=None)
    agent = None

    try:
        # Create agent with configuration passed directly
        agent = Agent(
            system_prompt="You are a helpful robot assistant that can control a Unitree Go2 robot.",
            model=Model.GPT_4O_MINI,
            provider=Provider.OPENAI,
        )

        # Register skills
        agent.register_skills(container)

        print("Agent created and skills registered successfully!")

        # Get tools to verify
        tools = agent.get_tools()
        print(f"Agent has access to {len(tools)} tools")

        return agent
    finally:
        # Ensure proper cleanup in order
        if agent:
            agent.stop()
        container._close_module()
        # Small delay to allow threads to finish cleanup
        time.sleep(0.1)


def test_skill_schemas():
    """Test that skill schemas are properly generated for LangChain."""
    print("\n=== Testing Skill Schemas ===")

    container = UnitreeSkillContainer(robot=None)

    try:
        skills = container.skills()

        # Check a few key skills (using snake_case names now)
        skill_names = ["move", "wait", "stand_up", "sit", "front_flip", "dance1"]

        for name in skill_names:
            if name in skills:
                skill_config = skills[name]
                print(f"\n{name} skill:")
                print(f"  Config: {skill_config}")
                if hasattr(skill_config, "schema"):
                    print(
                        f"  Schema keys: {skill_config.schema.keys() if skill_config.schema else 'None'}"
                    )
            else:
                print(f"\nWARNING: Skill '{name}' not found!")
    finally:
        # Ensure proper cleanup
        container._close_module()
        # Small delay to allow threads to finish cleanup
        time.sleep(0.1)
