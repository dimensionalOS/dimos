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
Simple test to verify the agent query works with minimal setup.
"""

import os
import sys
from pathlib import Path
from dotenv import load_dotenv

# Add parent directories to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent.parent))

from dimos.robot.unitree_webrtc.unitree_skill_container import UnitreeSkillContainer
from dimos.agents2 import Agent
from dimos.agents2.spec import Model, Provider

# Load environment variables
load_dotenv()


def main():
    """Simple sync test."""

    if not os.getenv("OPENAI_API_KEY"):
        print("ERROR: OPENAI_API_KEY not set")
        return

    print("Creating agent...")

    # Create container and agent
    container = UnitreeSkillContainer(robot=None)
    agent = Agent(
        system_prompt="You are a helpful robot. Answer concisely.",
        model=Model.GPT_4O_MINI,
        provider=Provider.OPENAI,
    )

    # Register and start
    agent.register_skills(container)
    agent.start()  # This now ensures the event loop is running

    print("Agent started. Testing query...")

    # Simple sync query - should just work now
    try:
        result = agent.query("What are 3 skills you can do?")
        print(f"\nAgent response:\n{result}")
    except Exception as e:
        print(f"Query failed: {e}")

    # Clean up
    agent.stop()
    print("\nDone!")


if __name__ == "__main__":
    main()
