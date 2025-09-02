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
Test that event loop handling works correctly in both Dask and non-Dask environments.
"""

import os
import sys
from pathlib import Path

# Add parent directories to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent.parent))

from dimos.robot.unitree_webrtc.unitree_skill_container import UnitreeSkillContainer
from dimos.agents2 import Agent
from dimos.agents2.spec import Model, Provider


def test_non_dask():
    """Test agent outside of Dask."""
    print("\n=== Testing Non-Dask Environment ===")

    # Mock API key to avoid that error
    os.environ["OPENAI_API_KEY"] = "test-key-12345"

    try:
        container = UnitreeSkillContainer(robot=None)
        agent = Agent(system_prompt="Test agent", model=Model.GPT_4O_MINI, provider=Provider.OPENAI)
        agent.register_skills(container)

        print("Starting agent (should start event loop in thread)...")
        agent.start()

        # Check if loop is set
        if agent._loop:
            print(f"Event loop type: {type(agent._loop).__name__}")
            if hasattr(agent._loop, "is_running"):
                print(f"Event loop running: {agent._loop.is_running()}")

        print("✓ Non-Dask test passed")
        agent.stop()

    except Exception as e:
        print(f"✗ Non-Dask test failed: {e}")
    finally:
        # Clean up mock key
        del os.environ["OPENAI_API_KEY"]


def test_with_dask():
    """Test agent inside Dask."""
    print("\n=== Testing Dask Environment ===")

    # Mock API key
    os.environ["OPENAI_API_KEY"] = "test-key-12345"

    try:
        from dimos.core import start

        print("Starting Dask cluster...")
        dimos = start(2)

        # Create container directly (not a Module)
        container = UnitreeSkillContainer(robot=None)

        print("Deploying agent as Module...")
        agent = dimos.deploy(
            Agent, system_prompt="Test agent", model=Model.GPT_4O_MINI, provider=Provider.OPENAI
        )

        print("Registering skills and starting agent...")
        agent.register_skills(container)
        agent.start()

        print("✓ Dask test passed - no AsyncIOMainLoop error!")

        agent.stop()
        dimos.stop()

    except Exception as e:
        print(f"✗ Dask test failed: {e}")
        import traceback

        traceback.print_exc()
    finally:
        # Clean up
        if "OPENAI_API_KEY" in os.environ:
            del os.environ["OPENAI_API_KEY"]


def main():
    print("=" * 60)
    print("Event Loop Handling Test")
    print("=" * 60)

    # Test 1: Outside Dask
    test_non_dask()

    # Test 2: Inside Dask
    test_with_dask()

    print("\n" + "=" * 60)
    print("All tests complete!")
    print("=" * 60)


if __name__ == "__main__":
    main()
