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
Test script to debug agent query issues.
Shows different ways to call the agent and handle async.
"""

import asyncio
import os
from pathlib import Path
import sys
import time

from dotenv import load_dotenv

# Add parent directories to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent.parent))

from dimos.agents2 import Agent
from dimos.agents2.spec import Model, Provider
from dimos.robot.unitree_webrtc.unitree_skill_container import UnitreeSkillContainer
from dimos.utils.logging_config import setup_logger

logger = setup_logger("test_agent_query")

# Load environment variables
load_dotenv()


async def test_async_query():
    """Test agent query using async/await pattern."""
    print("\n=== Testing Async Query ===\n")

    # Create skill container
    container = UnitreeSkillContainer(robot=None)

    # Create agent
    agent = Agent(
        system_prompt="You are a helpful robot assistant. List 3 skills you can do.",
        model=Model.GPT_4O_MINI,
        provider=Provider.OPENAI,
    )

    # Register skills and start
    agent.register_skills(container)
    agent.start()

    # Query asynchronously
    logger.info("Sending async query...")
    future = agent.query_async("Hello! What skills do you have?")

    # Wait for result
    logger.info("Waiting for response...")
    await asyncio.sleep(10)  # Give it time to process

    # Check if future is done
    if hasattr(future, "done") and future.done():
        try:
            result = future.result()
            logger.info(f"Got result: {result}")
        except Exception as e:
            logger.error(f"Future failed: {e}")
    else:
        logger.warning("Future not completed yet")

    agent.stop()

    return future


def test_sync_query_with_thread() -> None:
    """Test agent query using threading for the event loop."""
    print("\n=== Testing Sync Query with Thread ===\n")

    import threading

    # Create skill container
    container = UnitreeSkillContainer(robot=None)

    # Create agent
    agent = Agent(
        system_prompt="You are a helpful robot assistant. List 3 skills you can do.",
        model=Model.GPT_4O_MINI,
        provider=Provider.OPENAI,
    )

    # Register skills and start
    agent.register_skills(container)
    agent.start()

    # Track the thread we might create
    loop_thread = None

    # The agent's event loop should be running in the Module's thread
    # Let's check if it's running
    if agent._loop and agent._loop.is_running():
        logger.info("Agent's event loop is running")
    else:
        logger.warning("Agent's event loop is NOT running - this is the problem!")

        # Try to run the loop in a thread
        def run_loop() -> None:
            asyncio.set_event_loop(agent._loop)
            agent._loop.run_forever()

        loop_thread = threading.Thread(target=run_loop, daemon=False, name="EventLoopThread")
        loop_thread.start()
        time.sleep(1)  # Give loop time to start
        logger.info("Started event loop in thread")

    # Now try the query
    try:
        logger.info("Sending sync query...")
        result = agent.query("Hello! What skills do you have?")
        logger.info(f"Got result: {result}")
    except Exception as e:
        logger.error(f"Query failed: {e}")
        import traceback

        traceback.print_exc()

    agent.stop()

    # Then stop the manually created event loop thread if we created one
    if loop_thread and loop_thread.is_alive():
        logger.info("Stopping manually created event loop thread...")
        # Stop the event loop
        if agent._loop and agent._loop.is_running():
            agent._loop.call_soon_threadsafe(agent._loop.stop)
        # Wait for thread to finish
        loop_thread.join(timeout=5)
        if loop_thread.is_alive():
            logger.warning("Thread did not stop cleanly within timeout")

    # Finally close the container
    container._close_module()


# def test_with_real_module_system():
#     """Test using the real DimOS module system (like in test_agent.py)."""
#     print("\n=== Testing with Module System ===\n")

#     from dimos.core import start

#     # Start the DimOS system
#     dimos = start(2)

#     # Deploy container and agent as modules
#     container = dimos.deploy(UnitreeSkillContainer, robot=None)
#     agent = dimos.deploy(
#         Agent,
#         system_prompt="You are a helpful robot assistant. List 3 skills you can do.",
#         model=Model.GPT_4O_MINI,
#         provider=Provider.OPENAI,
#     )

#     # Register skills
#     agent.register_skills(container)
#     agent.start()

#     # Query
#     try:
#         logger.info("Sending query through module system...")
#         future = agent.query_async("Hello! What skills do you have?")

#         # In the module system, the loop should be running
#         time.sleep(5)  # Wait for processing

#         if hasattr(future, "result"):
#             result = future.result(timeout=10)
#             logger.info(f"Got result: {result}")
#     except Exception as e:
#         logger.error(f"Query failed: {e}")

#     # Clean up
#     agent.stop()
#     dimos.stop()


def main() -> None:
    """Run tests based on available API key."""

    if not os.getenv("OPENAI_API_KEY"):
        print("ERROR: OPENAI_API_KEY not set")
        print("Please set your OpenAI API key to test the agent")
        sys.exit(1)

    print("=" * 60)
    print("Agent Query Testing")
    print("=" * 60)

    # Test 1: Async query
    try:
        asyncio.run(test_async_query())
    except Exception as e:
        logger.error(f"Async test failed: {e}")

    # Test 2: Sync query with threading
    try:
        test_sync_query_with_thread()
    except Exception as e:
        logger.error(f"Sync test failed: {e}")

    # Test 3: Module system (optional - more complex)
    # try:
    #     test_with_real_module_system()
    # except Exception as e:
    #     logger.error(f"Module test failed: {e}")

    print("\n" + "=" * 60)
    print("Testing complete")
    print("=" * 60)


if __name__ == "__main__":
    main()
