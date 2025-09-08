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
Async version of the Unitree run file for agents2.
Properly handles the async nature of the agent.
"""

import asyncio
import os
import sys
from pathlib import Path
from dotenv import load_dotenv

# Add parent directories to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent.parent))

from dimos.robot.unitree_webrtc.unitree_go2 import UnitreeGo2
from dimos.robot.unitree_webrtc.unitree_skill_container import UnitreeSkillContainer
from dimos.agents2 import Agent
from dimos.agents2.spec import Model, Provider
from dimos.utils.logging_config import setup_logger

logger = setup_logger("run_unitree_async")

# Load environment variables
load_dotenv()

# System prompt path
SYSTEM_PROMPT_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__)))),
    "assets/agent/prompt.txt",
)


async def handle_query(agent, query_text):
    """Handle a single query asynchronously."""
    logger.info(f"Processing query: {query_text}")

    try:
        # Use query_async which returns a Future
        future = agent.query_async(query_text)

        # Wait for the result (with timeout)
        await asyncio.wait_for(asyncio.wrap_future(future), timeout=30.0)

        # Get the result
        if future.done():
            result = future.result()
            logger.info(f"Agent response: {result}")
            return result
        else:
            logger.warning("Query did not complete")
            return "Query timeout"

    except asyncio.TimeoutError:
        logger.error("Query timed out after 30 seconds")
        return "Query timeout"
    except Exception as e:
        logger.error(f"Error processing query: {e}")
        return f"Error: {str(e)}"


async def interactive_loop(agent):
    """Run an interactive query loop."""
    print("\n" + "=" * 60)
    print("Interactive Agent Mode")
    print("Type your commands or 'quit' to exit")
    print("=" * 60 + "\n")

    while True:
        try:
            # Get user input
            query = input("\nYou: ").strip()

            if query.lower() in ["quit", "exit", "q"]:
                break

            if not query:
                continue

            # Process query
            response = await handle_query(agent, query)
            print(f"\nAgent: {response}")

        except KeyboardInterrupt:
            break
        except Exception as e:
            logger.error(f"Error in interactive loop: {e}")


async def main():
    """Main async function."""
    print("\n" + "=" * 60)
    print("Unitree Go2 Robot with agents2 Framework (Async)")
    print("=" * 60)

    # Check for API key
    if not os.getenv("OPENAI_API_KEY"):
        print("ERROR: OPENAI_API_KEY not found")
        print("Set your API key in .env file or environment")
        sys.exit(1)

    # Load system prompt
    try:
        with open(SYSTEM_PROMPT_PATH, "r") as f:
            system_prompt = f.read()
    except FileNotFoundError:
        system_prompt = """You are a helpful robot assistant controlling a Unitree Go2 robot.
You have access to various movement and control skills. Be helpful and concise."""

    # Initialize robot (optional - comment out if no robot)
    robot = None
    if os.getenv("ROBOT_IP"):
        try:
            logger.info("Connecting to robot...")
            robot = UnitreeGo2(
                ip=os.getenv("ROBOT_IP"),
                connection_type=os.getenv("CONNECTION_TYPE", "webrtc"),
            )
            robot.start()
            await asyncio.sleep(3)
            logger.info("Robot connected")
        except Exception as e:
            logger.warning(f"Could not connect to robot: {e}")
            logger.info("Continuing without robot...")

    # Create skill container
    skill_container = UnitreeSkillContainer(robot=robot)

    # Create agent
    agent = Agent(
        system_prompt=system_prompt,
        model=Model.GPT_4O_MINI,  # Using mini for faster responses
        provider=Provider.OPENAI,
    )

    # Register skills and start
    agent.register_skills(skill_container)
    agent.start()

    # Log available skills
    skills = skill_container.skills()
    logger.info(f"Agent initialized with {len(skills)} skills")

    # Test query
    print("\n--- Testing agent query ---")
    test_response = await handle_query(agent, "Hello! Can you list 5 of your movement skills?")
    print(f"Test response: {test_response}\n")

    # Run interactive loop
    try:
        await interactive_loop(agent)
    except KeyboardInterrupt:
        logger.info("Interrupted by user")

    # Clean up
    logger.info("Shutting down...")
    agent.stop()
    if robot:
        logger.info("Robot disconnected")

    print("\nGoodbye!")


if __name__ == "__main__":
    # Run the async main function
    asyncio.run(main())
