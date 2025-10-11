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
Run script for Unitree Go2 robot with agents2 framework.
This is the migrated version using the new LangChain-based agent system.
"""

import os
import sys
import time
from pathlib import Path

from dotenv import load_dotenv

from dimos.agents2.cli.human import HumanInput

# Add parent directories to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent.parent))


from dimos.agents2 import Agent
from dimos.agents2.spec import Model, Provider
from dimos.robot.unitree_webrtc.unitree_go2 import UnitreeGo2
from dimos.robot.unitree_webrtc.unitree_skill_container import UnitreeSkillContainer
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.agents2.run_unitree")

# Load environment variables
load_dotenv()

# System prompt path
SYSTEM_PROMPT_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__)))),
    "assets/agent/prompt.txt",
)


class UnitreeAgentRunner:
    """Manages the Unitree robot with the new agents2 framework."""

    def __init__(self):
        self.robot = None
        self.agent = None
        self.agent_thread = None
        self.running = False

    def setup_robot(self) -> UnitreeGo2:
        """Initialize the robot connection."""
        logger.info("Initializing Unitree Go2 robot...")

        robot = UnitreeGo2(
            ip=os.getenv("ROBOT_IP"),
            connection_type=os.getenv("CONNECTION_TYPE", "webrtc"),
        )

        robot.start()
        time.sleep(3)

        logger.info("Robot initialized successfully")
        return robot

    def setup_agent(self, skillcontainers, system_prompt: str) -> Agent:
        """Create and configure the agent with skills."""
        logger.info("Setting up agent with skills...")

        # Create agent
        agent = Agent(
            system_prompt=system_prompt,
            model=Model.GPT_4O,  # Could add CLAUDE models to enum
            provider=Provider.OPENAI,  # Would need ANTHROPIC provider
        )

        for container in skillcontainers:
            print("REGISTERING SKILLS FROM CONTAINER:", container)
            agent.register_skills(container)

        agent.run_implicit_skill("human")

        agent.start()

        # Log available skills
        names = ", ".join([tool.name for tool in agent.get_tools()])
        logger.info(f"Agent configured with {len(names)} skills: {names}")

        agent.loop_thread()
        return agent

    def run(self):
        """Main run loop."""
        print("\n" + "=" * 60)
        print("Unitree Go2 Robot with agents2 Framework")
        print("=" * 60)
        print("\nThis system integrates:")
        print("  - Unitree Go2 quadruped robot")
        print("  - WebRTC communication interface")
        print("  - LangChain-based agent system (agents2)")
        print("  - Converted skill system with @skill decorators")
        print("\nStarting system...\n")

        # Check for API key (would need ANTHROPIC_API_KEY for Claude)
        if not os.getenv("OPENAI_API_KEY"):
            print("WARNING: OPENAI_API_KEY not found in environment")
            print("Please set your API key in .env file or environment")
            print("(Note: Full Claude support would require ANTHROPIC_API_KEY)")
            sys.exit(1)

        system_prompt = """You are a helpful robot assistant controlling a Unitree Go2 quadruped robot.
You can move, navigate, speak, and perform various actions. Be helpful and friendly."""

        try:
            # Setup components
            self.robot = self.setup_robot()

            self.agent = self.setup_agent(
                [
                    UnitreeSkillContainer(self.robot),
                    HumanInput(),
                ],
                system_prompt,
            )

            # Start handling queries
            self.running = True

            logger.info("=" * 60)
            logger.info("Unitree Go2 Agent Ready (agents2 framework)!")
            logger.info("You can:")
            logger.info("  - Type commands in the human cli")
            logger.info("  - Ask the robot to move or navigate")
            logger.info("  - Ask the robot to perform actions (sit, stand, dance, etc.)")
            logger.info("  - Ask the robot to speak text")
            logger.info("=" * 60)

            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            logger.info("Keyboard interrupt received")
        except Exception as e:
            logger.error(f"Error running robot: {e}")
            import traceback

            traceback.print_exc()
        # finally:
        # self.shutdown()

    def shutdown(self):
        logger.info("Shutting down...")
        self.running = False

        if self.agent:
            try:
                self.agent.stop()
                logger.info("Agent stopped")
            except Exception as e:
                logger.error(f"Error stopping agent: {e}")

        if self.robot:
            try:
                self.robot.stop()
                logger.info("Robot connection closed")
            except Exception as e:
                logger.error(f"Error stopping robot: {e}")

        logger.info("Shutdown complete")


def main():
    runner = UnitreeAgentRunner()
    runner.run()


if __name__ == "__main__":
    main()
