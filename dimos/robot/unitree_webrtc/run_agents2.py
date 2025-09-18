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

import os
import sys
import time
from dotenv import load_dotenv

from dimos.agents2 import Agent
from dimos.agents2.spec import Model, Provider
from dimos.agents2.cli.human import HumanInput
from dimos.robot.unitree_webrtc.unitree_go2 import UnitreeGo2
from dimos.robot.unitree_webrtc.unitree_skill_container import UnitreeSkillContainer
from dimos.agents2.skills.navigation import NavigationSkillContainer
from dimos.agents2.skills.unitree_wrappers.basic_wrappers import BasicSkillContainer
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.robot.unitree_webrtc.run_agents2")

load_dotenv()

SYSTEM_PROMPT_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__)))),
    "assets/agent/prompt.txt",
)
with open(SYSTEM_PROMPT_PATH, "r") as f:
    system_prompt = f.read()

# Check for API key
if not os.getenv("ANTHROPIC_API_KEY"):
    print("WARNING: ANTHROPIC_API_KEY not found in environment")
    print("Please set your API key in .env file or environment")
    sys.exit(1)


class UnitreeAgents2Runner:
    def __init__(self):
        self.robot = None
        self.agent = None

    def __enter__(self):
        self.robot = self.setup_robot()

        # Create skill containers
        # Note: For kill skill, we'll need to pass the skill library reference
        # which will be available from the agent after registration.
        basic_skills = BasicSkillContainer(robot=self.robot, skill_library=None)
        navigation_skills = NavigationSkillContainer(robot=self.robot)
        unitree_skills = UnitreeSkillContainer(robot=self.robot)
        human_input = HumanInput()

        self.agent = self.setup_agent(
            [
                unitree_skills,
                navigation_skills,
                basic_skills,
                human_input,
            ],
            system_prompt,
        )

        # Update the basic skills container with the agent's skill library
        # (needed for kill skill to work properly)
        basic_skills._skill_library = self.agent

        logger.info("=" * 60)
        logger.info("Unitree Go2 Agent Ready (agents2 framework)!")
        logger.info("You can:")
        logger.info("  - Type commands in the human CLI")
        logger.info("  - Ask the robot to navigate to locations")
        logger.info("  - Ask the robot to observe and describe surroundings")
        logger.info("  - Ask the robot to follow people or explore areas")
        logger.info("  - Ask the robot to perform actions (sit, stand, dance, etc.)")
        logger.info("  - Ask the robot to speak text")
        logger.info("=" * 60)

        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        logger.info("Shutting down...")

        if self.agent:
            try:
                self.agent.stop()
            except Exception as e:
                logger.error(f"Error stopping agent: {e}")

        logger.info("Shutdown complete")
        return False

    def setup_robot(self) -> UnitreeGo2:
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
        logger.info("Setting up agent with skills...")

        agent = Agent(
            system_prompt=system_prompt,
            # model=Model.CLAUDE_35_SONNET_LATEST,
            # provider=Provider.ANTHROPIC,
        )

        for container in skillcontainers:
            logger.info(f"Registering skills from container: {container}")
            agent.register_skills(container)

        agent.run_implicit_skill("human")

        agent.start()

        # Log available skills
        tools = agent.get_tools()
        names = ", ".join([tool.name for tool in tools])
        logger.info(f"Agent configured with {len(tools)} skills: {names}")

        # Start the agent loop thread
        agent.loop_thread()

        return agent

    def run(self):
        while True:
            try:
                time.sleep(1)
            except KeyboardInterrupt:
                return


def main():
    with UnitreeAgents2Runner() as runner:
        runner.run()


if __name__ == "__main__":
    main()
