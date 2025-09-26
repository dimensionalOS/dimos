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
import time
from typing import Optional
from dotenv import load_dotenv

from dimos.agents2 import Agent
from dimos.agents2.cli.human import HumanInput
from dimos.agents2.constants import AGENT_SYSTEM_PROMPT_PATH
from dimos.robot.robot import UnitreeRobot
from dimos.robot.unitree_webrtc.unitree_go2 import UnitreeGo2
from dimos.robot.unitree_webrtc.unitree_skill_container import UnitreeSkillContainer
from dimos.agents2.skills.navigation import NavigationSkillContainer
from dimos.robot.utils.robot_debugger import RobotDebugger
from dimos.utils.logging_config import setup_logger

from contextlib import ExitStack

logger = setup_logger("dimos.robot.unitree_webrtc.run_agents2")

load_dotenv()

with open(AGENT_SYSTEM_PROMPT_PATH, "r") as f:
    SYSTEM_PROMPT = f.read()


class UnitreeAgents2Runner:
    _robot: Optional[UnitreeRobot]
    _agent: Optional[Agent]
    _exit_stack: ExitStack

    def __init__(self):
        self._robot: UnitreeRobot = None
        self._agent = None
        self._exit_stack = ExitStack()

    def __enter__(self):
        logger.info("Initializing Unitree Go2 robot...")

        self._robot = self._exit_stack.enter_context(
            UnitreeGo2(
                ip=os.getenv("ROBOT_IP"),
                connection_type=os.getenv("CONNECTION_TYPE", "webrtc"),
            )
        )

        time.sleep(3)

        logger.info("Robot initialized successfully")

        self.setup_agent()

        self._exit_stack.enter_context(RobotDebugger(self._robot))

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
        self._exit_stack.close()
        logger.info("Shutdown complete")
        return False

    def setup_agent(self) -> None:
        if not self._robot:
            raise ValueError("robot not set")

        logger.info("Setting up agent with skills...")

        self._agent = Agent(system_prompt=SYSTEM_PROMPT)

        skill_containers = [
            UnitreeSkillContainer(robot=self._robot),
            self._exit_stack.enter_context(
                NavigationSkillContainer(
                    robot=self._robot,
                    video_stream=self._robot.connection.video,
                )
            ),
            HumanInput(),
        ]

        for container in skill_containers:
            logger.info(f"Registering skills from container: {container}")
            self._agent.register_skills(container)

        self._agent.run_implicit_skill("human")

        self._exit_stack.enter_context(self._agent)

        # Log available skills
        tools = self._agent.get_tools()
        names = ", ".join([tool.name for tool in tools])
        logger.info(f"Agent configured with {len(tools)} skills: {names}")

        # Start the agent loop thread
        self._agent.loop_thread()

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
