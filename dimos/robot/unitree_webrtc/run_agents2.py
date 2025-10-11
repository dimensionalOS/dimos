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
from dimos.core.resource import Resource
from dimos.robot.robot import UnitreeRobot
from dimos.robot.unitree_webrtc.unitree_go2 import UnitreeGo2
from dimos.robot.unitree_webrtc.unitree_skill_container import UnitreeSkillContainer
from dimos.agents2.skills.navigation import NavigationSkillContainer
from dimos.robot.utils.robot_debugger import RobotDebugger
from dimos.utils.logging_config import setup_logger

logger = setup_logger(__file__)

load_dotenv()

with open(AGENT_SYSTEM_PROMPT_PATH, "r") as f:
    SYSTEM_PROMPT = f.read()


class UnitreeAgents2Runner(Resource):
    _robot: Optional[UnitreeRobot]
    _agent: Optional[Agent]
    _robot_debugger: Optional[RobotDebugger]
    _navigation_skill: Optional[NavigationSkillContainer]

    def __init__(self):
        self._robot: UnitreeRobot = None
        self._agent = None
        self._robot_debugger = None
        self._navigation_skill = None

    def start(self) -> None:
        self._robot = UnitreeGo2(
            ip=os.getenv("ROBOT_IP"),
            connection_type=os.getenv("CONNECTION_TYPE", "webrtc"),
        )

        time.sleep(3)

        logger.info("Robot initialized successfully")

        self.setup_agent()

        self._robot_debugger = RobotDebugger(self._robot)
        self._robot_debugger.start()

    def stop(self) -> None:
        if self._navigation_skill:
            self._navigation_skill.stop()
        if self._robot_debugger:
            self._robot_debugger.stop()
        if self._agent:
            self._agent.stop()
        if self._robot:
            self._robot.stop()

    def setup_agent(self) -> None:
        if not self._robot:
            raise ValueError("robot not set")

        logger.info("Setting up agent with skills...")

        self._agent = Agent(system_prompt=SYSTEM_PROMPT)
        self._navigation_skill = NavigationSkillContainer(
            robot=self._robot,
            video_stream=self._robot.connection.video,
        )
        self._navigation_skill.start()

        skill_containers = [
            UnitreeSkillContainer(robot=self._robot),
            self._navigation_skill,
            HumanInput(),
        ]

        for container in skill_containers:
            logger.info(f"Registering skills from container: {container}")
            self._agent.register_skills(container)

        self._agent.run_implicit_skill("human")

        self._agent.start()

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
    runner = UnitreeAgents2Runner()
    runner.start()
    runner.run()
    runner.stop()


if __name__ == "__main__":
    main()
