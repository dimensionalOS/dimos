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

import argparse
import os
import time
from typing import Optional
from dotenv import load_dotenv

from dimos.agents2 import Agent
from dimos.agents2.cli.human import HumanInput
from dimos.agents2.constants import AGENT_SYSTEM_PROMPT_PATH
from dimos.agents2.skills.ros_navigation import RosNavigation
from dimos.robot.robot import UnitreeRobot
from dimos.robot.unitree_webrtc.unitree_g1 import UnitreeG1
from dimos.robot.utils.robot_debugger import RobotDebugger
from dimos.utils.logging_config import setup_logger

from contextlib import ExitStack

logger = setup_logger(__file__)

load_dotenv()

with open(AGENT_SYSTEM_PROMPT_PATH, "r") as f:
    SYSTEM_PROMPT = f.read()


class UnitreeAgents2Runner:
    _robot: Optional[UnitreeRobot]
    _agent: Optional[Agent]
    _exit_stack: ExitStack

    def __init__(self, gstreamer_host: str = "10.0.0.227"):
        self._robot: UnitreeRobot = None
        self._agent = None
        self._exit_stack = ExitStack()
        self._gstreamer_host = gstreamer_host

    def __enter__(self):
        self._robot = self._exit_stack.enter_context(
            UnitreeG1(
                ip=os.getenv("ROBOT_IP"),
                enable_connection=os.getenv("ROBOT_IP") is not None,
                enable_perception=True,
                enable_gstreamer_camera=True,
                gstreamer_host=self._gstreamer_host,
            )
        )

        time.sleep(2)

        self._agent = Agent(system_prompt=SYSTEM_PROMPT)

        skill_containers = [
            self._exit_stack.enter_context(RosNavigation(self._robot)),
            HumanInput(),
        ]

        for container in skill_containers:
            self._agent.register_skills(container)

        self._agent.run_implicit_skill("human")
        self._exit_stack.enter_context(self._agent)
        self._agent.loop_thread()

        self._exit_stack.enter_context(RobotDebugger(self._robot))

        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self._exit_stack.close()
        return False

    def run(self):
        while True:
            try:
                time.sleep(1)
            except KeyboardInterrupt:
                return


def main():
    parser = argparse.ArgumentParser(description="Run Unitree G1 with Agents2")
    parser.add_argument(
        "--gstreamer-host",
        type=str,
        default="10.0.0.227",
        help="GStreamer host IP address (default: 10.0.0.227)",
    )

    args = parser.parse_args()

    with UnitreeAgents2Runner(gstreamer_host=args.gstreamer_host) as runner:
        runner.run()


if __name__ == "__main__":
    main()
