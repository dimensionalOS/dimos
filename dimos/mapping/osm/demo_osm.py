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

import time
import reactivex as rx
from dotenv import load_dotenv
from reactivex import Observable

from dimos.agents2 import Agent
from dimos.agents2.cli.human import HumanInput
from dimos.agents2.constants import AGENT_SYSTEM_PROMPT_PATH
from dimos.agents2.skills.osm import OsmSkillContainer
from dimos.mapping.types import LatLon
from dimos.robot.robot import Robot
from dimos.robot.utils.robot_debugger import RobotDebugger
from dimos.utils.logging_config import setup_logger

from contextlib import ExitStack

logger = setup_logger(__file__)

load_dotenv()

with open(AGENT_SYSTEM_PROMPT_PATH, "r") as f:
    SYSTEM_PROMPT = f.read()


class FakeRobot(Robot):
    pass


class UnitreeAgents2Runner:
    def __init__(self):
        self._robot = None
        self._agent = None
        self._exit_stack = ExitStack()

    def __enter__(self):
        self._robot = FakeRobot()
        self._agent = Agent(system_prompt=SYSTEM_PROMPT)
        self._agent.register_skills(
            self._exit_stack.enter_context(OsmSkillContainer(self._robot, _get_fake_location()))
        )
        self._agent.register_skills(HumanInput())
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
    with UnitreeAgents2Runner() as runner:
        runner.run()


def _get_fake_location() -> Observable[LatLon]:
    return rx.of(LatLon(lat=37.78092426217621, lon=-122.40682866540769))


if __name__ == "__main__":
    main()
