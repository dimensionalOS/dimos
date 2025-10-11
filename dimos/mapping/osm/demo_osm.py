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
from dimos.core.resource import Resource
from dimos.mapping.types import LatLon
from dimos.robot.robot import Robot
from dimos.robot.utils.robot_debugger import RobotDebugger
from dimos.utils.logging_config import setup_logger

logger = setup_logger(__file__)

load_dotenv()

with open(AGENT_SYSTEM_PROMPT_PATH, "r") as f:
    SYSTEM_PROMPT = f.read()


class FakeRobot(Robot):
    pass


class UnitreeAgents2Runner(Resource):
    def __init__(self):
        self._robot = None
        self._agent = None
        self._robot_debugger = None
        self._osm_skill_container = None

    def start(self) -> None:
        self._robot = FakeRobot()
        self._agent = Agent(system_prompt=SYSTEM_PROMPT)
        self._osm_skill_container = OsmSkillContainer(self._robot, _get_fake_location())
        self._osm_skill_container.__enter__()
        self._agent.register_skills(self._osm_skill_container)
        self._agent.register_skills(HumanInput())
        self._agent.run_implicit_skill("human")
        self._agent.start()
        self._agent.loop_thread()
        self._robot_debugger = RobotDebugger(self._robot)
        self._robot_debugger.start()

    def stop(self) -> None:
        if self._robot_debugger:
            self._robot_debugger.stop()
        if self._osm_skill_container:
            self._osm_skill_container.__exit__(None, None, None)
        if self._agent:
            self._agent.stop()

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


def _get_fake_location() -> Observable[LatLon]:
    return rx.of(LatLon(lat=37.78092426217621, lon=-122.40682866540769))


if __name__ == "__main__":
    main()
