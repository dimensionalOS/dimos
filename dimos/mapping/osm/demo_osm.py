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

from dotenv import load_dotenv
from reactivex import interval

from dimos.agents2.agent import llm_agent
from dimos.agents2.cli.human import human_input
from dimos.agents2.skills.osm import osm_skill
from dimos.agents2.system_prompt import get_system_prompt
from dimos.core.blueprints import autoconnect
from dimos.core.module import Module
from dimos.core.stream import Out
from dimos.mapping.types import LatLon

load_dotenv()


class DemoRobot(Module):
    gps_location: Out[LatLon] = None

    def start(self):
        super().start()
        self._disposables.add(interval(1.0).subscribe(lambda _: self._publish_gps_location()))

    def stop(self):
        super().stop()

    def _publish_gps_location(self):
        self.gps_location.publish(LatLon(lat=37.78092426217621, lon=-122.40682866540769))


demo_robot = DemoRobot.blueprint


demo_osm = autoconnect(
    demo_robot(),
    osm_skill(),
    human_input(),
    llm_agent(system_prompt=get_system_prompt()),
)
