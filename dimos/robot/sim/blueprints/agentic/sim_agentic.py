# Copyright 2026 Dimensional Inc.
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

"""DimSim agentic blueprint — spatial + agent + skills + human input."""

from dimos.agents.agent import agent
from dimos.agents.skills.navigation import navigation_skill
from dimos.agents.skills.person_follow import person_follow_skill
from dimos.agents.skills.speak_skill import speak_skill
from dimos.agents.web_human_input import web_input
from dimos.core.blueprints import autoconnect
from dimos.robot.sim.blueprints.nav.sim_spatial import sim_spatial
from dimos.robot.sim.tf_module import _camera_info_static

sim_agentic = autoconnect(
    sim_spatial,
    agent(),
    navigation_skill(),
    person_follow_skill(camera_info=_camera_info_static()),
    web_input(),
    speak_skill(),
)

__all__ = ["sim_agentic"]
