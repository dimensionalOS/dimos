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

import pytest
import reactivex as rx

from dimos.agents2.agent import Agent
from dimos.agents2.skills.navigation import NavigationSkillContainer
from dimos.robot.robot import UnitreeRobot
from dimos.robot.unitree_webrtc.run_agents2 import SYSTEM_PROMPT
from dimos.utils.data import get_data
from dimos.msgs.sensor_msgs import Image


@pytest.fixture
def fake_robot(mocker):
    return mocker.Mock(spec=UnitreeRobot)


@pytest.fixture
def fake_video_stream():
    image_path = get_data("chair-image.png")
    image = Image.from_file(str(image_path))
    return rx.of(image)


@pytest.fixture
def navigation_skill_container(fake_robot, fake_video_stream):
    with NavigationSkillContainer(fake_robot, fake_video_stream) as container:
        yield container


@pytest.fixture
def navigation_agent(navigation_skill_container):
    agent = Agent(system_prompt=SYSTEM_PROMPT)
    agent.register_skills(navigation_skill_container)
    with agent:
        yield agent
