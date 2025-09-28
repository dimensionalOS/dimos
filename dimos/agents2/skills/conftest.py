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
from functools import partial

from dimos.agents2.skills.gps_nav_skill import GpsNavSkillContainer
from dimos.agents2.skills.navigation import NavigationSkillContainer
from dimos.agents2.skills.google_maps_skill_container import GoogleMapsSkillContainer
from dimos.mapping.types import LatLon
from dimos.robot.robot import GpsRobot
from dimos.robot.unitree_webrtc.run_agents2 import SYSTEM_PROMPT
from dimos.utils.data import get_data
from dimos.msgs.sensor_msgs import Image


@pytest.fixture
def fake_robot(mocker):
    return mocker.MagicMock()


@pytest.fixture
def fake_gps_robot(mocker):
    return mocker.Mock(spec=GpsRobot)


@pytest.fixture
def fake_video_stream():
    image_path = get_data("chair-image.png")
    image = Image.from_file(str(image_path))
    return rx.of(image)


@pytest.fixture
def fake_gps_position_stream():
    return rx.of(LatLon(lat=37.783, lon=-122.413))


@pytest.fixture
def navigation_skill_container(fake_robot, fake_video_stream):
    with NavigationSkillContainer(fake_robot, fake_video_stream) as container:
        yield container


@pytest.fixture
def gps_nav_skill_container(fake_gps_robot, fake_gps_position_stream):
    with GpsNavSkillContainer(fake_gps_robot, fake_gps_position_stream) as container:
        yield container


@pytest.fixture
def google_maps_skill_container(fake_gps_robot, fake_gps_position_stream, mocker):
    with GoogleMapsSkillContainer(fake_gps_robot, fake_gps_position_stream) as container:
        container._client = mocker.MagicMock()
        yield container


@pytest.fixture
def create_navigation_agent(navigation_skill_container, create_fake_agent):
    return partial(
        create_fake_agent,
        system_prompt=SYSTEM_PROMPT,
        skill_containers=[navigation_skill_container],
    )


@pytest.fixture
def create_gps_nav_agent(gps_nav_skill_container, create_fake_agent):
    return partial(
        create_fake_agent, system_prompt=SYSTEM_PROMPT, skill_containers=[gps_nav_skill_container]
    )


@pytest.fixture
def create_google_maps_agent(
    gps_nav_skill_container, google_maps_skill_container, create_fake_agent
):
    return partial(
        create_fake_agent,
        system_prompt=SYSTEM_PROMPT,
        skill_containers=[gps_nav_skill_container, google_maps_skill_container],
    )
