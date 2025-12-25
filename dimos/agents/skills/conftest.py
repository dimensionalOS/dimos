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

from functools import partial

import pytest
from reactivex.scheduler import ThreadPoolScheduler

from dimos.agents.skills.google_maps_skill_container import GoogleMapsSkillContainer
from dimos.agents.skills.gps_nav_skill import GpsNavSkillContainer
from dimos.agents.skills.navigation import NavigationSkillContainer
from dimos.agents.system_prompt import get_system_prompt
from dimos.robot.unitree_webrtc.unitree_skill_container import UnitreeSkillContainer

system_prompt = get_system_prompt()


@pytest.fixture(autouse=True)
def cleanup_threadpool_scheduler(monkeypatch):
    # TODO: get rid of this global threadpool
    """Clean up and recreate the global ThreadPoolScheduler after each test."""
    # Disable ChromaDB telemetry to avoid leaking threads
    monkeypatch.setenv("CHROMA_ANONYMIZED_TELEMETRY", "False")
    yield
    from dimos.utils import threadpool

    # Shutdown the global scheduler's executor
    threadpool.scheduler.executor.shutdown(wait=True)
    # Recreate it for the next test
    threadpool.scheduler = ThreadPoolScheduler(max_workers=threadpool.get_max_workers())


@pytest.fixture
def navigation_skill_container(mocker):
    container = NavigationSkillContainer()
    container.color_image.connection = mocker.MagicMock()
    container.odom.connection = mocker.MagicMock()
    container.start()
    yield container
    container.stop()


@pytest.fixture
def gps_nav_skill_container(mocker):
    container = GpsNavSkillContainer()
    container.gps_location.connection = mocker.MagicMock()
    container.gps_goal = mocker.MagicMock()
    container.start()
    yield container
    container.stop()


@pytest.fixture
def google_maps_skill_container(mocker):
    container = GoogleMapsSkillContainer()
    container.gps_location.connection = mocker.MagicMock()
    container.start()
    container._client = mocker.MagicMock()
    yield container
    container.stop()


@pytest.fixture
def unitree_skills(mocker):
    container = UnitreeSkillContainer()
    container._move = mocker.Mock()
    container._publish_request = mocker.Mock()
    container.start()
    yield container
    container.stop()


@pytest.fixture
def create_navigation_agent(navigation_skill_container, create_fake_agent):
    return partial(
        create_fake_agent,
        system_prompt=system_prompt,
        skill_containers=[navigation_skill_container],
    )


@pytest.fixture
def create_gps_nav_agent(gps_nav_skill_container, create_fake_agent):
    return partial(
        create_fake_agent, system_prompt=system_prompt, skill_containers=[gps_nav_skill_container]
    )


@pytest.fixture
def create_google_maps_agent(
    gps_nav_skill_container, google_maps_skill_container, create_fake_agent
):
    return partial(
        create_fake_agent,
        system_prompt=system_prompt,
        skill_containers=[gps_nav_skill_container, google_maps_skill_container],
    )


@pytest.fixture
def create_unitree_skills_agent(unitree_skills, create_fake_agent):
    return partial(
        create_fake_agent,
        system_prompt=system_prompt,
        skill_containers=[unitree_skills],
    )
