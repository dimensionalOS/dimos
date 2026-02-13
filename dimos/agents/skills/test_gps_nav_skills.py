# Copyright 2025-2026 Dimensional Inc.
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

from langchain_core.messages import HumanMessage
import pytest

from dimos.agents.skills.gps_nav_skill import GpsNavSkillContainer
from dimos.core.module import Module
from dimos.core.stream import Out
from dimos.mapping.types import LatLon


class FakeGPS(Module):
    """Provides a gps_location output so GpsNavSkillContainer's input port gets a transport."""

    gps_location: Out[LatLon]


class MockedGpsNavSkill(GpsNavSkillContainer):
    def __init__(self):
        Module.__init__(self)
        self._latest_location = LatLon(lat=37.782654, lon=-122.413273)
        self._started = True
        self._max_valid_distance = 50000


@pytest.mark.integration
def test_set_gps_travel_points(agent_setup) -> None:
    history = agent_setup(
        blueprints=[FakeGPS.blueprint(), MockedGpsNavSkill.blueprint()],
        messages=[
            HumanMessage(
                'Set GPS travel points to [{"lat": 37.782654, "lon": -122.413273}]. '
                "Use the set_gps_travel_points tool."
            )
        ],
    )

    assert "success" in history[-1].content.lower()


@pytest.mark.integration
def test_set_gps_travel_points_multiple(agent_setup) -> None:
    history = agent_setup(
        blueprints=[FakeGPS.blueprint(), MockedGpsNavSkill.blueprint()],
        messages=[
            HumanMessage(
                "Set GPS travel points to these locations in order: "
                '{"lat": 37.782654, "lon": -122.413273}, '
                '{"lat": 37.782660, "lon": -122.413260}, '
                '{"lat": 37.782670, "lon": -122.413270}. '
                "Use the set_gps_travel_points tool."
            )
        ],
    )

    assert "success" in history[-1].content.lower()
