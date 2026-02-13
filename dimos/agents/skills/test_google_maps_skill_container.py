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

import re

from langchain_core.messages import HumanMessage
import pytest

from dimos.agents.skills.google_maps_skill_container import GoogleMapsSkillContainer
from dimos.core.module import Module
from dimos.core.stream import Out
from dimos.mapping.google_maps.types import Coordinates, LocationContext, Position
from dimos.mapping.types import LatLon


class FakeGPS(Module):
    """Provides a gps_location output so GoogleMapsSkillContainer's input port gets a transport."""

    gps_location: Out[LatLon]


class FakeLocationClient:
    def get_location_context(self, location, radius=200):
        return LocationContext(
            street="Bourbon Street",
            coordinates=Coordinates(lat=37.782654, lon=-122.413273),
        )


class MockedWhereAmISkill(GoogleMapsSkillContainer):
    def __init__(self):
        Module.__init__(self)  # Skip GoogleMapsSkillContainer's __init__.
        self._client = FakeLocationClient()
        self._latest_location = LatLon(lat=37.782654, lon=-122.413273)
        self._started = True
        self._max_valid_distance = 20000


class FakePositionClient:
    def __init__(self):
        self._positions = iter(
            [
                Position(lat=37.782601, lon=-122.413201, description="address 1"),
                Position(lat=37.782602, lon=-122.413202, description="address 2"),
                Position(lat=37.782603, lon=-122.413203, description="address 3"),
            ]
        )

    def get_position(self, query, location):
        return next(self._positions)


class MockedPositionSkill(GoogleMapsSkillContainer):
    def __init__(self):
        Module.__init__(self)
        self._client = FakePositionClient()
        self._latest_location = LatLon(lat=37.782654, lon=-122.413273)
        self._started = True
        self._max_valid_distance = 20000


@pytest.mark.integration
def test_where_am_i(agent_setup) -> None:
    history = agent_setup(
        blueprints=[FakeGPS.blueprint(), MockedWhereAmISkill.blueprint()],
        messages=[HumanMessage("What street am I on? Use the where_am_i tool.")],
    )

    assert "bourbon" in history[-1].content.lower()


@pytest.mark.integration
def test_get_gps_position_for_queries(agent_setup) -> None:
    history = agent_setup(
        blueprints=[FakeGPS.blueprint(), MockedPositionSkill.blueprint()],
        messages=[
            HumanMessage(
                "What are the lat/lon for hyde park, regent park, russell park? "
                "Use the get_gps_position_for_queries tool."
            )
        ],
    )

    regex = r".*37\.782601.*122\.413201.*37\.782602.*122\.413202.*37\.782603.*122\.413203.*"
    assert re.match(regex, history[-1].content, re.DOTALL)
