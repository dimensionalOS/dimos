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

import re

from dimos.mapping.google_maps.types import Coordinates, LocationContext, Position
from dimos.mapping.types import LatLon


def test_where_am_i(create_google_maps_agent, google_maps_skill_container) -> None:
    google_maps_skill_container._latest_location = LatLon(lat=37.782654, lon=-122.413273)
    google_maps_skill_container._client.get_location_context.return_value = LocationContext(
        street="Bourbon Street", coordinates=Coordinates(lat=37.782654, lon=-122.413273)
    )
    agent = create_google_maps_agent(fixture="test_where_am_i.json")

    response = agent.query("what street am I on")

    assert "bourbon" in response.lower()


def test_get_gps_position_for_queries(
    create_google_maps_agent, google_maps_skill_container
) -> None:
    google_maps_skill_container._latest_location = LatLon(lat=37.782654, lon=-122.413273)
    google_maps_skill_container._client.get_position.side_effect = [
        Position(lat=37.782601, lon=-122.413201, description="address 1"),
        Position(lat=37.782602, lon=-122.413202, description="address 2"),
        Position(lat=37.782603, lon=-122.413203, description="address 3"),
    ]
    agent = create_google_maps_agent(fixture="test_get_gps_position_for_queries.json")

    response = agent.query("what are the lat/lon for hyde park, regent park, russell park?")

    regex = r".*37\.782601.*122\.413201.*37\.782602.*122\.413202.*37\.782603.*122\.413203.*"
    assert re.match(regex, response, re.DOTALL)
