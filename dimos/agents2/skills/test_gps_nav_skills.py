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


from dimos.mapping.types import LatLon


def test_set_gps_travel_points(create_gps_nav_agent, gps_nav_skill_container, mocker) -> None:
    gps_nav_skill_container._latest_location = LatLon(lat=37.782654, lon=-122.413273)
    gps_nav_skill_container._set_gps_travel_goal_points = mocker.Mock()
    agent = create_gps_nav_agent(fixture="test_set_gps_travel_points.json")

    agent.query("go to lat: 37.782654, lon: -122.413273")

    gps_nav_skill_container._set_gps_travel_goal_points.assert_called_once_with(
        [LatLon(lat=37.782654, lon=-122.413273)]
    )
    gps_nav_skill_container.gps_goal.publish.assert_called_once_with(
        [LatLon(lat=37.782654, lon=-122.413273)]
    )


def test_set_gps_travel_points_multiple(
    create_gps_nav_agent, gps_nav_skill_container, mocker
) -> None:
    gps_nav_skill_container._latest_location = LatLon(lat=37.782654, lon=-122.413273)
    gps_nav_skill_container._set_gps_travel_goal_points = mocker.Mock()
    agent = create_gps_nav_agent(fixture="test_set_gps_travel_points_multiple.json")

    agent.query(
        "go to lat: 37.782654, lon: -122.413273, then 37.782660,-122.413260, and then 37.782670,-122.413270"
    )

    gps_nav_skill_container._set_gps_travel_goal_points.assert_called_once_with(
        [
            LatLon(lat=37.782654, lon=-122.413273),
            LatLon(lat=37.782660, lon=-122.413260),
            LatLon(lat=37.782670, lon=-122.413270),
        ]
    )
    gps_nav_skill_container.gps_goal.publish.assert_called_once_with(
        [
            LatLon(lat=37.782654, lon=-122.413273),
            LatLon(lat=37.782660, lon=-122.413260),
            LatLon(lat=37.782670, lon=-122.413270),
        ]
    )
