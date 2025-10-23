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

import json

from reactivex import Observable
from reactivex.disposable import CompositeDisposable

from dimos.core.resource import Resource
from dimos.mapping.google_maps.google_maps import GoogleMaps
from dimos.mapping.osm.current_location_map import CurrentLocationMap
from dimos.mapping.types import LatLon
from dimos.mapping.utils.distance import distance_in_meters
from dimos.protocol.skill.skill import SkillContainer, skill
from dimos.robot.robot import Robot
from dimos.utils.logging_config import setup_logger

logger = setup_logger(__file__)


class GpsNavSkillContainer(SkillContainer, Resource):
    _robot: Robot
    _disposables: CompositeDisposable
    _latest_location: LatLon | None
    _position_stream: Observable[LatLon]
    _current_location_map: CurrentLocationMap
    _started: bool
    _max_valid_distance: int

    def __init__(self, robot: Robot, position_stream: Observable[LatLon]) -> None:
        super().__init__()
        self._robot = robot
        self._disposables = CompositeDisposable()
        self._latest_location = None
        self._position_stream = position_stream
        self._client = GoogleMaps()
        self._started = False
        self._max_valid_distance = 50000

    def start(self) -> None:
        self._started = True
        self._disposables.add(self._position_stream.subscribe(self._on_gps_location))

    def stop(self) -> None:
        self._disposables.dispose()
        super().stop()

    def _on_gps_location(self, location: LatLon) -> None:
        self._latest_location = location

    def _get_latest_location(self) -> LatLon:
        if not self._latest_location:
            raise ValueError("The position has not been set yet.")
        return self._latest_location

    @skill()
    def set_gps_travel_points(self, *points: dict[str, float]) -> str:
        """Define the movement path determined by GPS coordinates. Requires at least one. You can get the coordinates by using the `get_gps_position_for_queries` skill.

        Example:

            set_gps_travel_goals([{"lat": 37.8059, "lon":-122.4290}, {"lat": 37.7915, "lon": -122.4276}])
            # Travel first to {"lat": 37.8059, "lon":-122.4290}
            # then travel to {"lat": 37.7915, "lon": -122.4276}
        """

        if not self._started:
            raise ValueError(f"{self} has not been started.")

        new_points = [self._convert_point(x) for x in points]

        if not all(new_points):
            parsed = json.dumps([x.__dict__ if x else x for x in new_points])
            return f"Not all points were valid. I parsed this: {parsed}"

        logger.info(f"Set travel points: {new_points}")

        self._robot.set_gps_travel_goal_points(new_points)

        return "I've successfully set the travel points."

    def _convert_point(self, point: dict[str, float]) -> LatLon | None:
        if not isinstance(point, dict):
            return None
        lat = point.get("lat")
        lon = point.get("lon")

        if lat is None or lon is None:
            return None

        new_point = LatLon(lat=lat, lon=lon)
        distance = distance_in_meters(self._get_latest_location(), new_point)
        if distance > self._max_valid_distance:
            return None

        return new_point
