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
from typing import Any, Optional, Union
from reactivex import Observable

from dimos.core.resource import Resource
from dimos.mapping.google_maps.google_maps import GoogleMaps
from dimos.mapping.osm.current_location_map import CurrentLocationMap
from dimos.mapping.types import LatLon
from dimos.protocol.skill.skill import SkillContainer, skill
from dimos.robot.robot import Robot
from dimos.utils.logging_config import setup_logger

from reactivex.disposable import CompositeDisposable

logger = setup_logger(__file__)


class GoogleMapsSkillContainer(SkillContainer, Resource):
    _robot: Robot
    _disposables: CompositeDisposable
    _latest_location: Optional[LatLon]
    _position_stream: Observable[LatLon]
    _current_location_map: CurrentLocationMap
    _started: bool

    def __init__(self, robot: Robot, position_stream: Observable[LatLon]):
        super().__init__()
        self._robot = robot
        self._disposables = CompositeDisposable()
        self._latest_location = None
        self._position_stream = position_stream
        self._client = GoogleMaps()
        self._started = False

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
    def where_am_i(self, context_radius: int = 200) -> str:
        """This skill returns information about what street/locality/city/etc
        you are in. It also gives you nearby landmarks.

        Example:

            where_am_i(context_radius=200)

        Args:
            context_radius (int): default 200, how many meters to look around
        """

        if not self._started:
            raise ValueError(f"{self} has not been started.")

        location = self._get_latest_location()

        result = None
        try:
            result = self._client.get_location_context(location, radius=context_radius)
        except Exception:
            return "There is an issue with the Google Maps API."

        if not result:
            return "Could not find anything about the current location."

        return result.model_dump_json()

    @skill()
    def get_gps_position_for_queries(self, *queries: str) -> str:
        """Get the GPS position (latitude/longitude)

        Example:

            get_gps_position_for_queries(['Fort Mason', 'Lafayette Park'])
            # returns
            [{"lat": 37.8059, "lon":-122.4290}, {"lat": 37.7915, "lon": -122.4276}]

        Args:
            queries (list[str]): The places you want to look up.
        """

        if not self._started:
            raise ValueError(f"{self} has not been started.")

        location = self._get_latest_location()

        results: list[Union[dict[str, Any], str]] = []

        for query in queries:
            try:
                latlon = self._client.get_position(query, location)
            except Exception:
                latlon = None
            if latlon:
                results.append(latlon.model_dump())
            else:
                results.append(f"no result for {query}")

        return json.dumps(results)
