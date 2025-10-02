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

from typing import Optional
from reactivex import Observable

from dimos.mapping.osm.current_location_map import CurrentLocationMap
from dimos.mapping.utils.distance import distance_in_meters
from dimos.mapping.types import LatLon
from dimos.models.vl.qwen import QwenVlModel
from dimos.protocol.skill.skill import SkillContainer, skill
from dimos.robot.robot import Robot
from dimos.utils.logging_config import setup_logger

from reactivex.disposable import CompositeDisposable

logger = setup_logger(__file__)


class OsmSkillContainer(SkillContainer):
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
        self._current_location_map = CurrentLocationMap(QwenVlModel())
        self._started = False

    def __enter__(self) -> "OsmSkillContainer":
        self._started = True
        self._disposables.add(self._position_stream.subscribe(self._on_gps_location))
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self._disposables.dispose()
        self.stop()
        return False

    def _on_gps_location(self, location: LatLon) -> None:
        self._latest_location = location

    @skill()
    def street_map_query(self, query_sentence: str) -> str:
        """This skill uses a vision language model to find something on the map
        based on the query sentence. You can query it with something like "Where
        can I find a coffee shop?" and it returns the latitude and longitude.

        Example:

            street_map_query("Where can I find a coffee shop?")

        Args:
            query_sentence (str): The query sentence.
        """

        if not self._started:
            raise ValueError(f"{self} has not been started.")

        self._current_location_map.update_position(self._latest_location)
        location = self._current_location_map.query_for_one_position_and_context(
            query_sentence, self._latest_location
        )
        if not location:
            return "Could not find anything."

        latlon, context = location

        distance = int(distance_in_meters(latlon, self._latest_location))

        return f"{context}. It's at position latitude={latlon.lat}, longitude={latlon.lon}. It is {distance} meters away."
