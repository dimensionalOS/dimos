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


from dimos.core.skill_module import SkillModule
from dimos.core.stream import In
from dimos.mapping.osm.current_location_map import CurrentLocationMap
from dimos.mapping.types import LatLon
from dimos.mapping.utils.distance import distance_in_meters
from dimos.models.vl.qwen import QwenVlModel
from dimos.protocol.skill.skill import skill
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class OsmSkill(SkillModule):
    _latest_location: LatLon | None
    _current_location_map: CurrentLocationMap

    gps_location: In[LatLon]

    def __init__(self) -> None:
        super().__init__()
        self._latest_location = None
        self._current_location_map = CurrentLocationMap(QwenVlModel())

    def start(self) -> None:
        super().start()
        self._disposables.add(self.gps_location.subscribe(self._on_gps_location))  # type: ignore[arg-type]

    def stop(self) -> None:
        super().stop()

    def _on_gps_location(self, location: LatLon) -> None:
        self._latest_location = location

    @skill()
    def map_query(self, query_sentence: str) -> str:
        """This skill uses a vision language model to find something on the map
        based on the query sentence. You can query it with something like "Where
        can I find a coffee shop?" and it returns the latitude and longitude.

        Example:

            map_query("Where can I find a coffee shop?")

        Args:
            query_sentence (str): The query sentence.
        """

        self._current_location_map.update_position(self._latest_location)  # type: ignore[arg-type]
        location = self._current_location_map.query_for_one_position_and_context(
            query_sentence,
            self._latest_location,  # type: ignore[arg-type]
        )
        if not location:
            return "Could not find anything."

        latlon, context = location

        distance = int(distance_in_meters(latlon, self._latest_location))  # type: ignore[arg-type]

        return f"{context}. It's at position latitude={latlon.lat}, longitude={latlon.lon}. It is {distance} meters away."


osm_skill = OsmSkill.blueprint

__all__ = ["OsmSkill", "osm_skill"]
