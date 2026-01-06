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

from dimos.core.core import rpc
from dimos.core.rpc_client import RpcCall
from dimos.core.skill_module import SkillModule
from dimos.core.stream import In, Out
from dimos.mapping.types import LatLon
from dimos.mapping.utils.distance import distance_in_meters
from dimos.protocol.skill.skill import skill
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class GpsNavSkillContainer(SkillModule):
    _latest_location: LatLon | None = None
    _max_valid_distance: int = 50000
    _set_gps_travel_goal_points: RpcCall | None = None

    gps_location: In[LatLon]
    gps_goal: Out[LatLon]

    def __init__(self) -> None:
        super().__init__()

    @rpc
    def start(self) -> None:
        super().start()
        self._disposables.add(self.gps_location.subscribe(self._on_gps_location))  # type: ignore[arg-type]

    @rpc
    def stop(self) -> None:
        super().stop()

    @rpc
    def set_WebsocketVisModule_set_gps_travel_goal_points(self, callable: RpcCall) -> None:
        self._set_gps_travel_goal_points = callable
        self._set_gps_travel_goal_points.set_rpc(self.rpc)  # type: ignore[arg-type]

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

        new_points = [self._convert_point(x) for x in points]

        if not all(new_points):
            parsed = json.dumps([x.__dict__ if x else x for x in new_points])
            return f"Not all points were valid. I parsed this: {parsed}"

        for new_point in new_points:
            distance = distance_in_meters(self._get_latest_location(), new_point)  # type: ignore[arg-type]
            if distance > self._max_valid_distance:
                return f"Point {new_point} is too far ({int(distance)} meters away)."

        logger.info(f"Set travel points: {new_points}")

        if self.gps_goal._transport is not None:
            self.gps_goal.publish(new_points)

        if self._set_gps_travel_goal_points:
            self._set_gps_travel_goal_points(new_points)

        return "I've successfully set the travel points."

    def _convert_point(self, point: dict[str, float]) -> LatLon | None:
        if not isinstance(point, dict):
            return None
        lat = point.get("lat")
        lon = point.get("lon")

        if lat is None or lon is None:
            return None

        return LatLon(lat=lat, lon=lon)


gps_nav_skill = GpsNavSkillContainer.blueprint


__all__ = ["GpsNavSkillContainer", "gps_nav_skill"]
