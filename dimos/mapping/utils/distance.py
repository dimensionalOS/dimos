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

import math

from dimos.mapping.types import LatLon


def distance_in_meters(location1: LatLon, location2: LatLon) -> float:
    """Calculate the great circle distance between two points on Earth using Haversine formula.

    Args:
        location1: First location with latitude and longitude
        location2: Second location with latitude and longitude

    Returns:
        Distance in meters between the two points
    """
    # Earth's radius in meters
    EARTH_RADIUS_M = 6371000

    # Convert degrees to radians
    lat1_rad = math.radians(location1.lat)
    lat2_rad = math.radians(location2.lat)
    lon1_rad = math.radians(location1.lon)
    lon2_rad = math.radians(location2.lon)

    # Haversine formula
    dlat = lat2_rad - lat1_rad
    dlon = lon2_rad - lon1_rad

    a = math.sin(dlat / 2) ** 2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon / 2) ** 2
    c = 2 * math.asin(math.sqrt(a))

    distance = EARTH_RADIUS_M * c

    return distance
