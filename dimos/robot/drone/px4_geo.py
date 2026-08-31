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

"""World-frame geometry helpers shared by the PX4 drone and swarm modules.

LOCAL_POSITION_NED is referenced to *each drone's own home*, so subtracting two
drones' NED gives nonsense (each reads "0,0,0 at my spawn"). Real inter-drone
distance has to come from GLOBAL_POSITION_INT (lat/lon/alt) and a great-circle
computation — that is what lives here.
"""

from __future__ import annotations

import math

EARTH_RADIUS_M = 6371000.0
WGS84_EQUATORIAL_M = 6378137.0

# A world-frame point: (latitude_deg, longitude_deg, altitude_m).
GlobalPoint = tuple[float, float, float]


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Great-circle distance in meters between two (lat, lon) points in degrees."""
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return EARTH_RADIUS_M * c


def offset_latlon(lat: float, lon: float, dn_m: float, de_m: float) -> tuple[float, float]:
    """Offset a lat/lon by (north, east) meters.

    Small-angle approximation — accurate to a few centimeters for the sub-km
    offsets the swarm deals with.
    """
    new_lat = lat + math.degrees(dn_m / WGS84_EQUATORIAL_M)
    new_lon = lon + math.degrees(de_m / (WGS84_EQUATORIAL_M * math.cos(math.radians(lat))))
    return new_lat, new_lon


def distance_3d_m(g1: GlobalPoint, g2: GlobalPoint) -> float:
    """3-D distance between two (lat_deg, lon_deg, alt_m) world-frame points."""
    horiz = haversine_m(g1[0], g1[1], g2[0], g2[1])
    vert = g1[2] - g2[2]
    return math.sqrt(horiz * horiz + vert * vert)


def pairwise_distances(
    positions: dict[str, GlobalPoint],
) -> list[tuple[str, str, float]]:
    """Pairwise 3-D distance (meters) between drones' world-frame positions.

    Positions must be (lat_deg, lon_deg, alt_m). Returns ``(a, b, distance)``
    triples with ``a < b`` by key, sorted for stable reporting.
    """
    keys = sorted(positions.keys())
    out: list[tuple[str, str, float]] = []
    for i, a in enumerate(keys):
        for b in keys[i + 1 :]:
            out.append((a, b, distance_3d_m(positions[a], positions[b])))
    return out


__all__ = [
    "EARTH_RADIUS_M",
    "WGS84_EQUATORIAL_M",
    "GlobalPoint",
    "distance_3d_m",
    "haversine_m",
    "offset_latlon",
    "pairwise_distances",
]
