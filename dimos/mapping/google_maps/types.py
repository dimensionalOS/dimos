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

from typing import List
from pydantic import BaseModel


class Coordinates(BaseModel):
    """GPS coordinates."""

    lat: float
    lon: float


class Position(BaseModel):
    """Basic position information from geocoding."""

    lat: float
    lon: float
    description: str


class PlacePosition(BaseModel):
    """Position with places API details."""

    lat: float
    lon: float
    description: str
    address: str
    types: List[str]


class NearbyPlace(BaseModel):
    """Information about a nearby place."""

    name: str
    types: List[str]
    distance: float
    vicinity: str


class LocationContext(BaseModel):
    """Contextual information about a location."""

    formatted_address: str
    street_number: str
    street: str
    neighborhood: str
    locality: str
    admin_area: str
    country: str
    postal_code: str
    nearby_places: List[NearbyPlace]
    place_types_summary: str
    coordinates: Coordinates
