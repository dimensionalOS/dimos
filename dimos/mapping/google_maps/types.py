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
    types: list[str]


class NearbyPlace(BaseModel):
    """Information about a nearby place."""

    name: str
    types: list[str]
    distance: float
    vicinity: str


class LocationContext(BaseModel):
    """Contextual information about a location."""

    formatted_address: str | None = None
    street_number: str | None = None
    street: str | None = None
    neighborhood: str | None = None
    locality: str | None = None
    admin_area: str | None = None
    country: str | None = None
    postal_code: str | None = None
    nearby_places: list[NearbyPlace] = []
    place_types_summary: str | None = None
    coordinates: Coordinates
