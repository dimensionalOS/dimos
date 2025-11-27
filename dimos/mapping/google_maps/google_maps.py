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

import os

import googlemaps  # type: ignore[import-untyped]

from dimos.mapping.google_maps.types import (
    Coordinates,
    LocationContext,
    NearbyPlace,
    PlacePosition,
    Position,
)
from dimos.mapping.types import LatLon
from dimos.mapping.utils.distance import distance_in_meters
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class GoogleMaps:
    _client: googlemaps.Client
    _max_nearby_places: int

    def __init__(self, api_key: str | None = None) -> None:
        api_key = api_key or os.environ.get("GOOGLE_MAPS_API_KEY")
        if not api_key:
            raise ValueError("GOOGLE_MAPS_API_KEY environment variable not set")
        self._client = googlemaps.Client(key=api_key)
        self._max_nearby_places = 6

    def get_position(self, query: str, current_location: LatLon | None = None) -> Position | None:
        # Use location bias if current location is provided
        if current_location:
            geocode_results = self._client.geocode(
                query,
                bounds={
                    "southwest": {
                        "lat": current_location.lat - 0.5,
                        "lng": current_location.lon - 0.5,
                    },
                    "northeast": {
                        "lat": current_location.lat + 0.5,
                        "lng": current_location.lon + 0.5,
                    },
                },
            )
        else:
            geocode_results = self._client.geocode(query)

        if not geocode_results:
            return None

        result = geocode_results[0]

        location = result["geometry"]["location"]

        return Position(
            lat=location["lat"],
            lon=location["lng"],
            description=result["formatted_address"],
        )

    def get_position_with_places(
        self, query: str, current_location: LatLon | None = None
    ) -> PlacePosition | None:
        # Use location bias if current location is provided
        if current_location:
            places_results = self._client.places(
                query,
                location=(current_location.lat, current_location.lon),
                radius=50000,  # 50km radius for location bias
            )
        else:
            places_results = self._client.places(query)

        if not places_results or "results" not in places_results:
            return None

        results = places_results["results"]
        if not results:
            return None

        place = results[0]

        location = place["geometry"]["location"]

        return PlacePosition(
            lat=location["lat"],
            lon=location["lng"],
            description=place.get("name", ""),
            address=place.get("formatted_address", ""),
            types=place.get("types", []),
        )

    def get_location_context(
        self, latlon: LatLon, radius: int = 100, n_nearby_places: int = 6
    ) -> LocationContext | None:
        reverse_geocode_results = self._client.reverse_geocode((latlon.lat, latlon.lon))

        if not reverse_geocode_results:
            return None

        result = reverse_geocode_results[0]

        # Extract address components
        components = {}
        for component in result.get("address_components", []):
            types = component.get("types", [])
            if "street_number" in types:
                components["street_number"] = component["long_name"]
            elif "route" in types:
                components["street"] = component["long_name"]
            elif "neighborhood" in types:
                components["neighborhood"] = component["long_name"]
            elif "locality" in types:
                components["locality"] = component["long_name"]
            elif "administrative_area_level_1" in types:
                components["admin_area"] = component["long_name"]
            elif "country" in types:
                components["country"] = component["long_name"]
            elif "postal_code" in types:
                components["postal_code"] = component["long_name"]

        nearby_places, place_types_summary = self._get_nearby_places(
            latlon, radius, n_nearby_places
        )

        return LocationContext(
            formatted_address=result.get("formatted_address", ""),
            street_number=components.get("street_number", ""),
            street=components.get("street", ""),
            neighborhood=components.get("neighborhood", ""),
            locality=components.get("locality", ""),
            admin_area=components.get("admin_area", ""),
            country=components.get("country", ""),
            postal_code=components.get("postal_code", ""),
            nearby_places=nearby_places,
            place_types_summary=place_types_summary or "No specific landmarks nearby",
            coordinates=Coordinates(lat=latlon.lat, lon=latlon.lon),
        )

    def _get_nearby_places(
        self, latlon: LatLon, radius: int, n_nearby_places: int
    ) -> tuple[list[NearbyPlace], str]:
        nearby_places = []
        place_types_count: dict[str, int] = {}

        places_nearby = self._client.places_nearby(location=(latlon.lat, latlon.lon), radius=radius)

        if places_nearby and "results" in places_nearby:
            for place in places_nearby["results"][:n_nearby_places]:
                place_lat = place["geometry"]["location"]["lat"]
                place_lon = place["geometry"]["location"]["lng"]
                place_latlon = LatLon(lat=place_lat, lon=place_lon)

                place_info = NearbyPlace(
                    name=place.get("name", ""),
                    types=place.get("types", []),
                    vicinity=place.get("vicinity", ""),
                    distance=round(distance_in_meters(place_latlon, latlon), 1),
                )

                nearby_places.append(place_info)

                for place_type in place.get("types", []):
                    if place_type not in ["point_of_interest", "establishment"]:
                        place_types_count[place_type] = place_types_count.get(place_type, 0) + 1
            nearby_places.sort(key=lambda x: x.distance)

        place_types_summary = ", ".join(
            [
                f"{count} {ptype.replace('_', ' ')}{'s' if count > 1 else ''}"
                for ptype, count in sorted(
                    place_types_count.items(), key=lambda x: x[1], reverse=True
                )[:5]
            ]
        )

        return nearby_places, place_types_summary
