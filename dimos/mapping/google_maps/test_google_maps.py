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


from dimos.mapping.types import LatLon


def test_get_position(maps_client, maps_fixture) -> None:
    maps_client._client.geocode.return_value = maps_fixture("get_position.json")

    res = maps_client.get_position("golden gate bridge")

    assert res.model_dump() == {
        "description": "Golden Gate Bridge, Golden Gate Brg, San Francisco, CA, USA",
        "lat": 37.8199109,
        "lon": -122.4785598,
    }


def test_get_position_with_places(maps_client, maps_fixture) -> None:
    maps_client._client.places.return_value = maps_fixture("get_position_with_places.json")

    res = maps_client.get_position_with_places("golden gate bridge")

    assert res.model_dump() == {
        "address": "Golden Gate Brg, San Francisco, CA, United States",
        "description": "Golden Gate Bridge",
        "lat": 37.8199109,
        "lon": -122.4785598,
        "types": [
            "tourist_attraction",
            "point_of_interest",
            "establishment",
        ],
    }


def test_get_location_context(maps_client, maps_fixture) -> None:
    maps_client._client.reverse_geocode.return_value = maps_fixture(
        "get_location_context_reverse_geocode.json"
    )
    maps_client._client.places_nearby.return_value = maps_fixture(
        "get_location_context_places_nearby.json"
    )

    res = maps_client.get_location_context(LatLon(lat=37.78017758753598, lon=-122.4144951709186))

    assert res.model_dump() == {
        "admin_area": "California",
        "coordinates": {
            "lat": 37.78017758753598,
            "lon": -122.4144951709186,
        },
        "country": "United States",
        "formatted_address": "50 United Nations Plaza, San Francisco, CA 94102, USA",
        "locality": "San Francisco",
        "nearby_places": [
            {
                "distance": 9.3,
                "name": "U.S. General Services Administration - Pacific Rim Region",
                "types": [
                    "point_of_interest",
                    "establishment",
                ],
                "vicinity": "50 United Nations Plaza, San Francisco",
            },
            {
                "distance": 14.0,
                "name": "Federal Office Building",
                "types": [
                    "point_of_interest",
                    "establishment",
                ],
                "vicinity": "50 United Nations Plaza, San Francisco",
            },
            {
                "distance": 35.7,
                "name": "UN Plaza",
                "types": [
                    "city_hall",
                    "point_of_interest",
                    "local_government_office",
                    "establishment",
                ],
                "vicinity": "355 McAllister Street, San Francisco",
            },
            {
                "distance": 92.7,
                "name": "McAllister Market & Deli",
                "types": [
                    "liquor_store",
                    "atm",
                    "grocery_or_supermarket",
                    "finance",
                    "point_of_interest",
                    "food",
                    "store",
                    "establishment",
                ],
                "vicinity": "136 McAllister Street, San Francisco",
            },
            {
                "distance": 95.9,
                "name": "Civic Center / UN Plaza",
                "types": [
                    "subway_station",
                    "transit_station",
                    "point_of_interest",
                    "establishment",
                ],
                "vicinity": "1150 Market Street, San Francisco",
            },
            {
                "distance": 726.3,
                "name": "San Francisco",
                "types": [
                    "locality",
                    "political",
                ],
                "vicinity": "San Francisco",
            },
        ],
        "neighborhood": "Civic Center",
        "place_types_summary": "1 locality, 1 political, 1 subway station, 1 transit station, 1 city hall",
        "postal_code": "94102",
        "street": "United Nations Plaza",
        "street_number": "50",
    }
