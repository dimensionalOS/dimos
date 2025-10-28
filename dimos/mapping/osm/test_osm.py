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

from collections.abc import Generator
from typing import Any

import cv2
import numpy as np
import pytest
from requests import Request
import requests_mock

from dimos.mapping.osm.osm import get_osm_map
from dimos.mapping.types import LatLon
from dimos.utils.data import get_data

_fixture_dir = get_data("osm_map_test")


def _tile_callback(request: Request, context: Any) -> bytes:
    parts = (request.url or "").split("/")
    zoom, x, y_png = parts[-3], parts[-2], parts[-1]
    y = y_png.removesuffix(".png")
    tile_path = _fixture_dir / f"{zoom}_{x}_{y}.png"
    context.headers["Content-Type"] = "image/png"
    return tile_path.read_bytes()


@pytest.fixture
def mock_openstreetmap_org() -> Generator[None, None, None]:
    with requests_mock.Mocker() as m:
        m.get(requests_mock.ANY, content=_tile_callback)
        yield


def test_get_osm_map(mock_openstreetmap_org: None) -> None:
    position = LatLon(lat=37.751857, lon=-122.431265)
    map_image = get_osm_map(position, 18, 4)

    assert map_image.position == position
    assert map_image.n_tiles == 4

    expected_image = cv2.imread(str(_fixture_dir / "full.png"))
    expected_image_rgb = cv2.cvtColor(expected_image, cv2.COLOR_BGR2RGB)
    assert np.array_equal(map_image.image.data, expected_image_rgb), "Map is not the same."


def test_pixel_to_latlon(mock_openstreetmap_org: None) -> None:
    position = LatLon(lat=37.751857, lon=-122.431265)
    map_image = get_osm_map(position, 18, 4)
    latlon = map_image.pixel_to_latlon((100, 100))
    assert abs(latlon.lat - 37.7540056) < 0.0000001
    assert abs(latlon.lon - (-122.43385076)) < 0.0000001


def test_latlon_to_pixel(mock_openstreetmap_org: None) -> None:
    position = LatLon(lat=37.751857, lon=-122.431265)
    map_image = get_osm_map(position, 18, 4)
    coords = map_image.latlon_to_pixel(LatLon(lat=37.751, lon=-122.431))
    assert coords == (631, 808)
