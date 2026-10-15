# Copyright 2026 Dimensional Inc.
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

from pathlib import Path
from unittest.mock import MagicMock

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.std_msgs.msg import Header
import numpy as np
from PIL import Image as PILImage
import pytest

from dimos.mapping.models import LatLon
from dimos.mapping.osm import current_location_map
from dimos.mapping.osm.current_location_map import CurrentLocationMap
from dimos.mapping.osm.osm import MapImage
from dimos.models.vl.base import VlModel
from dimos.msgs.image import image_from_array, image_to_rgb


def test_position_marker_preserves_source_stamp_color_and_cached_tile(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    position = LatLon(lat=0, lon=0)
    pixels = np.full((256, 256, 3), [0, 10, 30], dtype=np.uint8)
    source = image_from_array(
        pixels, encoding="bgr8", header=Header(frame_id="map", stamp=Time(sec=123, nanosec=456))
    )
    tile = MapImage(source, position, 0, 1)
    monkeypatch.setattr(current_location_map, "get_osm_map", lambda *_args: tile)
    current = CurrentLocationMap(MagicMock(spec=VlModel))
    current.update_position(position)
    current._fetch_new_map()
    assert current._map_image is not None
    image = current._map_image.image
    assert image.header == source.header
    assert image.encoding == "rgb8"
    x, y = tile.latlon_to_pixel(position)
    np.testing.assert_array_equal(image_to_rgb(image)[y, x], [255, 0, 0])
    np.testing.assert_array_equal(image_to_rgb(image)[0, 0], [30, 10, 0])
    np.testing.assert_array_equal(image_to_rgb(source)[y, x], [30, 10, 0])
    output = tmp_path / "map.png"
    assert current.save_current_map_image(str(output)) == str(output)
    with PILImage.open(output) as saved:
        np.testing.assert_array_equal(np.asarray(saved), image_to_rgb(image))
