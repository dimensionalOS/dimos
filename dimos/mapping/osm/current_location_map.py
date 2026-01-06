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

from PIL import Image as PILImage, ImageDraw

from dimos.mapping.osm.osm import MapImage, get_osm_map
from dimos.mapping.osm.query import query_for_one_position, query_for_one_position_and_context
from dimos.mapping.types import LatLon
from dimos.models.vl.base import VlModel
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class CurrentLocationMap:
    _vl_model: VlModel
    _position: LatLon | None
    _map_image: MapImage | None

    def __init__(self, vl_model: VlModel) -> None:
        self._vl_model = vl_model
        self._position = None
        self._map_image = None
        self._zoom_level = 15
        self._n_tiles = 6
        # What ratio of the width is considered the center. 1.0 means the entire map is the center.
        self._center_width = 0.4

    def update_position(self, position: LatLon) -> None:
        self._position = position

    def query_for_one_position(self, query: str) -> LatLon | None:
        return query_for_one_position(self._vl_model, self._get_current_map(), query)  # type: ignore[no-untyped-call]

    def query_for_one_position_and_context(
        self, query: str, robot_position: LatLon
    ) -> tuple[LatLon, str] | None:
        return query_for_one_position_and_context(
            self._vl_model,
            self._get_current_map(),  # type: ignore[no-untyped-call]
            query,
            robot_position,
        )

    def _get_current_map(self):  # type: ignore[no-untyped-def]
        if not self._position:
            raise ValueError("Current position has not been set.")

        if not self._map_image or self._position_is_too_far_off_center():
            self._fetch_new_map()
            return self._map_image

        return self._map_image

    def _fetch_new_map(self) -> None:
        logger.info(
            f"Getting a new OSM map, position={self._position}, zoom={self._zoom_level} n_tiles={self._n_tiles}"
        )
        self._map_image = get_osm_map(self._position, self._zoom_level, self._n_tiles)  # type: ignore[arg-type]

        # Add position marker
        import numpy as np

        assert self._map_image is not None
        assert self._position is not None
        pil_image = PILImage.fromarray(self._map_image.image.data)
        draw = ImageDraw.Draw(pil_image)
        x, y = self._map_image.latlon_to_pixel(self._position)
        radius = 20
        draw.ellipse(
            [x - radius, y - radius, x + radius, y + radius],
            fill=(255, 0, 0),
            outline=(0, 0, 0),
            width=3,
        )

        self._map_image.image.data[:] = np.array(pil_image)

    def _position_is_too_far_off_center(self) -> bool:
        x, y = self._map_image.latlon_to_pixel(self._position)  # type: ignore[arg-type, union-attr]
        width = self._map_image.image.width  # type: ignore[union-attr]
        size_min = width * (0.5 - self._center_width / 2)
        size_max = width * (0.5 + self._center_width / 2)

        return x < size_min or x > size_max or y < size_min or y > size_max

    def save_current_map_image(self, filepath: str = "osm_debug_map.png") -> str:
        """Save the current OSM map image to a file for debugging.

        Args:
            filepath: Path where to save the image

        Returns:
            The filepath where the image was saved
        """
        if not self._map_image:
            self._get_current_map()  # type: ignore[no-untyped-call]

        if self._map_image is not None:
            self._map_image.image.save(filepath)
        logger.info(f"Saved OSM map image to {filepath}")
        return filepath
