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

from dataclasses import dataclass
import math
import io
from typing import Tuple
import requests
import numpy as np
from PIL import Image as PILImage

from dimos.mapping.types import ImageCoord, LatLon
from dimos.msgs.sensor_msgs import Image, ImageFormat


@dataclass(frozen=True)
class MapImage:
    image: Image
    position: LatLon
    zoom_level: int
    n_tiles: int

    def pixel_to_latlon(self, position: ImageCoord) -> LatLon:
        """Convert pixel coordinates to latitude/longitude.

        Args:
            position: (x, y) pixel coordinates in the image

        Returns:
            LatLon object with the corresponding latitude and longitude
        """
        pixel_x, pixel_y = position
        tile_size = 256

        # Get the center tile coordinates
        center_tile_x, center_tile_y = _lat_lon_to_tile(
            self.position.lat, self.position.lon, self.zoom_level
        )

        # Calculate the actual top-left tile indices (integers)
        start_tile_x = int(center_tile_x - self.n_tiles / 2.0)
        start_tile_y = int(center_tile_y - self.n_tiles / 2.0)

        # Convert pixel position to exact tile coordinates
        tile_x = start_tile_x + pixel_x / tile_size
        tile_y = start_tile_y + pixel_y / tile_size

        # Convert tile coordinates to lat/lon
        n = 2**self.zoom_level
        lon = tile_x / n * 360.0 - 180.0
        lat_rad = math.atan(math.sinh(math.pi * (1 - 2 * tile_y / n)))
        lat = math.degrees(lat_rad)

        return LatLon(lat=lat, lon=lon)

    def latlon_to_pixel(self, position: LatLon) -> ImageCoord:
        """Convert latitude/longitude to pixel coordinates.

        Args:
            position: LatLon object with latitude and longitude

        Returns:
            (x, y) pixel coordinates in the image
            Note: Can return negative values if position is outside the image bounds
        """
        tile_size = 256

        # Convert the input lat/lon to tile coordinates
        tile_x, tile_y = _lat_lon_to_tile(position.lat, position.lon, self.zoom_level)

        # Get the center tile coordinates
        center_tile_x, center_tile_y = _lat_lon_to_tile(
            self.position.lat, self.position.lon, self.zoom_level
        )

        # Calculate the actual top-left tile indices (integers)
        start_tile_x = int(center_tile_x - self.n_tiles / 2.0)
        start_tile_y = int(center_tile_y - self.n_tiles / 2.0)

        # Calculate pixel position relative to top-left corner
        pixel_x = int((tile_x - start_tile_x) * tile_size)
        pixel_y = int((tile_y - start_tile_y) * tile_size)

        return (pixel_x, pixel_y)


def _lat_lon_to_tile(lat: float, lon: float, zoom: int) -> Tuple[float, float]:
    """Convert latitude/longitude to tile coordinates at given zoom level."""
    n = 2**zoom
    x_tile = (lon + 180.0) / 360.0 * n
    lat_rad = math.radians(lat)
    y_tile = (1.0 - math.asinh(math.tan(lat_rad)) / math.pi) / 2.0 * n
    return x_tile, y_tile


def get_osm_map(position: LatLon, zoom_level: int = 18, n_tiles: int = 4) -> MapImage:
    """
    Tiles are always 256x256 pixels. With n_tiles=4, this should produce a 1024x1024 image.

    Args:
        position (LatLon): center position
        zoom_level (int, optional): Defaults to 18.
        n_tiles (int, optional): generate a map of n_tiles by n_tiles.
    """
    center_x, center_y = _lat_lon_to_tile(position.lat, position.lon, zoom_level)

    start_x = int(center_x - n_tiles / 2.0)
    start_y = int(center_y - n_tiles / 2.0)

    tile_size = 256
    output_size = tile_size * n_tiles
    output_img = PILImage.new("RGB", (output_size, output_size))

    headers = {"User-Agent": "Dimos OSM Client/1.0"}

    n_failed_tiles = 0

    for row in range(n_tiles):
        for col in range(n_tiles):
            tile_x = start_x + col
            tile_y = start_y + row

            url = f"https://tile.openstreetmap.org/{zoom_level}/{tile_x}/{tile_y}.png"

            try:
                response = requests.get(url, headers=headers, timeout=10)
                response.raise_for_status()

                tile_img = PILImage.open(io.BytesIO(response.content))

                paste_x = col * tile_size
                paste_y = row * tile_size

                output_img.paste(tile_img, (paste_x, paste_y))

            except Exception:
                n_failed_tiles += 1

    if n_failed_tiles > 3:
        raise ValueError("Failed to download all tiles for the requested map.")

    return MapImage(
        image=Image.from_numpy(np.array(output_img), format=ImageFormat.RGB),
        position=position,
        zoom_level=zoom_level,
        n_tiles=n_tiles,
    )
