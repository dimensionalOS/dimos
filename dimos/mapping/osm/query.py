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

import re

from dimos.mapping.osm.osm import MapImage
from dimos.mapping.types import LatLon
from dimos.models.vl.base import VlModel
from dimos.utils.generic import extract_json_from_llm_response
from dimos.utils.logging_config import setup_logger

_PROLOGUE = "This is an image of an open street map I'm on."
_JSON = "Please only respond with valid JSON."
logger = setup_logger()


def query_for_one_position(vl_model: VlModel, map_image: MapImage, query: str) -> LatLon | None:
    full_query = f"{_PROLOGUE} {query} {_JSON} If there's a match return the x, y coordinates from the image. Example: `[123, 321]`. If there's no match return `null`."
    response = vl_model.query(map_image.image.data, full_query)
    coords = tuple(map(int, re.findall(r"\d+", response)))
    if len(coords) != 2:
        return None
    return map_image.pixel_to_latlon(coords)


def query_for_one_position_and_context(
    vl_model: VlModel, map_image: MapImage, query: str, robot_position: LatLon
) -> tuple[LatLon, str] | None:
    example = '{"coordinates": [123, 321], "description": "A Starbucks on 27th Street"}'
    x, y = map_image.latlon_to_pixel(robot_position)
    my_location = f"I'm currently at x={x}, y={y}."
    full_query = f"{_PROLOGUE} {my_location} {query} {_JSON} If there's a match return the x, y coordinates from the image and what is there. Example response: `{example}`. If there's no match return `null`."
    logger.info(f"Qwen query: `{full_query}`")
    response = vl_model.query(map_image.image.data, full_query)

    try:
        doc = extract_json_from_llm_response(response)
        return map_image.pixel_to_latlon(tuple(doc["coordinates"])), str(doc["description"])
    except Exception:
        pass

    # TODO: Try more simplictic methods to parse.
    return None
