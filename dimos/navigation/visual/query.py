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

from dimos.models.qwen.bbox import BBox
from dimos.models.vl.base import VlModel
from dimos.msgs.sensor_msgs.Image import Image
from dimos.utils.generic import extract_json_from_llm_response


def get_object_bbox_from_image(
    vl_model: VlModel, image: Image, object_description: str
) -> BBox | None:
    prompt = (
        f"Look at this image and find the '{object_description}'. "
        'Return ONLY strict JSON with double quotes in this format: '
        '{"name":"object_name","bbox":[x1,y1,x2,y2]}. '
        "Use a normalized 0-1000 coordinate scale for every bbox value, where "
        "(0,0) is the image top-left and (1000,1000) is the image bottom-right. "
        "x1,y1 is the top-left and x2,y2 is the bottom-right corner. "
        "If not found, return null."
    )

    response = vl_model.query(image, prompt)

    result = extract_json_from_llm_response(response)
    if not result:
        return None

    try:
        normalized = tuple(map(float, result["bbox"]))
        if (
            len(normalized) == 4
            and all(0.0 <= value <= 1000.0 for value in normalized)
            and normalized[0] < normalized[2]
            and normalized[1] < normalized[3]
        ):
            return (
                normalized[0] * image.width / 1000.0,
                normalized[1] * image.height / 1000.0,
                normalized[2] * image.width / 1000.0,
                normalized[3] * image.height / 1000.0,
            )
    except Exception:
        pass

    return None
