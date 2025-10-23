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


from dimos.models.qwen.video_query import BBox
from dimos.models.vl.base import VlModel
from dimos.msgs.sensor_msgs import Image
from dimos.utils.generic import extract_json_from_llm_response


def get_object_bbox_from_image(
    vl_model: VlModel, image: Image, object_description: str
) -> BBox | None:
    prompt = (
        f"Look at this image and find the '{object_description}'. "
        "Return ONLY a JSON object with format: {'name': 'object_name', 'bbox': [x1, y1, x2, y2]} "
        "where x1,y1 is the top-left and x2,y2 is the bottom-right corner of the bounding box. If not found, return None."
    )

    response = vl_model.query(image, prompt)

    result = extract_json_from_llm_response(response)
    if not result:
        return None

    try:
        ret = tuple(map(float, result["bbox"]))
        if len(ret) == 4:
            return ret
    except Exception:
        pass

    return None
