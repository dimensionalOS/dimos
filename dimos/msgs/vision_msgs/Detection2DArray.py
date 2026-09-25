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
from typing import TypedDict

from dimos_lcm.vision_msgs.Detection2DArray import (
    Detection2DArray as LCMDetection2DArray,
)

from dimos.types.timestamped import to_timestamp


class BBoxJson(TypedDict):
    cx: float
    cy: float
    w: float
    h: float


class Detection2DJson(TypedDict):
    label: str
    score: float
    bbox: BBoxJson


class Detection2DArray(LCMDetection2DArray):  # type: ignore[misc]
    msg_name = "vision_msgs.Detection2DArray"

    # for _get_field_type() to work when decoding in _decode_one()
    __annotations__ = LCMDetection2DArray.__annotations__

    @property
    def ts(self) -> float:
        return to_timestamp(self.header.stamp)

    def to_json(self) -> list[Detection2DJson]:
        out: list[Detection2DJson] = []
        for d in self.detections[: self.detections_length]:
            results = d.results[: d.results_length]
            c = d.bbox.center.position
            out.append(
                {
                    "label": next(
                        (str(r.hypothesis.class_id) for r in results if r.hypothesis.class_id), ""
                    ),
                    "score": round(max((r.hypothesis.score for r in results), default=0.0), 2),
                    "bbox": {
                        "cx": round(c.x, 1),
                        "cy": round(c.y, 1),
                        "w": round(d.bbox.size_x, 1),
                        "h": round(d.bbox.size_y, 1),
                    },
                }
            )
        return out
