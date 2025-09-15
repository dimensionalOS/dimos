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

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Tuple

from dimos.msgs.sensor_msgs import Image
from dimos.types.timestamped import Timestamped

Bbox = Tuple[float, float, float, float]
CenteredBbox = Tuple[float, float, float, float]

InconvinientDetectionFormat = Tuple[List[Bbox], List[int], List[int], List[float], List[List[str]]]


Bbox = Tuple[float, float, float, float]
CenteredBbox = Tuple[float, float, float, float]
# yolo and detic have bad output formats
InconvinientDetectionFormat = Tuple[List[Bbox], List[int], List[int], List[float], List[List[str]]]


Detection = Tuple[Bbox, int, int, float, str]
Detections = List[Detection]


# yolo and detic have bad formats this translates into list of detections
def better_detection_format(inconvinient_detections: InconvinientDetectionFormat) -> Detections:
    bboxes, track_ids, class_ids, confidences, names = inconvinient_detections
    return [
        [bbox, track_id, class_id, confidence, name]
        for bbox, track_id, class_id, confidence, name in zip(
            bboxes, track_ids, class_ids, confidences, names
        )
    ]


@dataclass
class Detection2D(Timestamped):
    bbox: Bbox
    track_id: int
    class_id: int
    confidence: float
    name: str
    ts: float = 0.0
    image: Optional[Image] = None

    @classmethod
    def from_detection(cls, raw_detection: Detection, **kwargs) -> "Detection2D":
        [bbox, track_id, class_id, confidence, name] = raw_detection

        if kwargs.get("image", None) is not None:
            kwargs["ts"] = kwargs.get("image").ts

        return cls(
            bbox=bbox,
            track_id=track_id,
            class_id=class_id,
            confidence=confidence,
            name=name,
            **kwargs,
        )


@dataclass
class LocalizedDetection2D(Detection2D):
    def localize(self, pointcloud): ...
