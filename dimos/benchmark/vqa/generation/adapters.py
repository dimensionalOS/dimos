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

"""MoonDream and EdgeTAM adapters for the single-frame VQA pipeline."""

from __future__ import annotations

from dimos.models.segmentation.edge_tam import EdgeTAMImageSegmenter
from dimos.models.vl.moondream import MoondreamVlModel
from dimos.msgs.sensor_msgs.Image import Image
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D


class MoondreamObjectDetector:
    """Adapt MoonDream's query detection API to the grounding interface."""

    def __init__(self, model: MoondreamVlModel) -> None:
        self._model = model

    def detect(self, image: Image, query: str) -> ImageDetections2D:
        return self._model.query_detections(image, query)

    def locate(self, image: Image, query: str) -> ImageDetections2D:
        points = self._model.query_points(image, f"center of the {query}")
        for point in points:
            point.name = query
        return points


class EdgeTamObjectSegmenter:
    """Adapt EdgeTAM single-image segmentation to the grounding interface."""

    def __init__(self, segmenter: EdgeTAMImageSegmenter) -> None:
        self._segmenter = segmenter

    def segment(self, detections: ImageDetections2D) -> ImageDetections2D:
        return self._segmenter.segment(detections)

    def segment_points(self, points: ImageDetections2D) -> ImageDetections2D:
        return self._segmenter.segment_points(points)
