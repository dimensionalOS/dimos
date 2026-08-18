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

"""Explicit-image language grounding followed by promptable segmentation."""

from __future__ import annotations

from collections.abc import Callable
from typing import Protocol

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.models.segmentation.edge_tam import EdgeTAMImageSegmenter
from dimos.models.vl.base import VlModel
from dimos.models.vl.moondream import MoondreamVlModel
from dimos.msgs.sensor_msgs.Image import Image
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D
from dimos.perception.detection.type.detection2d.seg import Detection2DSeg
from dimos.spec.utils import Spec


class GroundedSegmentationSpec(Spec, Protocol):
    """General on-demand text-to-mask perception interface."""

    def segment(
        self,
        image: Image,
        prompts: list[str],
    ) -> ImageDetections2D[Detection2DSeg]: ...

    def segment_best(self, image: Image, prompt: str) -> ImageDetections2D[Detection2DSeg]: ...


class GroundedSegmentationConfig(ModuleConfig):
    grounder: Callable[[], VlModel] = MoondreamVlModel
    segmenter: Callable[[], EdgeTAMImageSegmenter] = EdgeTAMImageSegmenter


class GroundedSegmentationModule(Module, GroundedSegmentationSpec):
    """Ground text in one supplied image and return typed segmentation masks."""

    dedicated_worker = True
    config: GroundedSegmentationConfig

    def __init__(self, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(**kwargs)
        self._grounder: VlModel | None = None
        self._segmenter: EdgeTAMImageSegmenter | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self._grounder = self.config.grounder()
        self._segmenter = self.config.segmenter()
        self._grounder.start()

    @rpc
    def stop(self) -> None:
        if self._grounder is not None:
            self._grounder.stop()
        self._grounder = None
        self._segmenter = None
        super().stop()

    @rpc
    def segment(
        self,
        image: Image,
        prompts: list[str],
    ) -> ImageDetections2D[Detection2DSeg]:
        """Return masks for objects matching text prompts in the supplied image."""
        if self._grounder is None or self._segmenter is None:
            raise RuntimeError("Grounded segmentation module has not been started")
        normalized = tuple(dict.fromkeys(prompt.strip() for prompt in prompts if prompt.strip()))
        if not normalized:
            raise ValueError("prompts must contain at least one non-empty description")
        boxes: list[Detection2DBBox] = []
        for prompt in normalized:
            grounded = self._grounder.query_detections(image, prompt)
            for detection in grounded:
                detection.track_id = len(boxes)
                boxes.append(detection)
        if not boxes:
            return ImageDetections2D(image)
        segmented = self._segmenter.segment(ImageDetections2D(image, boxes))
        return ImageDetections2D(
            image,
            [detection for detection in segmented if isinstance(detection, Detection2DSeg)],
        )

    @rpc
    def segment_best(
        self,
        image: Image,
        prompt: str,
    ) -> ImageDetections2D[Detection2DSeg]:
        """Return one point-grounded mask for a uniquely described object."""
        if self._grounder is None or self._segmenter is None:
            raise RuntimeError("Grounded segmentation module has not been started")
        normalized = prompt.strip()
        if not normalized:
            raise ValueError("prompt must be non-empty")
        points = self._grounder.query_points(image, normalized)
        if not points:
            return ImageDetections2D(image)
        point = points[0]
        point.track_id = 0
        point.name = normalized
        return self._segmenter.segment_points(ImageDetections2D(image, [point]))
