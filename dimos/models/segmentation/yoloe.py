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

"""CPU-capable YOLO-E mask refinement for externally supplied boxes."""

from __future__ import annotations

from dataclasses import replace
from typing import Protocol

import numpy as np
from numpy.typing import NDArray

from dimos.msgs.sensor_msgs.Image import Image
from dimos.perception.detection.detectors.yoloe import Yoloe2DDetector, YoloePromptMode
from dimos.perception.detection.type.detection2d.bbox import Bbox, Detection2DBBox
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D
from dimos.perception.detection.type.detection2d.seg import Detection2DSeg
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class YoloeVisualPromptDetector(Protocol):
    """Subset of YOLO-E used for box-prompted mask refinement."""

    def set_prompts(
        self,
        text: list[str] | None = None,
        bboxes: NDArray[np.float64] | None = None,
    ) -> None: ...

    def predict_image(self, image: Image) -> ImageDetections2D: ...

    def stop(self) -> None: ...


def _intersection_over_union(left: Bbox, right: Bbox) -> float:
    x1 = max(left[0], right[0])
    y1 = max(left[1], right[1])
    x2 = min(left[2], right[2])
    y2 = min(left[3], right[3])
    intersection = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    left_area = max(0.0, left[2] - left[0]) * max(0.0, left[3] - left[1])
    right_area = max(0.0, right[2] - right[0]) * max(0.0, right[3] - right[1])
    union = left_area + right_area - intersection
    return intersection / union if union > 0.0 else 0.0


class YoloeBoxSegmenter:
    """Turn external box detections into masks using YOLO-E visual prompts."""

    def __init__(
        self,
        confidence: float = 0.05,
        detector: YoloeVisualPromptDetector | None = None,
    ) -> None:
        self._detector = detector or Yoloe2DDetector(
            model_name="yoloe-11s-seg.pt",
            prompt_mode=YoloePromptMode.PROMPT,
            max_area_ratio=None,
            conf=confidence,
        )

    def segment(
        self, detections: ImageDetections2D[Detection2DBBox]
    ) -> ImageDetections2D[Detection2DSeg]:
        """Refine every input box into one mask while preserving its metadata."""
        if not detections:
            return ImageDetections2D(detections.image, [])

        boxes = np.asarray([detection.bbox for detection in detections], dtype=np.float64)
        self._detector.set_prompts(bboxes=boxes)
        candidates = self._detector.predict_image(detections.image)

        masks_by_prompt: dict[int, list[Detection2DSeg]] = {}
        for candidate in candidates:
            if isinstance(candidate, Detection2DSeg):
                masks_by_prompt.setdefault(candidate.class_id, []).append(candidate)

        segmented: list[Detection2DSeg] = []
        for prompt_id, detection in enumerate(detections):
            matches = masks_by_prompt.get(prompt_id, [])
            if not matches:
                continue
            mask = max(
                matches,
                key=lambda candidate: _intersection_over_union(detection.bbox, candidate.bbox),
            )
            segmented.append(
                replace(
                    mask,
                    track_id=detection.track_id,
                    class_id=detection.class_id,
                    confidence=detection.confidence,
                    name=detection.name,
                    ts=detection.ts,
                    image=detection.image,
                )
            )

        if len(segmented) != len(detections):
            logger.warning(
                "YOLO-E did not produce a mask for every box",
                boxes=len(detections),
                masks=len(segmented),
            )
        return ImageDetections2D(detections.image, segmented)

    def stop(self) -> None:
        """Release the underlying YOLO-E predictor."""
        self._detector.stop()
