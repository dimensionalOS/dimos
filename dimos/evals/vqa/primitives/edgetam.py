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

"""EdgeTAM-segmented LiDAR range evidence for VQA object questions."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Protocol, cast

import numpy as np

from dimos.evals.vqa.contracts import InsufficientEvidenceError
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D

if TYPE_CHECKING:
    from dimos.evals.vqa.contracts import ObjectDetector
    from dimos.msgs.sensor_msgs.Image import Image


@dataclass(frozen=True)
class ObjectMaskEvidence:
    """One validated box-prompted object mask."""

    object_name: str
    prompt_bbox_xyxy: tuple[float, float, float, float]
    detection: Detection2DBBox
    mask: np.ndarray

    @property
    def mask_bbox_xyxy(self) -> tuple[float, float, float, float]:
        return cast("tuple[float, float, float, float]", tuple(map(float, self.detection.bbox)))

    @property
    def mask_area_px(self) -> int:
        return int(np.count_nonzero(self.mask))


class EdgeTAMImageSegmenterCompatible(Protocol):
    """The box-prompt operation used from ``EdgeTAMImageSegmenter``."""

    def segment(
        self, detections: ImageDetections2D[Detection2DBBox]
    ) -> ImageDetections2D[Detection2DBBox]: ...


class EdgeTamObjectMaskEstimator:
    """Detect and batch-segment named objects, caching evidence per image."""

    def __init__(
        self,
        detector: ObjectDetector,
        segmenter: EdgeTAMImageSegmenterCompatible | None = None,
    ) -> None:
        self._detector = detector
        self._segmenter = segmenter
        self._cached_image_key: tuple[int, int, float] | None = None
        self._cached_evidence: dict[str, ObjectMaskEvidence] = {}

    def estimate(self, image: Image, object_name: str) -> ObjectMaskEvidence:
        """Return one named object's validated mask."""
        return self.estimate_many(image, (object_name,))[0]

    def estimate_many(
        self,
        image: Image,
        object_names: tuple[str, ...],
    ) -> tuple[ObjectMaskEvidence, ...]:
        """Return several object masks from one EdgeTAM segmentation pass."""
        if not object_names:
            raise ValueError("object_names must not be empty")
        normalized_names = tuple(name.strip() for name in object_names)
        if any(not name for name in normalized_names):
            raise ValueError("object names must not be blank")
        if len({name.casefold() for name in normalized_names}) != len(normalized_names):
            raise ValueError("object names must be distinct")

        image_key = (id(image), id(image.data), image.ts)
        if image_key != self._cached_image_key:
            self._cached_image_key = image_key
            self._cached_evidence.clear()

        pending: list[tuple[str, Detection2DBBox]] = []
        for object_name in normalized_names:
            if object_name in self._cached_evidence:
                continue
            detected = self._detector.detect(image, object_name)
            valid_detections = [detection for detection in detected if detection.is_valid()]
            if len(valid_detections) != 1:
                raise InsufficientEvidenceError(
                    f"object mask requires exactly one valid detected {object_name!r}, "
                    f"got {len(valid_detections)}"
                )
            pending.append((object_name, valid_detections[0]))

        if pending:
            segmented = self._get_segmenter().segment(
                ImageDetections2D(image, [detection for _, detection in pending])
            )
            if len(segmented) != len(pending):
                raise InsufficientEvidenceError(
                    "object mask requires one segmentation result per detected object, "
                    f"got {len(segmented)} for {len(pending)} objects"
                )
            expected_shape = (image.height, image.width)
            for (object_name, detection), candidate in zip(pending, segmented, strict=True):
                mask = getattr(candidate, "mask", None)
                if not (
                    candidate.is_valid()
                    and isinstance(mask, np.ndarray)
                    and mask.ndim == 2
                    and mask.shape == expected_shape
                    and bool(np.any(mask))
                ):
                    raise InsufficientEvidenceError(
                        "object mask requires one valid segmentation mask matching "
                        f"the rectified image for {object_name!r}"
                    )
                bbox = detection.bbox
                self._cached_evidence[object_name] = ObjectMaskEvidence(
                    object_name=object_name,
                    prompt_bbox_xyxy=(
                        float(bbox[0]),
                        float(bbox[1]),
                        float(bbox[2]),
                        float(bbox[3]),
                    ),
                    detection=candidate,
                    mask=mask,
                )
        return tuple(self._cached_evidence[name] for name in normalized_names)

    def _get_segmenter(self) -> EdgeTAMImageSegmenterCompatible:
        if self._segmenter is None:
            from dimos.models.segmentation.edge_tam import EdgeTAMImageSegmenter

            self._segmenter = EdgeTAMImageSegmenter()
        return self._segmenter


class ObjectMaskEstimator(Protocol):
    """Estimate object masks from an image."""

    def estimate(self, image: Image, object_name: str) -> ObjectMaskEvidence: ...

    def estimate_many(
        self, image: Image, object_names: tuple[str, ...]
    ) -> tuple[ObjectMaskEvidence, ...]: ...
