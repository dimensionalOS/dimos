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

from typing import TYPE_CHECKING, Annotated, Any, Protocol

import numpy as np
from pydantic import BaseModel, ConfigDict, Field, StringConstraints

from dimos.evals.vqa.families import InsufficientEvidenceError
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D
from dimos.perception.detection.type.detection3d.pointcloud import ProjectedPointCloud

ObjectName = Annotated[str, StringConstraints(strip_whitespace=True, min_length=1)]

if TYPE_CHECKING:
    from dimos.evals.vqa.families import ObjectDetector
    from dimos.evals.vqa.preprocessing import CalibratedFrame


class ObjectRangeEvidence(BaseModel):
    """Median Euclidean camera-origin range supported by foreground points."""

    model_config = ConfigDict(extra="forbid", frozen=True, allow_inf_nan=False)

    object_name: ObjectName
    camera_range_m: float = Field(ge=0)
    supporting_point_count: int = Field(ge=1)
    prompt_bbox_xyxy: tuple[float, float, float, float]
    mask_bbox_xyxy: tuple[float, float, float, float]
    mask_area_px: int = Field(ge=1)
    range_quartiles_m: tuple[float, float, float]
    synchronization_delta_s: float = Field(ge=0)
    calibration_source: str = Field(min_length=1)


class EdgeTAMImageSegmenterCompatible(Protocol):
    """The box-prompt operation used from ``EdgeTAMImageSegmenter``."""

    def segment(
        self, detections: ImageDetections2D[Detection2DBBox]
    ) -> ImageDetections2D[Detection2DBBox]: ...


class EdgeTamLidarRangeEstimator:
    """Estimate object range from box-prompted masks and calibrated LiDAR points."""

    def __init__(
        self,
        detector: ObjectDetector,
        segmenter: EdgeTAMImageSegmenterCompatible | None = None,
        min_supporting_points: int = 5,
    ) -> None:
        if min_supporting_points < 1:
            raise ValueError("min_supporting_points must be at least 1")
        self._detector = detector
        self._segmenter = segmenter
        self._min_supporting_points = min_supporting_points
        self._cached_frame: CalibratedFrame | None = None
        self._cached_projection: ProjectedPointCloud | None = None
        self._cached_evidence: dict[str, ObjectRangeEvidence] = {}

    def estimate(self, frame: CalibratedFrame, object_name: str) -> ObjectRangeEvidence:
        """Estimate robust camera-origin range to one detected object."""
        if not isinstance(object_name, str):
            raise TypeError("object_name must be a string")
        normalized_name = object_name.strip()
        if not normalized_name:
            raise ValueError("object_name must not be blank")
        if frame is not self._cached_frame:
            self._cached_frame = frame
            self._cached_projection = None
            self._cached_evidence.clear()
        cache_key = normalized_name
        if cached := self._cached_evidence.get(cache_key):
            return cached

        detections = self._detector.query_detections(frame.image, normalized_name)
        valid_detections = [detection for detection in detections if detection.is_valid()]
        if len(valid_detections) != 1:
            raise InsufficientEvidenceError(
                f"object range requires exactly one valid detected {normalized_name!r}, "
                f"got {len(valid_detections)}"
            )
        detection = valid_detections[0]

        prompted = ImageDetections2D(frame.image, [detection])
        segmented = self._get_segmenter().segment(prompted)
        valid_masks: list[tuple[Detection2DBBox, np.ndarray[Any, Any]]] = []
        expected_shape = (frame.image.height, frame.image.width)
        for candidate in segmented:
            mask = getattr(candidate, "mask", None)
            if (
                candidate.is_valid()
                and isinstance(mask, np.ndarray)
                and mask.ndim == 2
                and mask.shape == expected_shape
                and bool(np.any(mask))
            ):
                valid_masks.append((candidate, mask))
        if len(valid_masks) != 1:
            raise InsufficientEvidenceError(
                "object range requires exactly one valid segmentation mask matching "
                f"the rectified image, got {len(valid_masks)}"
            )
        segmented_detection, mask = valid_masks[0]

        if (frame.camera_info.height, frame.camera_info.width) != (
            frame.image.height,
            frame.image.width,
        ):
            raise ValueError("camera calibration dimensions do not match the rectified image")
        if self._cached_projection is None:
            self._cached_projection = ProjectedPointCloud.from_pointcloud(
                frame.pointcloud,
                frame.camera_info,
                frame.pointcloud_to_camera,
            )
        _, supporting_points = self._cached_projection.points_in_detection(
            segmented_detection,
            nearest_per_pixel=True,
        )
        if len(supporting_points) < self._min_supporting_points:
            raise InsufficientEvidenceError(
                f"object range requires at least {self._min_supporting_points} supporting "
                f"points, got {len(supporting_points)}"
            )

        ranges = np.linalg.norm(supporting_points, axis=1)
        quartiles = np.quantile(ranges, (0.25, 0.5, 0.75))
        bbox = detection.bbox
        mask_bbox = segmented_detection.bbox
        evidence = ObjectRangeEvidence(
            object_name=normalized_name,
            camera_range_m=float(quartiles[1]),
            supporting_point_count=len(supporting_points),
            prompt_bbox_xyxy=(float(bbox[0]), float(bbox[1]), float(bbox[2]), float(bbox[3])),
            mask_bbox_xyxy=(
                float(mask_bbox[0]),
                float(mask_bbox[1]),
                float(mask_bbox[2]),
                float(mask_bbox[3]),
            ),
            mask_area_px=int(np.count_nonzero(mask)),
            range_quartiles_m=(
                float(quartiles[0]),
                float(quartiles[1]),
                float(quartiles[2]),
            ),
            synchronization_delta_s=float(frame.synchronization_delta_s),
            calibration_source=frame.calibration_source,
        )
        self._cached_evidence[cache_key] = evidence
        return evidence

    def _get_segmenter(self) -> EdgeTAMImageSegmenterCompatible:
        if self._segmenter is None:
            from dimos.models.segmentation.edge_tam import EdgeTAMImageSegmenter

            self._segmenter = EdgeTAMImageSegmenter()
        return self._segmenter


class ObjectRangeEstimator(Protocol):
    """Estimate object range from an explicit canonical frame."""

    def estimate(self, frame: CalibratedFrame, object_name: str) -> ObjectRangeEvidence: ...
