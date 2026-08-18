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

"""LiDAR range evidence derived from cached object masks."""

from __future__ import annotations

from typing import TYPE_CHECKING, Annotated

import numpy as np
from pydantic import BaseModel, ConfigDict, Field, StringConstraints

from dimos.evals.vqa.contracts import InsufficientEvidenceError, ObjectMaskEstimator
from dimos.perception.detection.type.detection3d.pointcloud import ProjectedPointCloud

if TYPE_CHECKING:
    from dimos.evals.vqa.preprocessing import CalibratedFrame

ObjectName = Annotated[str, StringConstraints(strip_whitespace=True, min_length=1)]


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


class LidarRangeEstimator:
    """Add calibrated LiDAR range statistics to object-mask evidence."""

    def __init__(self, masks: ObjectMaskEstimator, min_supporting_points: int = 5) -> None:
        if min_supporting_points < 1:
            raise ValueError("min_supporting_points must be at least 1")
        self._masks = masks
        self._min_supporting_points = min_supporting_points
        self._cached_frame: CalibratedFrame | None = None
        self._cached_projection: ProjectedPointCloud | None = None
        self._cached_evidence: dict[str, ObjectRangeEvidence] = {}

    def estimate(self, frame: CalibratedFrame, object_name: str) -> ObjectRangeEvidence:
        """Estimate robust camera-origin range to one detected object."""
        return self.estimate_many(frame, (object_name,))[0]

    def estimate_many(
        self, frame: CalibratedFrame, object_names: tuple[str, ...]
    ) -> tuple[ObjectRangeEvidence, ...]:
        """Estimate several ranges with cached masks and one projected cloud."""
        masks = self._masks.estimate_many(frame.image, object_names)
        if frame is not self._cached_frame:
            self._cached_frame = frame
            self._cached_projection = None
            self._cached_evidence.clear()
        if (frame.camera_info.height, frame.camera_info.width) != (
            frame.image.height,
            frame.image.width,
        ):
            raise ValueError("camera calibration dimensions do not match the rectified image")

        pending = [mask for mask in masks if mask.object_name not in self._cached_evidence]
        if pending:
            projection = self._cached_projection
            if projection is None:
                projection = ProjectedPointCloud.from_pointcloud(
                    frame.pointcloud,
                    frame.camera_info,
                    frame.pointcloud_to_camera,
                )
                self._cached_projection = projection
            for mask in pending:
                _, points = projection.points_in_detection(
                    mask.detection,
                    nearest_per_pixel=True,
                )
                if len(points) < self._min_supporting_points:
                    raise InsufficientEvidenceError(
                        f"object range requires at least {self._min_supporting_points} supporting "
                        f"points for {mask.object_name!r}, got {len(points)}"
                    )
                quartiles = np.quantile(np.linalg.norm(points, axis=1), (0.25, 0.5, 0.75))
                self._cached_evidence[mask.object_name] = ObjectRangeEvidence(
                    object_name=mask.object_name,
                    camera_range_m=float(quartiles[1]),
                    supporting_point_count=len(points),
                    prompt_bbox_xyxy=mask.prompt_bbox_xyxy,
                    mask_bbox_xyxy=mask.mask_bbox_xyxy,
                    mask_area_px=mask.mask_area_px,
                    range_quartiles_m=(
                        float(quartiles[0]),
                        float(quartiles[1]),
                        float(quartiles[2]),
                    ),
                    synchronization_delta_s=float(frame.synchronization_delta_s),
                    calibration_source=frame.calibration_source,
                )
        return tuple(self._cached_evidence[mask.object_name] for mask in masks)
