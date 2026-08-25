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

from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np
from pydantic import BaseModel, ConfigDict, Field

from dimos.evals.vqa.contracts import InsufficientEvidenceError, NonEmptyString

if TYPE_CHECKING:
    from dimos.evals.vqa.contracts import ObjectMaskEstimator
    from dimos.evals.vqa.pointcloud_frame import PointCloudFrame


@dataclass(frozen=True)
class _FrameProjection:
    """Reusable VQA projection with one camera point per image pixel."""

    camera_points: np.ndarray
    pixels: np.ndarray

    @classmethod
    def from_frame(cls, frame: PointCloudFrame) -> _FrameProjection:
        """Transform and project valid points into image coordinates."""
        camera_info = frame.camera_info
        intrinsics = np.asarray(camera_info.K, dtype=np.float64).reshape(3, 3)

        raw_points, _ = frame.pointcloud.as_numpy()
        points = np.asarray(raw_points, dtype=np.float64)
        points = points[np.all(np.isfinite(points), axis=1)]

        transform = np.asarray(frame.pointcloud_to_camera.to_matrix(), dtype=np.float64)
        homogeneous = np.column_stack((points, np.ones(len(points), dtype=np.float64)))
        camera_points = (transform @ homogeneous.T).T[:, :3]
        in_front = np.all(np.isfinite(camera_points), axis=1) & (camera_points[:, 2] > 0)
        camera_points = camera_points[in_front]

        projected = (intrinsics @ camera_points.T).T
        image_points = projected[:, :2] / projected[:, 2, np.newaxis]
        in_image = (
            np.all(np.isfinite(image_points), axis=1)
            & (image_points[:, 0] >= 0)
            & (image_points[:, 0] < camera_info.width)
            & (image_points[:, 1] >= 0)
            & (image_points[:, 1] < camera_info.height)
        )
        camera_points = camera_points[in_image]
        pixels = image_points[in_image].astype(np.intp)
        if len(pixels):
            linear_pixels = pixels[:, 1] * camera_info.width + pixels[:, 0]
            order = np.lexsort((camera_points[:, 2], linear_pixels))
            sorted_linear = linear_pixels[order]
            first_per_pixel = np.empty(len(order), dtype=bool)
            first_per_pixel[0] = True
            first_per_pixel[1:] = sorted_linear[1:] != sorted_linear[:-1]
            nearest = order[first_per_pixel]
            camera_points = camera_points[nearest]
            pixels = pixels[nearest]
        return cls(camera_points, pixels)

    def camera_points_in_mask(self, mask: np.ndarray) -> np.ndarray:
        """Return camera-frame points inside one segmentation mask."""
        selected = mask[self.pixels[:, 1], self.pixels[:, 0]] > 0
        return np.asarray(self.camera_points[selected])


class ObjectRangeEvidence(BaseModel):
    """Median Euclidean camera-origin range supported by foreground points."""

    model_config = ConfigDict(extra="forbid", frozen=True, allow_inf_nan=False)

    object_name: NonEmptyString
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
        self._cached_projection: tuple[PointCloudFrame, _FrameProjection] | None = None

    def estimate(self, frame: PointCloudFrame, object_name: str) -> ObjectRangeEvidence:
        """Estimate robust camera-origin range to one detected object."""
        return self.estimate_many(frame, (object_name,))[0]

    def estimate_many(
        self, frame: PointCloudFrame, object_names: tuple[str, ...]
    ) -> tuple[ObjectRangeEvidence, ...]:
        """Estimate several ranges with cached masks and one projected cloud."""
        masks = self._masks.estimate_many(frame.image, object_names)
        cached = self._cached_projection
        if cached is None or cached[0] is not frame:
            projection = _FrameProjection.from_frame(frame)
            self._cached_projection = (frame, projection)
        else:
            projection = cached[1]

        evidence = []
        for mask in masks:
            points = projection.camera_points_in_mask(mask.mask)
            if len(points) < self._min_supporting_points:
                raise InsufficientEvidenceError(
                    f"object range requires at least {self._min_supporting_points} supporting "
                    f"points for {mask.object_name!r}, got {len(points)}"
                )
            lower, median, upper = map(
                float,
                np.quantile(np.linalg.norm(points, axis=1), (0.25, 0.5, 0.75)),
            )
            evidence.append(
                ObjectRangeEvidence(
                    object_name=mask.object_name,
                    camera_range_m=median,
                    supporting_point_count=len(points),
                    prompt_bbox_xyxy=mask.prompt_bbox_xyxy,
                    mask_bbox_xyxy=mask.mask_bbox_xyxy,
                    mask_area_px=mask.mask_area_px,
                    range_quartiles_m=(lower, median, upper),
                    synchronization_delta_s=float(frame.synchronization_delta_s),
                    calibration_source=frame.calibration_source,
                )
            )
        return tuple(evidence)
