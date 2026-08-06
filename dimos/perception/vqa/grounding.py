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

"""Foreground-mask point-cloud grounding for one calibrated frame."""

from __future__ import annotations

import numpy as np

from dimos.perception.detection.type.detection2d.seg import Detection2DSeg
from dimos.perception.vqa.geometry import project_visible_points
from dimos.perception.vqa.models import CalibratedFrame, GroundedObject, ProjectionConfig


def ground_segmented_objects(
    frame: CalibratedFrame,
    detections: list[Detection2DSeg],
    *,
    min_foreground_points: int = 3,
    projection: ProjectionConfig = ProjectionConfig(),
) -> list[GroundedObject]:
    """Create object geometry from visible points covered by each foreground mask."""
    if min_foreground_points < 1:
        raise ValueError("min_foreground_points must be positive")

    projected = project_visible_points(frame, projection)
    grounded: list[GroundedObject] = []
    for index, detection in enumerate(detections):
        mask = detection.mask
        if mask.shape != (frame.image.height, frame.image.width):
            raise ValueError("segmentation mask dimensions must match the image")
        selected = [
            point
            for point, (x, y) in zip(projected.camera_points, projected.pixels, strict=True)
            if mask[y, x] > 0
        ]
        if len(selected) < min_foreground_points:
            continue

        ranges = np.linalg.norm(np.asarray(selected), axis=1)
        image_x = [x for x, y in projected.pixels if mask[y, x] > 0]
        median_x = float(np.median(image_x))
        direction = _horizontal_direction(median_x, frame.image.width)
        grounded.append(
            GroundedObject(
                id=f"{frame.id}-{detection.name}-{index}",
                label=detection.name,
                point_count=len(selected),
                range_m=float(np.median(ranges)),
                horizontal_direction=direction,
            )
        )
    return grounded


def _horizontal_direction(x: float, width: int) -> str:
    if x < width / 3:
        return "left"
    if x >= 2 * width / 3:
        return "right"
    return "center"
