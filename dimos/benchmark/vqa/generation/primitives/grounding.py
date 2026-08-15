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

"""Foreground-mask point-cloud grounding primitive for one calibrated frame."""

from __future__ import annotations

import numpy as np

from dimos.benchmark.vqa.contracts import CalibratedFrame, GroundedObject, ProjectedPoints
from dimos.perception.detection.type.detection2d.seg import Detection2DSeg


def ground_segmented_object(
    frame: CalibratedFrame,
    projected: ProjectedPoints,
    detection: Detection2DSeg,
    *,
    min_foreground_points: int = 3,
) -> GroundedObject | None:
    """Create object geometry from visible points covered by one foreground mask."""
    if min_foreground_points < 1:
        raise ValueError("min_foreground_points must be positive")
    mask = detection.mask
    if mask.shape != (frame.image.height, frame.image.width):
        raise ValueError("segmentation mask dimensions must match the image")
    support = [
        (point, x)
        for point, (x, y) in zip(projected.camera_points, projected.pixels, strict=True)
        if mask[y, x] > 0
    ]
    if len(support) < min_foreground_points:
        return None

    points = np.asarray([point for point, _ in support], dtype=np.float64)
    median_image_x = float(np.median([x for _, x in support]))
    return GroundedObject(
        id="",
        label=detection.name,
        point_count=len(points),
        range_m=float(np.median(np.linalg.norm(points, axis=1))),
        horizontal_direction=_horizontal_direction(median_image_x, frame.image.width),
        camera_x_m=float(np.median(points[:, 0])),
    )


def _horizontal_direction(x: float, width: int) -> str:
    if x < width / 3:
        return "left"
    if x >= 2 * width / 3:
        return "right"
    return "center"
