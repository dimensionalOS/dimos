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

"""Point-cloud selection helpers for private VQA primitives."""

from __future__ import annotations

import numpy as np

from dimos.benchmark.vqa.contracts import CalibratedFrame
from dimos.benchmark.vqa.generation.primitives.projection import project_visible_points


def points_in_mask(frame: CalibratedFrame, mask: np.ndarray) -> np.ndarray:
    """Return nearest visible camera points covered by one foreground mask."""
    if mask.shape != (frame.image.height, frame.image.width):
        raise ValueError("segmentation mask dimensions must match the image")
    projected = project_visible_points(frame)
    return np.asarray(
        [
            point
            for point, (x, y) in zip(projected.camera_points, projected.pixels, strict=True)
            if mask[y, x] > 0
        ],
        dtype=np.float64,
    )
