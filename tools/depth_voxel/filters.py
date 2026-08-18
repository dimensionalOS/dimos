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

"""Depth-image denoising filters applied before unprojection.

Every filter takes a raw DEPTH16 ``uint16`` array and returns one of the same
shape, with rejected pixels zeroed. Zero already means "no measurement" in the
DEPTH16 convention, so the unprojector drops them for free.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy import ndimage

MILLIMETERS_PER_METER = 1000.0


@dataclass(frozen=True)
class DepthFilter:
    """Parameterised depth cleanup. Every stage is off by default."""

    name: str
    min_range_m: float = 0.0
    max_range_m: float = np.inf
    median_kernel: int = 0
    """Square median-filter window in pixels; 0 disables."""
    speckle_max_deviation_m: float = 0.0
    """Reject a pixel whose depth differs from its 3x3 median by more than this; 0 disables."""
    gradient_max_step_m: float = 0.0
    """Reject pixels on a depth discontinuity steeper than this; 0 disables."""
    min_component_pixels: int = 0
    """Drop connected components of valid pixels smaller than this; 0 disables."""
    erode_border_pixels: int = 0
    """Erode the valid mask by this many pixels to shave edge-bleed halos; 0 disables."""
    temporal_max_deviation_m: float = 0.0
    """Reject a pixel deviating from the running per-pixel median by more than this; 0 disables."""
    temporal_window: int = 3
    subsample: int = 1
    """Keep every Nth pixel in each axis; 1 disables."""

    history: list = field(default_factory=list, compare=False, repr=False)

    def describe(self) -> str:
        parts = []
        if self.min_range_m > 0:
            parts.append(f"min={self.min_range_m}m")
        if np.isfinite(self.max_range_m):
            parts.append(f"max={self.max_range_m}m")
        if self.median_kernel:
            parts.append(f"median={self.median_kernel}px")
        if self.speckle_max_deviation_m:
            parts.append(f"speckle={self.speckle_max_deviation_m}m")
        if self.gradient_max_step_m:
            parts.append(f"grad={self.gradient_max_step_m}m")
        if self.min_component_pixels:
            parts.append(f"blob>={self.min_component_pixels}px")
        if self.erode_border_pixels:
            parts.append(f"erode={self.erode_border_pixels}px")
        if self.temporal_max_deviation_m:
            parts.append(f"temporal={self.temporal_max_deviation_m}m/{self.temporal_window}")
        if self.subsample > 1:
            parts.append(f"subsample={self.subsample}")
        return ", ".join(parts) or "none"

    def reset(self) -> None:
        self.history.clear()

    def __call__(self, depth: np.ndarray) -> np.ndarray:
        out = depth.copy()
        valid = out > 0

        if self.min_range_m > 0:
            valid &= out >= self.min_range_m * MILLIMETERS_PER_METER
        if np.isfinite(self.max_range_m):
            valid &= out <= self.max_range_m * MILLIMETERS_PER_METER

        working = np.where(valid, out, 0).astype(np.float32)

        if self.median_kernel > 1:
            smoothed = ndimage.median_filter(working, size=self.median_kernel)
            working = np.where(valid, smoothed, 0)

        if self.speckle_max_deviation_m > 0:
            local_median = ndimage.median_filter(working, size=3)
            deviation = np.abs(working - local_median)
            valid &= deviation <= self.speckle_max_deviation_m * MILLIMETERS_PER_METER

        if self.gradient_max_step_m > 0:
            limit = self.gradient_max_step_m * MILLIMETERS_PER_METER
            dilated = ndimage.maximum_filter(working, size=3)
            eroded = ndimage.minimum_filter(np.where(valid, working, np.inf), size=3)
            valid &= (dilated - eroded) <= limit

        if self.temporal_max_deviation_m > 0:
            self.history.append(working.copy())
            del self.history[: -self.temporal_window]
            if len(self.history) == self.temporal_window:
                stack_median = np.median(np.stack(self.history), axis=0)
                deviation = np.abs(working - stack_median)
                valid &= deviation <= self.temporal_max_deviation_m * MILLIMETERS_PER_METER

        if self.min_component_pixels > 0:
            labels, count = ndimage.label(valid)
            if count:
                sizes = np.bincount(labels.ravel())
                sizes[0] = 0
                valid &= (sizes >= self.min_component_pixels)[labels]

        if self.erode_border_pixels > 0:
            valid = ndimage.binary_erosion(
                valid, iterations=self.erode_border_pixels, border_value=0
            )

        if self.subsample > 1:
            keep = np.zeros_like(valid)
            keep[:: self.subsample, :: self.subsample] = True
            valid &= keep

        return np.where(valid, np.round(working).astype(np.uint16), 0)


UNFILTERED = DepthFilter(name="baseline-unfiltered")
