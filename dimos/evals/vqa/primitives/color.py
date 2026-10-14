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

"""Dominant-color evidence from a segmentation mask."""

from __future__ import annotations

import re
from typing import TYPE_CHECKING

import numpy as np
from pydantic import BaseModel, ConfigDict, Field

from dimos.evals.vqa.contracts import InsufficientEvidenceError, NonEmptyString

if TYPE_CHECKING:
    from dimos.msgs.sensor_msgs.Image import Image

COLOR_CHOICES = (
    "red",
    "orange",
    "yellow",
    "green",
    "blue",
    "purple",
    "brown",
    "white",
    "black",
    "gray",
)

_COLOR_NAME_PATTERN = re.compile(
    r"(?<![a-z0-9])(" + "|".join(re.escape(name) for name in COLOR_CHOICES) + r")(?![a-z0-9])",
    re.IGNORECASE,
)

_MIN_MASK_PIXELS = 32
_MIN_BUCKET_FRACTION = 0.4

_BLACK = COLOR_CHOICES.index("black")
_WHITE = COLOR_CHOICES.index("white")
_GRAY = COLOR_CHOICES.index("gray")
_RED = COLOR_CHOICES.index("red")
_ORANGE = COLOR_CHOICES.index("orange")
_YELLOW = COLOR_CHOICES.index("yellow")
_GREEN = COLOR_CHOICES.index("green")
_BLUE = COLOR_CHOICES.index("blue")
_PURPLE = COLOR_CHOICES.index("purple")
_BROWN = COLOR_CHOICES.index("brown")


class ObjectColorEvidence(BaseModel):
    """Dominant color bucket inside one object mask."""

    model_config = ConfigDict(extra="forbid", frozen=True)

    object_name: NonEmptyString
    dominant_bucket: NonEmptyString
    bucket_fraction: float = Field(ge=0, le=1)
    runner_up_bucket: NonEmptyString
    runner_up_fraction: float = Field(ge=0, le=1)
    mask_pixel_count: int = Field(ge=1)


def object_name_mentions_color(object_name: str) -> bool:
    """True when an object reference already encodes a COLOR_CHOICES token."""
    return _COLOR_NAME_PATTERN.search(object_name) is not None


def dominant_color_in_mask(image: Image, object_name: str, mask: np.ndarray) -> ObjectColorEvidence:
    """Map the largest HSV bucket inside a boolean mask to a fixed color choice."""
    import cv2

    if mask.shape != (image.height, image.width):
        raise InsufficientEvidenceError(
            f"object color mask shape {mask.shape} does not match image "
            f"{(image.height, image.width)}"
        )
    selected = np.asarray(mask, dtype=bool)
    mask_pixel_count = int(selected.sum())
    if mask_pixel_count < _MIN_MASK_PIXELS:
        raise InsufficientEvidenceError(
            f"object color requires at least {_MIN_MASK_PIXELS} mask pixels, got {mask_pixel_count}"
        )

    rgb = np.asarray(image.to_rgb().data[:, :, :3], dtype=np.uint8)
    if rgb.shape[:2] != selected.shape:
        raise InsufficientEvidenceError("object color requires an RGB image matching the mask")
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    hue = hsv[:, :, 0][selected].astype(np.int16)
    sat = hsv[:, :, 1][selected].astype(np.int16)
    val = hsv[:, :, 2][selected].astype(np.int16)

    buckets = _hsv_to_bucket_indices(hue, sat, val)
    counts = np.bincount(buckets, minlength=len(COLOR_CHOICES))
    ranked = np.argsort(counts)
    winner = int(ranked[-1])
    runner_up = int(ranked[-2])
    total = int(counts.sum())
    winner_count = int(counts[winner])
    runner_up_count = int(counts[runner_up])
    fraction = winner_count / total
    runner_up_fraction = runner_up_count / total

    if winner_count <= runner_up_count:
        raise InsufficientEvidenceError(
            "object color mask does not have one unambiguous dominant bucket"
        )
    if fraction < _MIN_BUCKET_FRACTION:
        raise InsufficientEvidenceError(
            "object color mask does not have one unambiguous dominant bucket"
        )

    return ObjectColorEvidence(
        object_name=object_name,
        dominant_bucket=COLOR_CHOICES[winner],
        bucket_fraction=fraction,
        runner_up_bucket=COLOR_CHOICES[runner_up],
        runner_up_fraction=runner_up_fraction,
        mask_pixel_count=mask_pixel_count,
    )


def _hsv_to_bucket_indices(
    hue: np.ndarray, saturation: np.ndarray, value: np.ndarray
) -> np.ndarray:
    """Vectorized OpenCV-HSV (H in 0..179) → COLOR_CHOICES index."""
    # Neutrals first so black/white/gray are counted rather than discarded.
    # Brown uses the orange hue band with lower brightness; magenta/pink maps to purple.
    return np.select(
        [
            value < 40,
            (value > 220) & (saturation < 35),
            saturation < 40,
            ((hue < 10) | (hue >= 170)),
            (hue < 25) & (value < 140),
            hue < 25,
            hue < 35,
            hue < 85,
            hue < 130,
            hue < 150,
            (hue < 170),
        ],
        [
            _BLACK,
            _WHITE,
            _GRAY,
            _RED,
            _BROWN,
            _ORANGE,
            _YELLOW,
            _GREEN,
            _BLUE,
            _PURPLE,
            _PURPLE,  # magenta / pink
        ],
        default=_BROWN,
    ).astype(np.intp)
