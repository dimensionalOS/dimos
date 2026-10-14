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

from __future__ import annotations

import numpy as np
import pytest

from dimos.evals.vqa.contracts import InsufficientEvidenceError
from dimos.evals.vqa.primitives.color import (
    dominant_color_in_mask,
    object_name_mentions_color,
)
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat


def _rgb_image(rgb: tuple[int, int, int], size: int = 40) -> Image:
    return Image.from_numpy(
        np.full((size, size, 3), rgb, dtype=np.uint8),
        format=ImageFormat.RGB,
    )


@pytest.mark.parametrize(
    ("name", "expected"),
    (
        ("chair", False),
        ("red bag", True),
        ("black couch", True),
        ("redhead", False),
        ("Gray Table", True),
    ),
)
def test_object_name_mentions_color(name: str, expected: bool) -> None:
    assert object_name_mentions_color(name) is expected


@pytest.mark.parametrize(
    ("rgb", "expected"),
    (
        ((0, 0, 255), "blue"),
        ((250, 250, 250), "white"),
        ((5, 5, 5), "black"),
        ((128, 128, 128), "gray"),
        ((128, 64, 0), "brown"),
        ((255, 0, 200), "purple"),  # magenta / pink → purple, not brown
    ),
)
def test_dominant_color_in_mask_labels(rgb: tuple[int, int, int], expected: str) -> None:
    image = _rgb_image(rgb)
    mask = np.ones((40, 40), dtype=bool)

    evidence = dominant_color_in_mask(image, "chair", mask)

    assert evidence.dominant_bucket == expected


def test_dominant_color_uses_mask_not_background() -> None:
    pixels = np.full((40, 40, 3), (0, 0, 255), dtype=np.uint8)  # blue background
    pixels[10:30, 10:30] = (255, 0, 0)  # red object
    image = Image.from_numpy(pixels, format=ImageFormat.RGB)
    mask = np.zeros((40, 40), dtype=bool)
    mask[10:30, 10:30] = True

    evidence = dominant_color_in_mask(image, "bag", mask)

    assert evidence.dominant_bucket == "red"


def test_dominant_color_rejects_ties() -> None:
    pixels = np.zeros((40, 40, 3), dtype=np.uint8)
    pixels[:, :20] = (255, 0, 0)
    pixels[:, 20:] = (0, 0, 255)
    image = Image.from_numpy(pixels, format=ImageFormat.RGB)

    with pytest.raises(InsufficientEvidenceError, match="unambiguous dominant"):
        dominant_color_in_mask(image, "chair", np.ones((40, 40), dtype=bool))
