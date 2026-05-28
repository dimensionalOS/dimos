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

"""Tests for the GPU-free red-object detector (the demo's reliable find path)."""

import numpy as np
import pytest

from dimos.experimental.pack_mind.red_detector import DEFAULT_RED_THRESHOLD, red_fraction


def _frame(rgb: tuple[int, int, int], h: int = 48, w: int = 64) -> np.ndarray:
    img = np.zeros((h, w, 3), dtype=np.uint8)
    img[:] = rgb
    return img


@pytest.mark.unit
def test_all_red_frame_is_fully_red() -> None:
    assert red_fraction(_frame((220, 20, 20))) == pytest.approx(1.0)


@pytest.mark.unit
def test_blue_and_green_and_gray_are_not_red() -> None:
    for rgb in [(20, 20, 220), (20, 220, 20), (128, 128, 128), (0, 0, 0), (255, 255, 255)]:
        assert red_fraction(_frame(rgb)) == 0.0


@pytest.mark.unit
def test_partial_red_patch_crosses_threshold() -> None:
    img = _frame((130, 130, 130))  # neutral background
    img[:, :20] = (230, 30, 30)  # a red object filling ~31% of the width
    frac = red_fraction(img)
    assert frac > DEFAULT_RED_THRESHOLD
    assert frac == pytest.approx(20 / 64, abs=0.01)


@pytest.mark.unit
def test_tiny_red_speck_below_threshold() -> None:
    img = _frame((130, 130, 130))
    img[0, 0] = (230, 30, 30)  # a single red pixel
    assert red_fraction(img) < DEFAULT_RED_THRESHOLD


@pytest.mark.unit
def test_grayscale_or_bad_shape_is_safe() -> None:
    assert red_fraction(np.zeros((10, 10), dtype=np.uint8)) == 0.0
