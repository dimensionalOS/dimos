# Copyright 2025-2026 Dimensional Inc.
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

from unittest.mock import MagicMock

import numpy as np
import pytest
from reactivex.testing import ReactiveTest, TestScheduler

from dimos.msgs.sensor_msgs.Image import Image, ImageFormat, sharpness_barrier
from dimos.utils.data import get_data


@pytest.fixture
def img():
    image_file_path = get_data("cafe.jpg")
    return Image.from_file(str(image_file_path))


def test_file_load(img: Image) -> None:
    assert isinstance(img.data, np.ndarray)
    assert img.width == 1024
    assert img.height == 771
    assert img.channels == 3
    assert img.shape == (771, 1024, 3)
    assert img.data.dtype == np.uint8
    assert img.format == ImageFormat.BGR
    assert img.frame_id == ""
    assert isinstance(img.ts, float)
    assert img.ts > 0
    assert img.data.flags["C_CONTIGUOUS"]


def test_lcm_encode_decode(img: Image) -> None:
    binary_msg = img.lcm_encode()
    decoded_img = Image.lcm_decode(binary_msg)

    assert isinstance(decoded_img, Image)
    assert decoded_img is not img
    assert decoded_img == img


def test_rgb_bgr_conversion(img: Image) -> None:
    rgb = img.to_rgb()
    assert not rgb == img
    assert rgb.to_bgr() == img


def test_opencv_conversion(img: Image) -> None:
    ocv = img.to_opencv()
    decoded_img = Image.from_opencv(ocv)

    # artificially patch timestamp
    decoded_img.ts = img.ts
    assert decoded_img == img


def test_sharpness_barrier() -> None:
    # Mock images with known sharpness values, avoiding real data from disk
    sharpness_values = [0.3711, 0.3241, 0.3067, 0.2583, 0.3665]
    mock_images = []
    for sharp in sharpness_values:
        img = MagicMock()
        img.sharpness = sharp
        mock_images.append(img)

    # sharpness_barrier(20) -> 0.05s windows in virtual time. Subscription is at
    # t=200, so windows close at 200.05, 200.10, ... Items 1-4 land in the first
    # window, item 5 in the second.
    scheduler = TestScheduler()
    source = scheduler.create_hot_observable(
        ReactiveTest.on_next(200.01, mock_images[0]),
        ReactiveTest.on_next(200.02, mock_images[1]),
        ReactiveTest.on_next(200.03, mock_images[2]),
        ReactiveTest.on_next(200.04, mock_images[3]),
        ReactiveTest.on_next(200.06, mock_images[4]),
        ReactiveTest.on_completed(200.08),
    )

    results = scheduler.start(lambda: source.pipe(sharpness_barrier(20, scheduler=scheduler)))

    emitted = [m.value.value for m in results.messages if m.value.kind == "N"]
    assert len(emitted) == 2, f"Expected one emission per window, got {len(emitted)}"
    assert emitted[0].sharpness == 0.3711  # Sharpest of the 4 in the first window
    assert emitted[1].sharpness == 0.3665  # Only item in the second window
