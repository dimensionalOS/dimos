# Copyright 2025 Dimensional Inc.
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

import numpy as np
import pytest

from dimos.msgs.sensor_msgs.Image import Image, ImageFormat, sharpness_window
from dimos.utils.data import get_data
from dimos.utils.testing import TimedSensorReplay


@pytest.fixture
def img():
    image_file_path = get_data("cafe.jpg")
    return Image.from_file(str(image_file_path))


def test_file_load(img: Image):
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


def test_lcm_encode_decode(img: Image):
    binary_msg = img.lcm_encode()
    decoded_img = Image.lcm_decode(binary_msg)

    assert isinstance(decoded_img, Image)
    assert decoded_img is not img
    assert decoded_img == img


def test_rgb_bgr_conversion(img: Image):
    rgb = img.to_rgb()
    assert not rgb == img
    assert rgb.to_bgr() == img


def test_opencv_conversion(img: Image):
    ocv = img.to_opencv()
    decoded_img = Image.from_opencv(ocv)

    # artificially patch timestamp
    decoded_img.ts = img.ts
    assert decoded_img == img


def test_sharpness_detector():
    get_data("unitree_office_walk")  # Preload data for testing
    video_store = TimedSensorReplay(
        "unitree_office_walk/video", autocast=lambda x: Image.from_numpy(x).to_rgb()
    )

    for image in video_store.iterate():
        print(image.sharpness())


def test_sharpness_sliding_window():
    get_data("unitree_office_walk")  # Preload data for testing
    sharpness_window(
        0.5,
        video_store=TimedSensorReplay(
            "unitree_office_walk/video", autocast=lambda x: Image.from_numpy(x).to_rgb()
        ).stream(),
    ).subscribe(print)
