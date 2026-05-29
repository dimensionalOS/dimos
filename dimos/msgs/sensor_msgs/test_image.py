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

import sys

import numpy as np
import pytest
from reactivex import operators as ops

_IS_MACOS = sys.platform == "darwin"

from dimos.memory.timeseries.legacy import LegacyPickleStore
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


@pytest.mark.tool
def test_sharpness_stream() -> None:
    get_data("unitree_office_walk")  # Preload data for testing
    video_store = LegacyPickleStore(
        "unitree_office_walk/video", autocast=lambda x: Image.from_numpy(x).to_rgb()
    )

    cnt = 0
    for image in video_store.iterate():
        cnt = cnt + 1
        print(image.sharpness)
        if cnt > 30:
            return


def test_sharpness_barrier() -> None:
    import time
    from unittest.mock import MagicMock

    # Create mock images with known sharpness values
    # This avoids loading real data from disk
    mock_images = []
    sharpness_values = [0.3711, 0.3241, 0.3067, 0.2583, 0.3665]  # Just 5 images for 1 window

    for i, sharp in enumerate(sharpness_values):
        img = MagicMock()
        img.sharpness = sharp
        img.ts = 1758912038.208 + i * 0.01  # Simulate timestamps
        mock_images.append(img)

    # Track what goes into windows and what comes out
    start_wall_time = None
    window_contents = []  # List of (wall_time, image)
    emitted_images = []

    def track_input(img):
        """Track all images going into sharpness_barrier with wall-clock time"""
        nonlocal start_wall_time
        wall_time = time.time()
        if start_wall_time is None:
            start_wall_time = wall_time
        relative_time = wall_time - start_wall_time
        window_contents.append((relative_time, img))
        return img

    def track_output(img) -> None:
        """Track what sharpness_barrier emits"""
        emitted_images.append(img)

    # macOS timer coalescing causes jitter at high frequencies, so use
    # wider windows (2Hz) there. Linux handles 20Hz windows fine.
    _freq = 2 if _IS_MACOS else 20
    from reactivex import from_iterable, interval

    source = from_iterable(mock_images).pipe(
        ops.zip(interval(0.01)),  # 100Hz emission rate
        ops.map(lambda x: x[0]),  # Extract just the image
    )

    source.pipe(
        ops.do_action(track_input),  # Track inputs
        sharpness_barrier(_freq),
        ops.do_action(track_output),  # Track outputs
    ).run()

    time.sleep(0.6 if _IS_MACOS else 0.08)

    if _IS_MACOS:
        # At 2Hz all 5 images land in one 500ms window → exactly 1 emission
        assert len(emitted_images) == 1, (
            f"Expected exactly 1 emission (one window), got {len(emitted_images)}"
        )
        assert emitted_images[0].sharpness == 0.3711
    else:
        # Items span 2 windows at 20Hz: items 1-4 in first, item 5 in second
        assert len(emitted_images) == 2, (
            f"Expected exactly 2 emissions (one per window), got {len(emitted_images)}"
        )
        assert emitted_images[0].sharpness == 0.3711  # Highest among first 4
        assert emitted_images[1].sharpness == 0.3665  # Only item in second window
