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
from reactivex import operators as ops

from dimos.msgs.sensor_msgs.Image import Image, ImageFormat, sharpness_barrier
from dimos.utils.data import get_data
from dimos.utils.testing import TimedSensorReplay


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
    video_store = TimedSensorReplay(
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

    # Use 20Hz frequency (0.05s windows) for faster test
    # Emit images at 100Hz to get ~5 per window
    from reactivex import from_iterable, interval

    source = from_iterable(mock_images).pipe(
        ops.zip(interval(0.01)),  # 100Hz emission rate
        ops.map(lambda x: x[0]),  # Extract just the image
    )

    source.pipe(
        ops.do_action(track_input),  # Track inputs
        sharpness_barrier(20),  # 20Hz = 0.05s windows
        ops.do_action(track_output),  # Track outputs
    ).run()

    # Only need 0.08s for 1 full window at 20Hz plus buffer
    time.sleep(0.08)

    # Verify we got correct emissions (items span across 2 windows due to timing)
    # Items 1-4 arrive in first window (0-50ms), item 5 arrives in second window (50-100ms)
    assert len(emitted_images) == 2, (
        f"Expected exactly 2 emissions (one per window), got {len(emitted_images)}"
    )

    # Group inputs by wall-clock windows and verify we got the sharpest

    # Verify each window emitted the sharpest image from that window
    # First window (0-50ms): items 1-4
    assert emitted_images[0].sharpness == 0.3711  # Highest among first 4 items

    # Second window (50-100ms): only item 5
    assert emitted_images[1].sharpness == 0.3665  # Only item in second window
