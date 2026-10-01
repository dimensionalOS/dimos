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

"""File, color, CDR and reactive selection checks on generated images."""

from pathlib import Path

from dimos_generated.sensor_msgs.msg import Image
from dimos_generated.std_msgs.msg import Header
import numpy as np
from PIL import Image as PILImage
import pytest
from reactivex.testing import ReactiveTest, TestScheduler

from dimos.msgs.image import (
    image_from_array,
    image_from_file,
    image_sharpness,
    image_to_bgr,
    image_to_rgb,
    image_view,
)
from dimos.msgs.time import time_from_nanoseconds, to_nanoseconds
from dimos.utils.reactive import quality_barrier


@pytest.fixture
def img(tmp_path: Path) -> Image:
    pixels = np.zeros((771, 1024, 3), dtype=np.uint8)
    pixels[..., 0] = 201
    pixels[..., 1] = 37
    path = tmp_path / "camera.png"
    PILImage.fromarray(pixels).save(path)
    return image_from_file(path, header=Header(stamp=time_from_nanoseconds(1234567890123456789)))


def test_file_load(img: Image) -> None:
    pixels = image_view(img)
    assert isinstance(pixels, np.ndarray)
    assert (img.width, img.height) == (1024, 771)
    assert pixels.shape == (771, 1024, 3)
    assert pixels.dtype == np.uint8
    assert img.encoding == "rgb8"
    assert img.header.frame_id == ""
    assert to_nanoseconds(img.header.stamp) == 1234567890123456789
    assert pixels.flags["C_CONTIGUOUS"]
    np.testing.assert_array_equal(pixels[0, 0], [201, 37, 0])


def test_cdr_encode_decode(img: Image) -> None:
    decoded = Image.decode(img.encode())
    assert decoded is not img
    assert decoded == img


def test_rgb_bgr_conversion(img: Image) -> None:
    bgr = image_from_array(image_to_bgr(img), encoding="bgr8", header=img.header)
    assert bgr != img
    restored = image_from_array(image_to_rgb(bgr), encoding="rgb8", header=img.header)
    assert restored == img


def test_opencv_conversion(img: Image) -> None:
    pixels = image_to_bgr(img)
    generated = image_from_array(pixels, encoding="bgr8", header=img.header)
    np.testing.assert_array_equal(image_to_rgb(generated), image_view(img))
    assert generated.header == img.header


def test_sharpness_barrier() -> None:
    # Real generated images: the first and fifth win the same two virtual windows.
    pattern = (np.indices((8, 8)).sum(axis=0) % 2).astype(np.uint8)
    images = [
        image_from_array(pattern * level, encoding="mono8") for level in (255, 200, 150, 50, 230)
    ]
    assert image_sharpness(images[0]) > max(image_sharpness(value) for value in images[1:4])
    scheduler = TestScheduler()
    source = scheduler.create_hot_observable(
        ReactiveTest.on_next(200.01, images[0]),
        ReactiveTest.on_next(200.02, images[1]),
        ReactiveTest.on_next(200.03, images[2]),
        ReactiveTest.on_next(200.04, images[3]),
        ReactiveTest.on_next(200.06, images[4]),
        ReactiveTest.on_completed(200.08),
    )
    results = scheduler.start(lambda: source.pipe(quality_barrier(image_sharpness, 20, scheduler)))
    emitted = [message.value.value for message in results.messages if message.value.kind == "N"]
    assert len(emitted) == 2
    assert emitted[0] is images[0]
    assert emitted[1] is images[4]
