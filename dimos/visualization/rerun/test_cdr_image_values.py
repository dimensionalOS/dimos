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

import imagecodecs
import numpy as np
import pytest
import rerun as rr

from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages.sensor_msgs.msg import CompressedImage
from dimos.messages.std_msgs.msg import Header
from dimos.visualization.rerun.cdr import image_archetype


def test_png_color_is_rendered_as_encoded_image():
    pixels = np.array([[[255, 0, 0], [0, 255, 0]]], dtype=np.uint8)
    message = CompressedImage(
        header=Header(stamp=Time(sec=0, nanosec=0), frame_id="camera"),
        format="png",
        data=np.frombuffer(imagecodecs.png_encode(pixels), dtype=np.uint8),
    )
    archetype = image_archetype(message)
    assert isinstance(archetype, rr.EncodedImage)
    assert bytes(archetype.blob.as_arrow_array().to_pylist()[0]) == bytes(message.data)


@pytest.mark.parametrize("codec", ["png", "jxl"])
def test_compressed_uint16_depth_keeps_values_and_units(codec):
    pixels = np.arange(80, dtype=np.uint16).reshape(10, 8)
    encoded = (
        imagecodecs.png_encode(pixels)
        if codec == "png"
        else imagecodecs.jpegxl_encode(pixels, lossless=True)
    )
    message = CompressedImage(
        header=Header(frame_id="depth", stamp=Time(sec=0, nanosec=0)),
        format=f"16UC1; {codec}",
        data=np.frombuffer(encoded, dtype=np.uint8),
    )
    archetype = image_archetype(message)
    assert isinstance(archetype, rr.DepthImage)
    assert archetype.meter.as_arrow_array().to_pylist() == [1000.0]
    np.testing.assert_array_equal(
        np.frombuffer(bytes(archetype.buffer.as_arrow_array().to_pylist()[0]), dtype=np.uint16),
        pixels.ravel(),
    )
