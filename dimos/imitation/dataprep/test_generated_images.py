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

# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.sensor_msgs.msg import Image
from dimos_generated.std_msgs.msg import Header
from dimos_message_build.registry import decode as cdr_decode, encode as cdr_encode
import numpy as np
import pytest

from dimos.imitation.dataprep.core import StreamField, is_image_array, resolve_field
from dimos.msgs.image import compressed_image_from_image, image_from_array


@pytest.mark.parametrize("compressed", [False, True])
def test_generated_camera_field_is_a_pixel_array_after_cdr(compressed):
    pixels = np.full((8, 10, 3), [30, 90, 150], dtype=np.uint8)
    image = image_from_array(
        pixels, encoding="bgr8", header=Header(frame_id="camera", stamp=Time(sec=0, nanosec=0))
    )
    message = compressed_image_from_image(image, quality=100) if compressed else image
    message = cdr_decode(cdr_encode(message), type(message))
    before = cdr_encode(message)
    array = resolve_field(message, StreamField(stream="camera"))
    assert array.shape == pixels.shape and array.dtype == pixels.dtype
    assert is_image_array(array)
    np.testing.assert_allclose(array, pixels, atol=2 if compressed else 0)
    assert cdr_encode(message) == before


def test_generated_mono16_camera_respects_row_padding_and_endian():
    pixels = np.array([[1, 65535], [256, 4096]], dtype=">u2")
    padded = np.zeros((2, 6), dtype=np.uint8)
    padded[:, :4] = pixels.view(np.uint8).reshape(2, 4)
    message = Image(
        width=2,
        height=2,
        encoding="mono16",
        is_bigendian=1,
        step=6,
        data=np.asarray(padded.ravel(), dtype=np.uint8),
        header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
    )
    array = resolve_field({"camera": message}, StreamField(stream="dict", field="camera"))
    assert is_image_array(array)
    np.testing.assert_array_equal(array, pixels)
    assert array.strides == (6, 2)
