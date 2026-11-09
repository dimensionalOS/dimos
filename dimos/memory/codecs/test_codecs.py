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

"""Grid tests for Codec implementations.

Runs roundtrip encode→decode tests across every codec, verifying data preservation.
"""

import numpy as np
import pytest

from dimos.memory.codecs.base import codec_for, codec_from_id, codec_id
from dimos.memory.codecs.pickle import PickleCodec
from dimos.message_utils import image
from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages.sensor_msgs.msg import Image
from dimos.messages.std_msgs.msg import Header


@pytest.mark.parametrize("encoding,dtype", [("rgb8", np.uint8), ("mono16", np.uint16), ("16UC1", np.uint16), ("32FC1", np.float32)])
def test_generated_image_storage_preserves_pixels_and_exact_header(encoding,dtype):
    shape=(4,6,3) if encoding=="rgb8" else (4,6)
    pixels=np.arange(np.prod(shape),dtype=dtype).reshape(shape)
    source=image.from_array(pixels,encoding=encoding,header=Header(Time(17,123456789),"camera"))
    codec=codec_for(Image,sample=source)
    assert codec_id(codec)=="cdr"
    decoded=codec.decode(codec.encode(source))
    assert decoded.header==source.header
    assert decoded.encoding==encoding
    np.testing.assert_array_equal(image.view(decoded),pixels)


@pytest.mark.parametrize("value", [42, "hello", {"key":[1,2,3]}])
def test_arbitrary_python_storage_remains_pickle(value):
    codec=codec_for(type(value))
    assert isinstance(codec,PickleCodec)
    assert codec.decode(codec.encode(value))==value


def test_compressed_cdr_storage_chain_preserves_value():
    codec=codec_from_id("lz4+cdr", "dimos.messages.sensor_msgs.msg.Image")
    source=image.from_array(np.zeros((4,6,3),dtype=np.uint8),encoding="rgb8")
    decoded=codec.decode(codec.encode(source))
    np.testing.assert_array_equal(image.view(decoded),image.view(source))
