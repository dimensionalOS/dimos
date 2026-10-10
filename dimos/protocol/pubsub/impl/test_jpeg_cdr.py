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

import pickle
from types import SimpleNamespace

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.sensor_msgs.msg import CompressedImage, Image
from dimos_generated.std_msgs.msg import Header
from dimos_message_build.registry import decode as cdr_decode, encode as cdr_encode
import numpy as np
import pytest

from dimos.core.transport import JpegLcmTransport
from dimos.msgs.image import image_from_array, image_from_compressed, image_view
from dimos.protocol.pubsub.encoders import DecodingError
from dimos.protocol.pubsub.impl.jpeg_lcm import JpegEncoderMixin
from dimos.protocol.pubsub.impl.jpeg_shm import JpegSharedMemoryEncoderMixin


@pytest.mark.parametrize("codec", [JpegEncoderMixin(), JpegSharedMemoryEncoderMixin(quality=90)])
def test_jpeg_wire_is_standard_cdr_and_preserves_exact_image_header(codec):
    header = Header(frame_id="optical", stamp=Time(sec=-1, nanosec=987654321))
    image = image_from_array(
        np.full((16, 16, 3), [20, 80, 140], dtype=np.uint8), encoding="rgb8", header=header
    )
    topic = SimpleNamespace(topic="/image", msg_type=CompressedImage)
    before = cdr_encode(image)
    encoded = codec.encode(image, topic)
    wire = cdr_decode(encoded, CompressedImage)
    assert encoded[:4] == b"\x00\x01\x00\x00"
    assert wire.header == header
    assert wire.format == "rgb8; jpeg compressed bgr8"
    decoded = codec.decode(encoded, topic)
    assert decoded.encoding == "bgr8" and decoded.header == header
    np.testing.assert_allclose(image_view(decoded)[0, 0], [140, 80, 20], atol=3)
    assert cdr_encode(image) == before


def test_jpeg_lcm_requires_truthful_compressed_wire_type():
    codec = JpegEncoderMixin()
    with pytest.raises(DecodingError, match="CompressedImage"):
        codec.decode(b"", SimpleNamespace(topic="/image", msg_type=Image))
    with pytest.raises(DecodingError, match="SELF_TEST"):
        codec.decode(b"", SimpleNamespace(topic="LCM_SELF_TEST", msg_type=CompressedImage))


def test_jpeg_transport_wire_metadata_and_pickle_keep_separate_application_type(monkeypatch):
    monkeypatch.setattr(
        "dimos.protocol.pubsub.impl.jpeg_lcm.JpegLCM", lambda **kwargs: SimpleNamespace()
    )
    transport = JpegLcmTransport("/image", Image)
    assert transport.topic.msg_type is CompressedImage
    restored = pickle.loads(pickle.dumps(transport))
    assert restored.image_type is Image
    assert restored.topic.msg_type is CompressedImage


def test_compressed_image_rejects_invalid_data_and_unknown_format():
    with pytest.raises(ValueError, match="invalid compressed"):
        image_from_compressed(
            CompressedImage(
                format="jpeg",
                data=np.frombuffer(b"not jpeg", dtype=np.uint8),
                header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
            )
        )
    with pytest.raises(ValueError, match="unsupported compressed"):
        image_from_compressed(
            CompressedImage(
                format="h264",
                data=np.frombuffer(b"not jpeg", dtype=np.uint8),
                header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
            )
        )
