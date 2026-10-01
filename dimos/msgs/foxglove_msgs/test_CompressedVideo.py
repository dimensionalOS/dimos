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

from dimos_generated.foxglove_msgs.msg import CompressedVideo
import pytest
import rerun as rr

from dimos.msgs.helpers import resolve_msg_type
from dimos.msgs.time import time_from_nanoseconds, to_nanoseconds
from dimos.visualization.rerun.message_helpers import video_archetype

PACKET = b"\x00\x00\x00\x01\x65payload"


def test_cdr_encode_decode() -> None:
    original = CompressedVideo(
        data=PACKET,
        format="h264",
        frame_id="front_camera",
        timestamp=time_from_nanoseconds(1234567890123456789),
    )
    decoded = CompressedVideo.decode(original.encode())

    assert bytes(decoded.data) == PACKET
    assert decoded.format == "h264"
    assert decoded.frame_id == "front_camera"
    assert to_nanoseconds(decoded.timestamp) == 1234567890123456789


def test_resolves_generated_canonical_type() -> None:
    """Typed transports discover generated schema-bearing values."""
    resolved = resolve_msg_type("foxglove_msgs/msg/CompressedVideo")
    assert resolved is CompressedVideo


def test_to_rerun_video_stream() -> None:
    stream = video_archetype(CompressedVideo(data=PACKET, format="h264"))
    assert isinstance(stream, rr.VideoStream)
    assert bytes(stream.sample.as_arrow_array().to_pylist()[0]) == PACKET  # type: ignore[union-attr]


def test_to_rerun_unknown_codec() -> None:
    with pytest.raises(ValueError, match="mjpeg"):
        video_archetype(CompressedVideo(data=PACKET, format="mjpeg"))
