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

import pytest
import rerun as rr

from dimos.msgs.foxglove_msgs.CompressedVideo import CompressedVideo
from dimos.msgs.helpers import resolve_msg_type

PACKET = b"\x00\x00\x00\x01\x65payload"


def test_lcm_encode_decode() -> None:
    original = CompressedVideo(data=PACKET, format="h264", frame_id="front_camera", ts=1234567890.5)
    decoded = CompressedVideo.lcm_decode(original.lcm_encode())

    assert decoded.data.tobytes() == PACKET
    assert decoded.format == "h264"
    assert decoded.frame_id == "front_camera"
    assert abs(decoded.ts - original.ts) < 1e-6


def test_resolves_over_the_raw_lcm_type() -> None:
    """The rerun bridge decodes by type name — it must land on the to_rerun() class."""
    resolved = resolve_msg_type("foxglove_msgs.CompressedVideo")
    assert resolved is CompressedVideo


def test_to_rerun_video_stream() -> None:
    stream = CompressedVideo(data=PACKET, format="h264").to_rerun()
    assert isinstance(stream, rr.VideoStream)
    assert bytes(stream.sample.as_arrow_array().to_pylist()[0]) == PACKET  # type: ignore[union-attr]


def test_to_rerun_unknown_codec() -> None:
    with pytest.raises(ValueError, match="mjpeg"):
        CompressedVideo(data=PACKET, format="mjpeg").to_rerun()
