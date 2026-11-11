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

from __future__ import annotations

import numpy as np
import pytest

from dimos.msgs.perception_msgs.EmbeddedFrame import EmbeddedFrame
from dimos.msgs.sensor_msgs.Imu import Imu


def test_lcm_round_trip() -> None:
    embedding = np.array([0.25, -0.5, 1.0, 0.125], dtype=np.float32)
    frame = EmbeddedFrame(
        embedding=embedding,
        sharpness=42.5,
        embedder="siglip",
        embed_hz=3.0,
        frame_id="cam",
        ts=1234.5,
    )

    decoded = EmbeddedFrame.lcm_decode(frame.lcm_encode())

    assert decoded.ts == 1234.5
    assert decoded.frame_id == "cam"
    assert decoded.sharpness == 42.5
    assert decoded.embedder == "siglip"
    assert decoded.embed_hz == 3.0
    assert decoded.embedding.dtype == np.float32
    assert np.array_equal(decoded.embedding, embedding)


def test_lcm_decode_rejects_other_message_type() -> None:
    with pytest.raises(ValueError, match="Decode error"):
        EmbeddedFrame.lcm_decode(Imu(ts=1.0).lcm_encode())
