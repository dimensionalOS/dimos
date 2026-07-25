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

from dimos.protocol.pubsub.impl.webrtc.mvp_metrics import (
    FrameStamp,
    decode_frame_stamp,
    make_stamped_frame,
    percentile,
)


def test_frame_stamp_survives_small_pixel_error() -> None:
    frame = make_stamped_frame(
        width=640,
        height=360,
        sequence=42,
        timestamp_ms=1_900_000_000_123,
    )
    noise = np.random.default_rng(7).integers(-12, 13, frame.shape, dtype=np.int16)
    noisy = np.clip(frame.astype(np.int16) + noise, 0, 255).astype(np.uint8)

    assert decode_frame_stamp(noisy) == FrameStamp(
        timestamp_ms=1_900_000_000_123,
        sequence=42,
    )


def test_frame_stamp_rejects_corruption() -> None:
    frame = make_stamped_frame(width=640, height=360, sequence=2, timestamp_ms=1000)
    frame[:48, :6] = 255 - frame[:48, :6]

    assert decode_frame_stamp(frame) is None


def test_percentile_empty_and_median() -> None:
    assert percentile([], 0.5) is None
    assert percentile([10.0, 20.0, 30.0], 0.5) == 20.0
