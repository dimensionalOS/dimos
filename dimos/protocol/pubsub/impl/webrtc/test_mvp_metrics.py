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

import asyncio
from typing import Any, cast

import numpy as np
import pytest

from dimos.protocol.pubsub.impl.webrtc.mvp_cli import (
    _publisher_config,
    _set_local_description_with_ice_timeout,
)
from dimos.protocol.pubsub.impl.webrtc.mvp_metrics import (
    FrameStamp,
    decode_frame_stamp,
    make_stamped_frame,
    percentile,
)
from dimos.protocol.pubsub.impl.webrtc.providers.broker import BrokerConfig, BrokerProvider


def test_strict_h264_codec_preferences_exclude_fallbacks() -> None:
    class VideoTransceiver:
        kind = "video"

        def __init__(self) -> None:
            self.preferences: list[Any] = []

        def setCodecPreferences(self, codecs: list[Any]) -> None:
            self.preferences = codecs

    transceiver = VideoTransceiver()

    class PeerConnection:
        def getTransceivers(self) -> list[VideoTransceiver]:
            return [transceiver]

    provider = BrokerProvider(BrokerConfig(api_key="test-key", strict_video_codec=True))
    provider._pc = cast("Any", PeerConnection())
    provider._prefer_video_codec("h264")

    assert transceiver.preferences
    assert {codec.mimeType.lower() for codec in transceiver.preferences} == {"video/h264"}


def test_strict_video_codec_rejects_an_unavailable_encoder() -> None:
    provider = BrokerProvider(BrokerConfig(api_key="test-key", strict_video_codec=True))

    with pytest.raises(RuntimeError, match="Required video codec video/not-real"):
        provider._prefer_video_codec("not-real")


def test_ice_gathering_timeout_is_an_error() -> None:
    class StuckPeerConnection:
        iceGatheringState = "gathering"  # noqa: N815

        async def setLocalDescription(self, description: object) -> None:
            del description
            await asyncio.sleep(1)

    with pytest.raises(RuntimeError, match="ICE gathering did not complete"):
        asyncio.run(
            _set_local_description_with_ice_timeout(
                StuckPeerConnection(),
                object(),
                timeout=0.001,
            )
        )


def test_publisher_config_requires_h264() -> None:
    config = _publisher_config(
        broker_url="https://broker.example.test",
        api_key="test-key",
        robot_name="webrtc-test",
    )

    assert config.robot_type == "webrtc-mvp"
    assert config.video_codec == "h264"
    assert config.strict_video_codec is True


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
