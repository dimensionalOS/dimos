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

"""Roundtrip + browser-payload tests for VideoStats."""

from dimos_cdrgen.registry import decode, encode
import numpy as np

from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages.sensor_msgs.msg import Joy
from dimos.messages.std_msgs.msg import Header
from dimos.teleop.utils.video_stats import VideoStats


def _sample() -> VideoStats:
    return VideoStats(
        ts=1700_000_000.5,
        frame_id="video",
        fps=28.0,
        kbps=2100.5,
        width=1280,
        height=720,
        loss_pct=2.1,
        jitter_buffer_ms=28.0,
        decode_ms=6.0,
        frames_dropped=2,
        freezes=14,
        e2e_latency_ms=87.5,
    )


def test_cdr_roundtrip_preserves_all_fields() -> None:
    original = _sample()
    decoded = VideoStats.from_joy(decode(encode(original.to_joy()), Joy))
    assert decoded == original
    # ts is float; round-trips through CDR int sec + int nanosec, so allow small slack.
    assert abs(decoded.ts - original.ts) < 1e-6


def test_from_dict_matches_browser_payload_shape() -> None:
    """Browser ships exactly this JSON shape over state_reliable."""
    payload = {
        "type": "video_stats",  # extra key ignored
        "fps": 28.0,
        "kbps": 2100.5,
        "width": 1280,
        "height": 720,
        "loss_pct": 2.1,
        "jitter_ms": 5.5,  # extra unused key in current payload
        "frames_dropped": 2,
        "freezes": 14,
        "jitter_buffer_ms": 28.0,
        "decode_ms": 6.0,
        "e2e_latency_ms": 87.5,
    }
    stats = VideoStats.from_dict(payload)
    assert stats.fps == 28.0
    assert stats.kbps == 2100.5
    assert stats.width == 1280
    assert stats.height == 720
    assert stats.frames_dropped == 2
    assert stats.freezes == 14
    assert stats.e2e_latency_ms == 87.5


def test_decode_pads_short_axes_for_forward_compat() -> None:
    """A shorter canonical metrics layout zero-fills absent fields."""
    short = Joy(
        Header(Time(100, 0), "video"),
        np.array([25.0, 1500.0, 640.0], dtype=np.float32),
        np.empty(0, dtype=np.int32),
    )
    decoded = VideoStats.from_joy(decode(encode(short), Joy))
    assert decoded.fps == 25.0
    assert decoded.kbps == 1500.0
    assert decoded.width == 640
    # Missing fields zero-filled.
    assert decoded.height == 0
    assert decoded.freezes == 0


def test_carrier_is_generated_cdr_joy() -> None:
    """CDR contains the canonical Joy schema and unchanged positional metrics."""
    blob = encode(_sample().to_joy())
    j = decode(blob, Joy)
    expected = [28.0, 2100.5, 1280.0, 720.0, 2.1, 28.0, 6.0, 2.0, 14.0, 87.5]
    # Joy.axes is float32[], so compare with tolerance.
    assert len(j.axes) == len(expected)
    for got, want in zip(j.axes, expected, strict=True):
        assert abs(got - want) < 1e-5, f"axis mismatch: {got} vs {want}"
    assert j.header.frame_id == "video"


def test_received_zero_stamp_is_not_replaced_by_wall_time():
    value = _sample().to_joy()
    value.header.stamp = Time(0, 0)
    assert VideoStats.from_joy(value).ts == 0.0
