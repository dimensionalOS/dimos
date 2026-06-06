#!/usr/bin/env python3
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

# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""Roundtrip + browser-payload tests for VideoStats."""

from dimos_lcm.sensor_msgs import Joy as LCMJoy

from dimos.msgs.sensor_msgs.VideoStats import VideoStats


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
    )


def test_lcm_roundtrip_preserves_all_fields() -> None:
    original = _sample()
    decoded = VideoStats.lcm_decode(original.lcm_encode())
    assert decoded == original
    # ts is float; round-trips through LCM int sec + int nsec, so allow small slack.
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
    }
    stats = VideoStats.from_dict(payload)
    assert stats.fps == 28.0
    assert stats.kbps == 2100.5
    assert stats.width == 1280
    assert stats.height == 720
    assert stats.frames_dropped == 2
    assert stats.freezes == 14


def test_decode_pads_short_axes_for_forward_compat() -> None:
    """An older recording with fewer metrics should still decode (zero-fill)."""
    short = LCMJoy()
    short.header.frame_id = "video"
    short.header.stamp.sec = 100
    short.header.stamp.nsec = 0
    short.axes_length = 3
    short.axes = [25.0, 1500.0, 640.0]  # only fps, kbps, width
    short.buttons_length = 0
    short.buttons = []

    decoded = VideoStats.lcm_decode(short.lcm_encode())
    assert decoded.fps == 25.0
    assert decoded.kbps == 1500.0
    assert decoded.width == 640
    # Missing fields zero-filled.
    assert decoded.height == 0
    assert decoded.freezes == 0


def test_carrier_is_lcm_joy() -> None:
    """Encoded bytes are a valid LCMJoy — proves we ride on the existing type."""
    blob = _sample().lcm_encode()
    j = LCMJoy.lcm_decode(blob)
    expected = [28.0, 2100.5, 1280.0, 720.0, 2.1, 28.0, 6.0, 2.0, 14.0]
    # Joy.axes is float32[], so compare with tolerance.
    assert len(j.axes) == len(expected)
    for got, want in zip(j.axes, expected, strict=True):
        assert abs(got - want) < 1e-5, f"axis mismatch: {got} vs {want}"
    assert j.header.frame_id == "video"


if __name__ == "__main__":
    test_lcm_roundtrip_preserves_all_fields()
    test_from_dict_matches_browser_payload_shape()
    test_decode_pads_short_axes_for_forward_compat()
    test_carrier_is_lcm_joy()
    print("all OK")
