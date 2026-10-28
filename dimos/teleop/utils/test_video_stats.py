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

from dimos_cdrgen.registry import decode, encode

from dimos.messages.dimos_msgs.msg import VideoStats
from dimos.teleop.utils import video_stats


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
    stats = video_stats.from_dict(payload)
    assert stats.fps == 28.0
    assert stats.kbps == 2100.5
    assert stats.width == 1280
    assert stats.height == 720
    assert stats.frames_dropped == 2
    assert stats.freezes == 14
    assert stats.e2e_latency_ms == 87.5


def test_generated_video_stats_roundtrip_preserves_timestamp_and_metrics():
    original = video_stats.from_dict(
        {"ts": 17.25, "fps": 28.0, "width": 1280, "height": 720, "freezes": 14}
    )
    assert decode(encode(original), VideoStats) == original
    assert original.header.stamp.sec == 17
    assert original.header.stamp.nanosec == 250000000
    assert original.header.frame_id == "video"


def test_browser_null_metrics_become_zero():
    stats = video_stats.from_dict({"fps": None, "width": None, "e2e_latency_ms": None})
    assert (stats.fps, stats.width, stats.e2e_latency_ms) == (0.0, 0, 0.0)
