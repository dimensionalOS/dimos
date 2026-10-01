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

"""CDR video telemetry carries declared fields, not a positional Joy payload."""

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.dimos_msgs.msg import VideoStats
from dimos_generated.std_msgs.msg import Header
import pytest

from dimos.memory.store.sqlite import SqliteStore
from dimos.teleop.utils.video_stats import video_stats_from_dict


def test_cdr_and_sqlite_preserve_exact_counters_and_source_header(tmp_path) -> None:
    expected = VideoStats(
        header=Header(stamp=Time(sec=1700000000, nanosec=123456789), frame_id="video"),
        fps=28.0,
        kbps=2100.5,
        width=1280,
        height=720,
        loss_pct=2.1,
        frames_dropped=2**32 + 1,
        freezes=2**32 + 2,
    )
    assert VideoStats.decode(expected.encode()) == expected
    assert VideoStats.msg_name == "dimos_msgs/msg/VideoStats"
    assert "uint64 frames_dropped" in VideoStats.schema
    path = tmp_path / "video.db"
    with SqliteStore(path=str(path)) as store:
        stream = store.stream("video_stats", VideoStats, codec="cdr")
        stream.append(expected, ts=1700000000.0)
    with SqliteStore(path=str(path)) as store:
        assert store.stream("video_stats").first().data == expected


def test_browser_json_maps_named_fields_and_preserves_zero_timestamp() -> None:
    stats = video_stats_from_dict(
        {
            "type": "video_stats",
            "ts": 0,
            "fps": 28.0,
            "kbps": 2100.5,
            "width": 1280,
            "height": 720,
            "frames_dropped": 2,
            "freezes": 14,
            "e2e_latency_ms": 87.5,
        }
    )
    assert stats.header.stamp.sec == 0
    assert stats.header.stamp.nanosec == 0
    assert stats.header.frame_id == "video"
    assert stats.fps == 28.0
    assert stats.kbps == 2100.5
    assert stats.width == 1280
    assert stats.height == 720
    assert stats.frames_dropped == 2
    assert stats.freezes == 14
    assert stats.e2e_latency_ms == 87.5


def test_null_metrics_use_defaults_and_missing_timestamp_gets_receipt_time() -> None:
    stats = video_stats_from_dict({"fps": None, "width": None})
    assert stats.fps == 0
    assert stats.width == 0
    assert stats.header.stamp.sec > 0


@pytest.mark.parametrize("field", ["width", "height", "frames_dropped", "freezes"])
def test_negative_unsigned_metrics_are_rejected(field: str) -> None:
    with pytest.raises(ValueError, match=field):
        video_stats_from_dict({field: -1})


@pytest.mark.parametrize("field,bits", [("width", 32), ("frames_dropped", 64)])
def test_oversized_unsigned_metrics_are_rejected(field: str, bits: int) -> None:
    with pytest.raises(ValueError, match=field):
        video_stats_from_dict({field: 1 << bits})
