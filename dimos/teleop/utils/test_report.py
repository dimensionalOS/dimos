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

"""Generated teleop recordings retain source-time report metrics."""

import json
from pathlib import Path

from dimos_generated.dimos_msgs.msg import VideoStats
from dimos_generated.geometry_msgs.msg import PoseStamped, TwistStamped
from dimos_generated.std_msgs.msg import Header, UInt32
import pytest

from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.time import time_from_nanoseconds
from dimos.teleop.utils.report import _run_duration, _summary, generate_report


def stamped(stamp_ns: int) -> TwistStamped:
    return TwistStamped(header=Header(stamp=time_from_nanoseconds(stamp_ns)))


def test_epoch_nanoseconds_and_out_of_order_source_frames() -> None:
    base = 1_700_000_000_123_456_789
    messages = [stamped(base + offset) for offset in (2, 0, 1)]
    summary = _summary(messages)
    assert summary["count"] == 3
    assert summary["rate_hz"] == pytest.approx(1e9)
    assert summary["jitter_ms"]["p50"] == pytest.approx(1e-6)
    assert _run_duration({"cmd_vel_stamped": messages}) == pytest.approx(2e-9)


def test_zero_source_stamp_and_unstamped_buttons() -> None:
    assert _run_duration({"poses": [stamped(0), stamped(1_000_000_000)]}) == 1.0
    buttons = _summary([UInt32(data=1), UInt32(data=0)])
    assert buttons["count"] == 2 and buttons["rate_hz"] is None
    assert buttons["jitter_ms"] is None


def test_generated_sqlite_recording_produces_report(tmp_path: Path) -> None:
    path = tmp_path / "recording_teleop_cdr.db"
    base = 1_700_000_000_123_456_789
    with SqliteStore(path=str(path)) as store:
        commands = store.stream("cmd_vel_stamped", TwistStamped)
        poses = store.stream("left_controller_output", PoseStamped)
        for index in range(3):
            header = Header(stamp=time_from_nanoseconds(base + index * 20_000_000))
            commands.append(TwistStamped(header=header), ts=100 + index)
            poses.append(PoseStamped(header=header), ts=100 + index)
        store.stream("teleop_buttons", UInt32).append(UInt32(data=3), ts=101)
        store.stream("video_stats", VideoStats).append(
            VideoStats(header=header, width=640, height=480, frames_dropped=2**32 + 1), ts=102
        )
    report = json.loads(generate_report(path).read_text())
    assert report["duration_s"] == 0.04
    assert report["streams"]["cmd_vel_stamped"]["rate_hz"] == pytest.approx(50)
    assert report["streams"]["left_controller_output"]["count"] == 3
    assert report["streams"]["teleop_buttons"]["count"] == 1
    assert report["streams"]["teleop_buttons"]["rate_hz"] is None
    assert report["video"]["frames_dropped"] == 2**32 + 1
