# Copyright 2025 Dimensional Inc.
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

import time
from datetime import datetime, timezone

import pytest
from reactivex import operators as ops
from reactivex.scheduler import ThreadPoolScheduler

from dimos.msgs.sensor_msgs import Image
from dimos.types.timestamped import (
    Timestamped,
    TimestampedBufferCollection,
    TimestampedCollection,
    align_timestamped,
    to_datetime,
    to_ros_stamp,
)
from dimos.utils import testing
from dimos.utils.data import get_data
from dimos.utils.reactive import backpressure


def test_timestamped_dt_method():
    ts = 1751075203.4120464
    timestamped = Timestamped(ts)
    dt = timestamped.dt()
    assert isinstance(dt, datetime)
    assert abs(dt.timestamp() - ts) < 1e-6
    assert dt.tzinfo is not None, "datetime should be timezone-aware"


def test_to_ros_stamp():
    """Test the to_ros_stamp function with different input types."""

    # Test with float timestamp
    ts_float = 1234567890.123456789
    result = to_ros_stamp(ts_float)
    assert result.sec == 1234567890
    # Float precision limitation - check within reasonable range
    assert abs(result.nanosec - 123456789) < 1000

    # Test with integer timestamp
    ts_int = 1234567890
    result = to_ros_stamp(ts_int)
    assert result.sec == 1234567890
    assert result.nanosec == 0

    # Test with datetime object
    dt = datetime(2009, 2, 13, 23, 31, 30, 123456, tzinfo=timezone.utc)
    result = to_ros_stamp(dt)
    assert result.sec == 1234567890
    assert abs(result.nanosec - 123456000) < 1000  # Allow small rounding error


def test_to_datetime():
    """Test the to_datetime function with different input types."""

    # Test with float timestamp
    ts_float = 1234567890.123456
    dt = to_datetime(ts_float)
    assert isinstance(dt, datetime)
    assert dt.tzinfo is not None  # Should have timezone
    assert abs(dt.timestamp() - ts_float) < 1e-6

    # Test with integer timestamp
    ts_int = 1234567890
    dt = to_datetime(ts_int)
    assert isinstance(dt, datetime)
    assert dt.tzinfo is not None
    assert dt.timestamp() == ts_int

    # Test with RosStamp
    ros_stamp = {"sec": 1234567890, "nanosec": 123456000}
    dt = to_datetime(ros_stamp)
    assert isinstance(dt, datetime)
    assert dt.tzinfo is not None
    expected_ts = 1234567890.123456
    assert abs(dt.timestamp() - expected_ts) < 1e-6

    # Test with datetime (already has timezone)
    dt_input = datetime(2009, 2, 13, 23, 31, 30, tzinfo=timezone.utc)
    dt_result = to_datetime(dt_input)
    assert dt_result.tzinfo is not None
    # Should convert to local timezone by default

    # Test with naive datetime (no timezone)
    dt_naive = datetime(2009, 2, 13, 23, 31, 30)
    dt_result = to_datetime(dt_naive)
    assert dt_result.tzinfo is not None

    # Test with specific timezone
    dt_utc = to_datetime(ts_float, tz=timezone.utc)
    assert dt_utc.tzinfo == timezone.utc
    assert abs(dt_utc.timestamp() - ts_float) < 1e-6


class SimpleTimestamped(Timestamped):
    def __init__(self, ts: float, data: str):
        super().__init__(ts)
        self.data = data


@pytest.fixture
def sample_items():
    return [
        SimpleTimestamped(1.0, "first"),
        SimpleTimestamped(3.0, "third"),
        SimpleTimestamped(5.0, "fifth"),
        SimpleTimestamped(7.0, "seventh"),
    ]


@pytest.fixture
def collection(sample_items):
    return TimestampedCollection(sample_items)


def test_empty_collection():
    collection = TimestampedCollection()
    assert len(collection) == 0
    assert collection.duration() == 0.0
    assert collection.time_range() is None
    assert collection.find_closest(1.0) is None


def test_add_items():
    collection = TimestampedCollection()
    item1 = SimpleTimestamped(2.0, "two")
    item2 = SimpleTimestamped(1.0, "one")

    collection.add(item1)
    collection.add(item2)

    assert len(collection) == 2
    assert collection[0].data == "one"  # Should be sorted by timestamp
    assert collection[1].data == "two"


def test_find_closest(collection):
    # Exact match
    assert collection.find_closest(3.0).data == "third"

    # Between items (closer to left)
    assert collection.find_closest(1.5, tolerance=1.0).data == "first"

    # Between items (closer to right)
    assert collection.find_closest(3.5, tolerance=1.0).data == "third"

    # Exactly in the middle (should pick the later one due to >= comparison)
    assert (
        collection.find_closest(4.0, tolerance=1.0).data == "fifth"
    )  # 4.0 is equidistant from 3.0 and 5.0

    # Before all items
    assert collection.find_closest(0.0, tolerance=1.0).data == "first"

    # After all items
    assert collection.find_closest(10.0, tolerance=4.0).data == "seventh"

    # low tolerance, should return None
    assert collection.find_closest(10.0, tolerance=2.0) is None


def test_find_before_after(collection):
    # Find before
    assert collection.find_before(2.0).data == "first"
    assert collection.find_before(5.5).data == "fifth"
    assert collection.find_before(1.0) is None  # Nothing before first item

    # Find after
    assert collection.find_after(2.0).data == "third"
    assert collection.find_after(5.0).data == "seventh"
    assert collection.find_after(7.0) is None  # Nothing after last item


def test_merge_collections():
    collection1 = TimestampedCollection(
        [
            SimpleTimestamped(1.0, "a"),
            SimpleTimestamped(3.0, "c"),
        ]
    )
    collection2 = TimestampedCollection(
        [
            SimpleTimestamped(2.0, "b"),
            SimpleTimestamped(4.0, "d"),
        ]
    )

    merged = collection1.merge(collection2)

    assert len(merged) == 4
    assert [item.data for item in merged] == ["a", "b", "c", "d"]


def test_duration_and_range(collection):
    assert collection.duration() == 6.0  # 7.0 - 1.0
    assert collection.time_range() == (1.0, 7.0)


def test_slice_by_time(collection):
    # Slice inclusive of boundaries
    sliced = collection.slice_by_time(2.0, 6.0)
    assert len(sliced) == 2
    assert sliced[0].data == "third"
    assert sliced[1].data == "fifth"

    # Empty slice
    empty_slice = collection.slice_by_time(8.0, 10.0)
    assert len(empty_slice) == 0

    # Slice all
    all_slice = collection.slice_by_time(0.0, 10.0)
    assert len(all_slice) == 4


def test_iteration(collection):
    items = list(collection)
    assert len(items) == 4
    assert [item.ts for item in items] == [1.0, 3.0, 5.0, 7.0]


def test_single_item_collection():
    single = TimestampedCollection([SimpleTimestamped(5.0, "only")])
    assert single.duration() == 0.0
    assert single.time_range() == (5.0, 5.0)


def test_time_window_collection():
    # Create a collection with a 2-second window
    window = TimestampedBufferCollection[SimpleTimestamped](window_duration=2.0)

    # Add messages at different timestamps
    window.add(SimpleTimestamped(1.0, "msg1"))
    window.add(SimpleTimestamped(2.0, "msg2"))
    window.add(SimpleTimestamped(3.0, "msg3"))

    # At this point, all messages should be present (within 2s window)
    assert len(window) == 3

    # Add a message at t=4.0, should keep messages from t=2.0 onwards
    window.add(SimpleTimestamped(4.0, "msg4"))
    assert len(window) == 3  # msg1 should be dropped
    assert window[0].data == "msg2"  # oldest is now msg2
    assert window[-1].data == "msg4"  # newest is msg4

    # Add a message at t=5.5, should drop msg2 and msg3
    window.add(SimpleTimestamped(5.5, "msg5"))
    assert len(window) == 2  # only msg4 and msg5 remain
    assert window[0].data == "msg4"
    assert window[1].data == "msg5"

    # Verify time range
    assert window.start_ts == 4.0
    assert window.end_ts == 5.5


def test_timestamp_alignment():
    # Create a dedicated scheduler for this test to avoid thread leaks
    test_scheduler = ThreadPoolScheduler(max_workers=6)
    try:
        speed = 5.0

        # ensure that lfs package is downloaded
        get_data("unitree_office_walk")

        raw_frames = []

        def spy(image):
            raw_frames.append(image.ts)
            print(image.ts)
            return image

        # sensor reply of raw video frames
        video_raw = (
            testing.TimedSensorReplay(
                "unitree_office_walk/video", autocast=lambda x: Image.from_numpy(x).to_rgb()
            )
            .stream(speed)
            .pipe(ops.take(30))
        )

        processed_frames = []

        def process_video_frame(frame):
            processed_frames.append(frame.ts)
            time.sleep(0.5 / speed)
            return frame

        # fake reply of some 0.5s processor of video frames that drops messages
        fake_video_processor = backpressure(
            video_raw.pipe(ops.map(spy)), scheduler=test_scheduler
        ).pipe(ops.map(process_video_frame))

        aligned_frames = (
            align_timestamped(fake_video_processor, video_raw).pipe(ops.to_list()).run()
        )

        assert len(raw_frames) == 30
        assert len(processed_frames) > 2
        assert len(aligned_frames) > 2

        # Due to async processing, the last frame might not be aligned before completion
        assert len(aligned_frames) >= len(processed_frames) - 1

        for value in aligned_frames:
            [primary, secondary] = value
            diff = abs(primary.ts - secondary.ts)
            print(
                f"Aligned pair: primary={primary.ts:.6f}, secondary={secondary.ts:.6f}, diff={diff:.6f}s"
            )
            assert diff <= 0.05
    finally:
        # Always shutdown the scheduler to clean up threads
        test_scheduler.executor.shutdown(wait=True)
        # Give threads time to finish cleanup
        time.sleep(0.2)
