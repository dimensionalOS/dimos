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

from datetime import datetime, timezone
import time

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


def test_timestamped_dt_method() -> None:
    ts = 1751075203.4120464
    timestamped = Timestamped(ts)
    dt = timestamped.dt()
    assert isinstance(dt, datetime)
    assert abs(dt.timestamp() - ts) < 1e-6
    assert dt.tzinfo is not None, "datetime should be timezone-aware"


def test_to_ros_stamp() -> None:
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


def test_to_datetime() -> None:
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
    def __init__(self, ts: float, data: str) -> None:
        super().__init__(ts)
        self.data = data


@pytest.fixture
def test_scheduler():
    """Fixture that provides a ThreadPoolScheduler and cleans it up after the test."""
    scheduler = ThreadPoolScheduler(max_workers=6)
    yield scheduler
    # Cleanup after test
    scheduler.executor.shutdown(wait=True)
    time.sleep(0.2)  # Give threads time to finish cleanup


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


def test_empty_collection() -> None:
    collection = TimestampedCollection()
    assert len(collection) == 0
    assert collection.duration() == 0.0
    assert collection.time_range() is None
    assert collection.find_closest(1.0) is None


def test_add_items() -> None:
    collection = TimestampedCollection()
    item1 = SimpleTimestamped(2.0, "two")
    item2 = SimpleTimestamped(1.0, "one")

    collection.add(item1)
    collection.add(item2)

    assert len(collection) == 2
    assert collection[0].data == "one"  # Should be sorted by timestamp
    assert collection[1].data == "two"


def test_find_closest(collection) -> None:
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


def test_find_before_after(collection) -> None:
    # Find before
    assert collection.find_before(2.0).data == "first"
    assert collection.find_before(5.5).data == "fifth"
    assert collection.find_before(1.0) is None  # Nothing before first item

    # Find after
    assert collection.find_after(2.0).data == "third"
    assert collection.find_after(5.0).data == "seventh"
    assert collection.find_after(7.0) is None  # Nothing after last item


def test_merge_collections() -> None:
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


def test_duration_and_range(collection) -> None:
    assert collection.duration() == 6.0  # 7.0 - 1.0
    assert collection.time_range() == (1.0, 7.0)


def test_slice_by_time(collection) -> None:
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


def test_iteration(collection) -> None:
    items = list(collection)
    assert len(items) == 4
    assert [item.ts for item in items] == [1.0, 3.0, 5.0, 7.0]


def test_single_item_collection() -> None:
    single = TimestampedCollection([SimpleTimestamped(5.0, "only")])
    assert single.duration() == 0.0
    assert single.time_range() == (5.0, 5.0)


def test_time_window_collection() -> None:
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


def test_timestamp_alignment(test_scheduler) -> None:
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
    # Pass the scheduler to backpressure to manage threads properly
    fake_video_processor = backpressure(
        video_raw.pipe(ops.map(spy)), scheduler=test_scheduler
    ).pipe(ops.map(process_video_frame))

    aligned_frames = align_timestamped(fake_video_processor, video_raw).pipe(ops.to_list()).run()

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

    assert len(aligned_frames) > 2


def test_timestamp_alignment_primary_first() -> None:
    """Test alignment when primary messages arrive before secondary messages."""
    from reactivex import Subject

    primary_subject = Subject()
    secondary_subject = Subject()

    results = []

    # Set up alignment with a 2-second buffer
    aligned = align_timestamped(
        primary_subject, secondary_subject, buffer_size=2.0, match_tolerance=0.1
    )

    # Subscribe to collect results
    aligned.subscribe(lambda x: results.append(x))

    # Send primary messages first
    primary1 = SimpleTimestamped(1.0, "primary1")
    primary2 = SimpleTimestamped(2.0, "primary2")
    primary3 = SimpleTimestamped(3.0, "primary3")

    primary_subject.on_next(primary1)
    primary_subject.on_next(primary2)
    primary_subject.on_next(primary3)

    # At this point, no results should be emitted (no secondaries yet)
    assert len(results) == 0

    # Send secondary messages that match primary1 and primary2
    secondary1 = SimpleTimestamped(1.05, "secondary1")  # Matches primary1
    secondary2 = SimpleTimestamped(2.02, "secondary2")  # Matches primary2

    secondary_subject.on_next(secondary1)
    assert len(results) == 1  # primary1 should now be matched
    assert results[0][0].data == "primary1"
    assert results[0][1].data == "secondary1"

    secondary_subject.on_next(secondary2)
    assert len(results) == 2  # primary2 should now be matched
    assert results[1][0].data == "primary2"
    assert results[1][1].data == "secondary2"

    # Send a secondary that's too far from primary3
    secondary_far = SimpleTimestamped(3.5, "secondary_far")  # Too far from primary3
    secondary_subject.on_next(secondary_far)
    # At this point primary3 is removed as unmatchable since secondary progressed past it
    assert len(results) == 2  # primary3 should not match (outside tolerance)

    # Send a new primary that can match with the future secondary
    primary4 = SimpleTimestamped(3.45, "primary4")
    primary_subject.on_next(primary4)
    assert len(results) == 3  # Should match with secondary_far
    assert results[2][0].data == "primary4"
    assert results[2][1].data == "secondary_far"

    # Complete the streams
    primary_subject.on_completed()
    secondary_subject.on_completed()


def test_timestamp_alignment_multiple_secondaries() -> None:
    """Test alignment with multiple secondary observables."""
    from reactivex import Subject

    primary_subject = Subject()
    secondary1_subject = Subject()
    secondary2_subject = Subject()

    results = []

    # Set up alignment with two secondary streams
    aligned = align_timestamped(
        primary_subject,
        secondary1_subject,
        secondary2_subject,
        buffer_size=1.0,
        match_tolerance=0.05,
    )

    # Subscribe to collect results
    aligned.subscribe(lambda x: results.append(x))

    # Send a primary message
    primary1 = SimpleTimestamped(1.0, "primary1")
    primary_subject.on_next(primary1)

    # No results yet (waiting for both secondaries)
    assert len(results) == 0

    # Send first secondary
    sec1_msg1 = SimpleTimestamped(1.01, "sec1_msg1")
    secondary1_subject.on_next(sec1_msg1)

    # Still no results (waiting for secondary2)
    assert len(results) == 0

    # Send second secondary
    sec2_msg1 = SimpleTimestamped(1.02, "sec2_msg1")
    secondary2_subject.on_next(sec2_msg1)

    # Now we should have a result
    assert len(results) == 1
    assert results[0][0].data == "primary1"
    assert results[0][1].data == "sec1_msg1"
    assert results[0][2].data == "sec2_msg1"

    # Test partial match (one secondary missing)
    primary2 = SimpleTimestamped(2.0, "primary2")
    primary_subject.on_next(primary2)

    # Send only one secondary
    sec1_msg2 = SimpleTimestamped(2.01, "sec1_msg2")
    secondary1_subject.on_next(sec1_msg2)

    # No result yet
    assert len(results) == 1

    # Send a secondary2 that's too far
    sec2_far = SimpleTimestamped(2.1, "sec2_far")  # Outside tolerance
    secondary2_subject.on_next(sec2_far)

    # Still no result (secondary2 is outside tolerance)
    assert len(results) == 1

    # Complete the streams
    primary_subject.on_completed()
    secondary1_subject.on_completed()
    secondary2_subject.on_completed()


def test_timestamp_alignment_delayed_secondary() -> None:
    """Test alignment when secondary messages arrive late but still within tolerance."""
    from reactivex import Subject

    primary_subject = Subject()
    secondary_subject = Subject()

    results = []

    # Set up alignment with a 2-second buffer
    aligned = align_timestamped(
        primary_subject, secondary_subject, buffer_size=2.0, match_tolerance=0.1
    )

    # Subscribe to collect results
    aligned.subscribe(lambda x: results.append(x))

    # Send primary messages
    primary1 = SimpleTimestamped(1.0, "primary1")
    primary2 = SimpleTimestamped(2.0, "primary2")
    primary3 = SimpleTimestamped(3.0, "primary3")

    primary_subject.on_next(primary1)
    primary_subject.on_next(primary2)
    primary_subject.on_next(primary3)

    # No results yet
    assert len(results) == 0

    # Send delayed secondaries (in timestamp order)
    secondary1 = SimpleTimestamped(1.05, "secondary1")  # Matches primary1
    secondary_subject.on_next(secondary1)
    assert len(results) == 1  # primary1 matched
    assert results[0][0].data == "primary1"
    assert results[0][1].data == "secondary1"

    secondary2 = SimpleTimestamped(2.02, "secondary2")  # Matches primary2
    secondary_subject.on_next(secondary2)
    assert len(results) == 2  # primary2 matched
    assert results[1][0].data == "primary2"
    assert results[1][1].data == "secondary2"

    # Now send a secondary that's past primary3's match window
    secondary_future = SimpleTimestamped(3.2, "secondary_future")  # Too far from primary3
    secondary_subject.on_next(secondary_future)
    # At this point, primary3 should be removed as unmatchable
    assert len(results) == 2  # No new matches

    # Send a new primary that can match with secondary_future
    primary4 = SimpleTimestamped(3.15, "primary4")
    primary_subject.on_next(primary4)
    assert len(results) == 3  # Should match immediately
    assert results[2][0].data == "primary4"
    assert results[2][1].data == "secondary_future"

    # Complete the streams
    primary_subject.on_completed()
    secondary_subject.on_completed()


def test_timestamp_alignment_buffer_cleanup() -> None:
    """Test that old buffered primaries are cleaned up."""
    import time as time_module

    from reactivex import Subject

    primary_subject = Subject()
    secondary_subject = Subject()

    results = []

    # Set up alignment with a 0.5-second buffer
    aligned = align_timestamped(
        primary_subject, secondary_subject, buffer_size=0.5, match_tolerance=0.05
    )

    # Subscribe to collect results
    aligned.subscribe(lambda x: results.append(x))

    # Use real timestamps for this test
    now = time_module.time()

    # Send an old primary
    old_primary = Timestamped(now - 1.0)  # 1 second ago
    old_primary.data = "old"
    primary_subject.on_next(old_primary)

    # Send a recent secondary to trigger cleanup
    recent_secondary = Timestamped(now)
    recent_secondary.data = "recent"
    secondary_subject.on_next(recent_secondary)

    # Old primary should not match (outside buffer window)
    assert len(results) == 0

    # Send a matching pair within buffer
    new_primary = Timestamped(now + 0.1)
    new_primary.data = "new_primary"
    new_secondary = Timestamped(now + 0.11)
    new_secondary.data = "new_secondary"

    primary_subject.on_next(new_primary)
    secondary_subject.on_next(new_secondary)

    # Should have one match
    assert len(results) == 1
    assert results[0][0].data == "new_primary"
    assert results[0][1].data == "new_secondary"

    # Complete the streams
    primary_subject.on_completed()
    secondary_subject.on_completed()
