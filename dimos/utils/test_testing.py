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

import hashlib
import os
import subprocess

import reactivex as rx
from reactivex import operators as ops

from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.robot.unitree_webrtc.type.odometry import Odometry
from dimos.utils import testing
from dimos.utils.data import get_data


def test_sensor_replay():
    counter = 0
    for message in testing.SensorReplay(name="office_lidar").iterate():
        counter += 1
        assert isinstance(message, dict)
    assert counter == 500


def test_sensor_replay_cast():
    counter = 0
    for message in testing.SensorReplay(
        name="office_lidar", autocast=LidarMessage.from_msg
    ).iterate():
        counter += 1
        assert isinstance(message, LidarMessage)
    assert counter == 500


def test_timed_sensor_replay():
    get_data("unitree_office_walk")
    odom_store = testing.TimedSensorReplay("unitree_office_walk/odom", autocast=Odometry.from_msg)

    itermsgs = []
    for msg in odom_store.iterate():
        itermsgs.append(msg)
        if len(itermsgs) > 9:
            break

    assert len(itermsgs) == 10

    print("\n")

    timed_msgs = []

    for msg in odom_store.stream().pipe(ops.take(10), ops.to_list()).run():
        timed_msgs.append(msg)

    assert len(timed_msgs) == 10

    for i in range(10):
        print(itermsgs[i], timed_msgs[i])
        assert itermsgs[i] == timed_msgs[i]


def test_iterate_ts_no_seek():
    """Test iterate_ts without seek (start_timestamp=None)"""
    odom_store = testing.TimedSensorReplay("unitree_office_walk/odom", autocast=Odometry.from_msg)

    # Test without seek
    ts_msgs = []
    for ts, msg in odom_store.iterate_ts():
        ts_msgs.append((ts, msg))
        if len(ts_msgs) >= 5:
            break

    assert len(ts_msgs) == 5
    # Check that we get tuples of (timestamp, data)
    for ts, msg in ts_msgs:
        assert isinstance(ts, float)
        assert isinstance(msg, Odometry)


def test_iterate_ts_with_from_timestamp():
    """Test iterate_ts with from_timestamp (absolute timestamp)"""
    odom_store = testing.TimedSensorReplay("unitree_office_walk/odom", autocast=Odometry.from_msg)

    # First get all messages to find a good seek point
    all_msgs = []
    for ts, msg in odom_store.iterate_ts():
        all_msgs.append((ts, msg))
        if len(all_msgs) >= 10:
            break

    # Seek to timestamp of 5th message
    seek_timestamp = all_msgs[4][0]

    # Test with from_timestamp
    seeked_msgs = []
    for ts, msg in odom_store.iterate_ts(from_timestamp=seek_timestamp):
        seeked_msgs.append((ts, msg))
        if len(seeked_msgs) >= 5:
            break

    assert len(seeked_msgs) == 5
    # First message should be at or after seek timestamp
    assert seeked_msgs[0][0] >= seek_timestamp
    # Should match the data from position 5 onward
    assert seeked_msgs[0][1] == all_msgs[4][1]


def test_iterate_ts_with_relative_seek():
    """Test iterate_ts with seek (relative seconds after first timestamp)"""
    odom_store = testing.TimedSensorReplay("unitree_office_walk/odom", autocast=Odometry.from_msg)

    # Get first few messages to understand timing
    all_msgs = []
    for ts, msg in odom_store.iterate_ts():
        all_msgs.append((ts, msg))
        if len(all_msgs) >= 10:
            break

    # Calculate relative seek time (e.g., 0.5 seconds after start)
    first_ts = all_msgs[0][0]
    seek_seconds = 0.5
    expected_start_ts = first_ts + seek_seconds

    # Test with relative seek
    seeked_msgs = []
    for ts, msg in odom_store.iterate_ts(seek=seek_seconds):
        seeked_msgs.append((ts, msg))
        if len(seeked_msgs) >= 5:
            break

    # First message should be at or after expected timestamp
    assert seeked_msgs[0][0] >= expected_start_ts
    # Make sure we're actually skipping some messages
    assert seeked_msgs[0][0] > first_ts


def test_stream_with_seek():
    """Test stream method with seek parameters"""
    odom_store = testing.TimedSensorReplay("unitree_office_walk/odom", autocast=Odometry.from_msg)

    # Test stream with relative seek
    msgs_with_seek = []
    for msg in odom_store.stream(seek=0.2).pipe(ops.take(5), ops.to_list()).run():
        msgs_with_seek.append(msg)

    assert len(msgs_with_seek) == 5

    # Test stream with from_timestamp
    # First get a reference timestamp
    first_msgs = []
    for msg in odom_store.stream().pipe(ops.take(3), ops.to_list()).run():
        first_msgs.append(msg)

    # Now test from_timestamp (would need actual timestamps from iterate_ts to properly test)
    # This is a basic test to ensure the parameter is accepted
    msgs_with_timestamp = []
    for msg in (
        odom_store.stream(from_timestamp=1000000000.0).pipe(ops.take(3), ops.to_list()).run()
    ):
        msgs_with_timestamp.append(msg)


def test_duration_with_loop():
    """Test duration parameter with looping in TimedSensorReplay"""
    odom_store = testing.TimedSensorReplay("unitree_office_walk/odom", autocast=Odometry.from_msg)

    # Collect timestamps from a small duration window
    collected_ts = []
    duration = 0.3  # 300ms window

    # First pass: collect timestamps in the duration window
    for ts, msg in odom_store.iterate_ts(duration=duration):
        collected_ts.append(ts)
        if len(collected_ts) >= 100:  # Safety limit
            break

    # Should have some messages but not too many
    assert len(collected_ts) > 0
    assert len(collected_ts) < 20  # Assuming ~30Hz data

    # Test looping with duration - should repeat the same window
    loop_count = 0
    prev_ts = None

    for ts, msg in odom_store.iterate_ts(duration=duration, loop=True):
        if prev_ts is not None and ts < prev_ts:
            # We've looped back to the beginning
            loop_count += 1
            if loop_count >= 2:  # Stop after 2 full loops
                break
        prev_ts = ts

    assert loop_count >= 2  # Verify we actually looped
