#!/usr/bin/env python3

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

import lcm
import pytest

from dimos.msgs.geometry_msgs import Pose, PoseStamped, Quaternion, Transform, Vector3
from dimos.protocol.tf.tf import TBuffer, TTBuffer


@pytest.mark.tool
def test_tf_broadcast_and_query():
    """Test TF broadcasting and querying between two TF instances.
    If you run foxglove-bridge this will show up in the UI"""
    from dimos.robot.module.tf import TF, TFConfig

    broadcaster = TF()
    querier = TF()

    # Create a transform from world to robot
    current_time = time.time()

    world_to_robot = Transform(
        translation=Vector3(1.0, 2.0, 3.0),
        rotation=Quaternion(0.0, 0.0, 0.0, 1.0),  # Identity rotation
        frame_id="world",
        child_frame_id="robot",
        ts=current_time,
    )

    # Broadcast the transform
    broadcaster.send(world_to_robot)

    # Give time for the message to propagate
    time.sleep(0.05)

    # Query should now be able to find the transform
    assert querier.can_transform("world", "robot", current_time)

    # Verify frames are available
    frames = querier.get_frames()
    assert "world" in frames
    assert "robot" in frames

    # Add another transform in the chain
    robot_to_sensor = Transform(
        translation=Vector3(0.5, 0.0, 0.2),
        rotation=Quaternion(0.0, 0.0, 0.707107, 0.707107),  # 90 degrees around Z
        frame_id="robot",
        child_frame_id="sensor",
        ts=current_time,
    )

    random_object_in_view = Pose(
        position=Vector3(1.0, 0.0, 0.0),
    )

    broadcaster.send(robot_to_sensor)
    time.sleep(0.05)

    # Should be able to query the full chain
    assert querier.can_transform("world", "sensor", current_time)

    t = querier.lookup("world", "sensor")
    print("FOUND T", t)

    # random_object_in_view.find_transform()

    # Stop services
    broadcaster.stop()
    querier.stop()


class TestTBuffer:
    def test_add_transform(self):
        buffer = TBuffer(buffer_size=10.0)
        transform = Transform(
            translation=Vector3(1.0, 2.0, 3.0),
            rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
            frame_id="world",
            child_frame_id="robot",
            ts=time.time(),
        )

        buffer.add(transform)
        assert len(buffer) == 1
        assert buffer[0] == transform

    def test_buffer_pruning(self):
        buffer = TBuffer(buffer_size=1.0)  # 1 second buffer

        # Add old transform
        old_time = time.time() - 2.0
        old_transform = Transform(
            translation=Vector3(1.0, 0.0, 0.0),
            frame_id="world",
            child_frame_id="robot",
            ts=old_time,
        )
        buffer.add(old_transform)

        # Add recent transform
        recent_transform = Transform(
            translation=Vector3(2.0, 0.0, 0.0),
            frame_id="world",
            child_frame_id="robot",
            ts=time.time(),
        )
        buffer.add(recent_transform)

        # Old transform should be pruned
        assert len(buffer) == 1
        assert buffer[0].translation.x == 2.0


class TestTTBuffer:
    def test_multiple_frame_pairs(self):
        ttbuffer = TTBuffer(buffer_size=10.0)

        # Add transforms for different frame pairs
        transform1 = Transform(
            translation=Vector3(1.0, 0.0, 0.0),
            frame_id="world",
            child_frame_id="robot1",
            ts=time.time(),
        )

        transform2 = Transform(
            translation=Vector3(2.0, 0.0, 0.0),
            frame_id="world",
            child_frame_id="robot2",
            ts=time.time(),
        )

        ttbuffer.receive_transform(transform1, transform2)

        # Should have two separate buffers
        assert len(ttbuffer.buffers) == 2
        assert ("world", "robot1") in ttbuffer.buffers
        assert ("world", "robot2") in ttbuffer.buffers

    def test_get_latest_transform(self):
        ttbuffer = TTBuffer()

        # Add multiple transforms
        for i in range(3):
            transform = Transform(
                translation=Vector3(float(i), 0.0, 0.0),
                frame_id="world",
                child_frame_id="robot",
                ts=time.time() + i * 0.1,
            )
            ttbuffer.receive_transform(transform)
            time.sleep(0.01)

        # Get latest transform
        latest = ttbuffer.get_transform("world", "robot")
        assert latest is not None
        assert latest.translation.x == 2.0

    def test_get_transform_at_time(self):
        ttbuffer = TTBuffer()
        base_time = time.time()

        # Add transforms at known times
        for i in range(5):
            transform = Transform(
                translation=Vector3(float(i), 0.0, 0.0),
                frame_id="world",
                child_frame_id="robot",
                ts=base_time + i * 0.5,
            )
            ttbuffer.receive_transform(transform)

        # Get transform closest to middle time
        middle_time = base_time + 1.25  # Should be closest to i=2 (t=1.0) or i=3 (t=1.5)
        result = ttbuffer.get_transform("world", "robot", time_point=middle_time)
        assert result is not None
        # At t=1.25, it's equidistant from i=2 (t=1.0) and i=3 (t=1.5)
        # The implementation picks the later one when equidistant
        assert result.translation.x == 3.0

    def test_time_tolerance(self):
        ttbuffer = TTBuffer()
        base_time = time.time()

        # Add single transform
        transform = Transform(
            translation=Vector3(1.0, 0.0, 0.0),
            frame_id="world",
            child_frame_id="robot",
            ts=base_time,
        )
        ttbuffer.receive_transform(transform)

        # Within tolerance
        result = ttbuffer.get_transform(
            "world", "robot", time_point=base_time + 0.1, time_tolerance=0.2
        )
        assert result is not None

        # Outside tolerance
        result = ttbuffer.get_transform(
            "world", "robot", time_point=base_time + 0.5, time_tolerance=0.1
        )
        assert result is None

    def test_nonexistent_frame_pair(self):
        ttbuffer = TTBuffer()

        # Try to get transform for non-existent frame pair
        result = ttbuffer.get_transform("foo", "bar")
        assert result is None

    def test_string_representations(self):
        # Test empty buffers
        empty_buffer = TBuffer()
        assert str(empty_buffer) == "TBuffer(empty)"

        empty_ttbuffer = TTBuffer()
        assert str(empty_ttbuffer) == "TTBuffer(empty)"

        # Test TBuffer with data
        buffer = TBuffer()
        base_time = time.time()
        for i in range(3):
            transform = Transform(
                translation=Vector3(float(i), 0.0, 0.0),
                frame_id="world",
                child_frame_id="robot",
                ts=base_time + i * 0.1,
            )
            buffer.add(transform)

        buffer_str = str(buffer)
        assert "3 msgs" in buffer_str
        assert "world -> robot" in buffer_str
        assert "0.20s" in buffer_str  # duration

        # Test TTBuffer with multiple frame pairs
        ttbuffer = TTBuffer()
        transforms = [
            Transform(frame_id="world", child_frame_id="robot1", ts=base_time),
            Transform(frame_id="world", child_frame_id="robot2", ts=base_time + 0.5),
            Transform(frame_id="robot1", child_frame_id="sensor", ts=base_time + 1.0),
        ]

        for t in transforms:
            ttbuffer.receive_transform(t)

        ttbuffer_str = str(ttbuffer)
        print("\nTTBuffer string representation:")
        print(ttbuffer_str)

        assert "TTBuffer(3 buffers):" in ttbuffer_str
        assert "TBuffer(1 msgs" in ttbuffer_str
        assert "world -> robot1" in ttbuffer_str
        assert "world -> robot2" in ttbuffer_str
        assert "robot1 -> sensor" in ttbuffer_str
