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

import math
import time

import pytest

from dimos.core import TF
from dimos.msgs.geometry_msgs import PoseStamped, Quaternion, Transform, Vector3
from dimos.protocol.tf import MultiTBuffer, TBuffer


# from https://foxglove.dev/blog/understanding-ros-transforms
def test_tf_ros_example() -> None:
    tf = TF()

    base_link_to_arm = Transform(
        translation=Vector3(1.0, -1.0, 0.0),
        rotation=Quaternion.from_euler(Vector3(0, 0, math.pi / 6)),
        frame_id="base_link",
        child_frame_id="arm",
        ts=time.time(),
    )

    arm_to_end = Transform(
        translation=Vector3(1.0, 1.0, 0.0),
        rotation=Quaternion(0.0, 0.0, 0.0, 1.0),  # Identity rotation
        frame_id="arm",
        child_frame_id="end_effector",
        ts=time.time(),
    )

    tf.publish(base_link_to_arm, arm_to_end)
    time.sleep(0.2)

    end_effector_global_pose = tf.get("base_link", "end_effector")

    assert end_effector_global_pose.translation.x == pytest.approx(1.366, abs=1e-3)
    assert end_effector_global_pose.translation.y == pytest.approx(0.366, abs=1e-3)

    tf.stop()


def test_tf_main() -> None:
    """Test TF broadcasting and querying between two TF instances.
    If you run foxglove-bridge this will show up in the UI"""

    # here we create broadcasting and receiving TF instance.
    # this is to verify that comms work multiprocess, normally
    # you'd use only one instance in your module
    broadcaster = TF()
    querier = TF()

    # Create a transform from world to robot
    current_time = time.time()

    world_to_charger = Transform(
        translation=Vector3(2.0, -2.0, 0.0),
        rotation=Quaternion.from_euler(Vector3(0, 0, 2)),
        frame_id="world",
        child_frame_id="charger",
        ts=current_time,
    )

    world_to_robot = Transform(
        translation=Vector3(1.0, 2.0, 3.0),
        rotation=Quaternion(0.0, 0.0, 0.0, 1.0),  # Identity rotation
        frame_id="world",
        child_frame_id="robot",
        ts=current_time,
    )

    # Broadcast the transform
    broadcaster.publish(world_to_robot)
    broadcaster.publish(world_to_charger)
    # Give time for the message to propagate
    time.sleep(0.05)

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

    broadcaster.publish(robot_to_sensor)

    time.sleep(0.05)

    # we can now query (from a separate process given we use querier) the transform tree
    chain_transform = querier.get("world", "sensor")

    # broadcaster will agree with us
    assert broadcaster.get("world", "sensor") == chain_transform

    # The chain should compose: world->robot (1,2,3) + robot->sensor (0.5,0,0.2)
    # Expected translation: (1.5, 2.0, 3.2)
    assert abs(chain_transform.translation.x - 1.5) < 0.001
    assert abs(chain_transform.translation.y - 2.0) < 0.001
    assert abs(chain_transform.translation.z - 3.2) < 0.001

    # we see something on camera
    random_object_in_view = PoseStamped(
        frame_id="random_object",
        position=Vector3(1, 0, 0),
    )

    print("Random obj", random_object_in_view)

    # random_object is perceived by the sensor
    # we create a transform pointing from sensor to object
    random_t = random_object_in_view.new_transform_from("sensor")

    # we could have also done
    assert random_t == random_object_in_view.new_transform_to("sensor").inverse()

    print("randm t", random_t)

    # we broadcast our object location
    broadcaster.publish(random_t)

    ## we could also publish world -> random_object if we wanted to
    # broadcaster.publish(
    #    broadcaster.get("world", "sensor") + random_object_in_view.new_transform("sensor").inverse()
    # )
    ## (this would mess with the transform system because it expects trees not graphs)
    ## and our random_object would get re-connected to world from sensor

    print(broadcaster)

    # Give time for the message to propagate
    time.sleep(0.05)

    # we know where the object is in the world frame now
    world_object = broadcaster.get("world", "random_object")

    # both instances agree
    assert querier.get("world", "random_object") == world_object

    print("world object", world_object)

    # if you have "diagon" https://diagon.arthursonzogni.com/ installed you can draw a graph
    print(broadcaster.graph())

    assert abs(world_object.translation.x - 1.5) < 0.001
    assert abs(world_object.translation.y - 3.0) < 0.001
    assert abs(world_object.translation.z - 3.2) < 0.001

    # this doesn't work atm
    robot_to_charger = broadcaster.get("robot", "charger")

    # Expected: robot->world->charger
    print(f"robot_to_charger translation: {robot_to_charger.translation}")
    print(f"robot_to_charger rotation: {robot_to_charger.rotation}")

    assert abs(robot_to_charger.translation.x - 1.0) < 0.001
    assert abs(robot_to_charger.translation.y - (-4.0)) < 0.001
    assert abs(robot_to_charger.translation.z - (-3.0)) < 0.001

    # Stop services (they were autostarted but don't know how to autostop)
    broadcaster.stop()
    querier.stop()


class TestTBuffer:
    def test_add_transform(self) -> None:
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

    def test_get(self) -> None:
        buffer = TBuffer()
        base_time = time.time()

        # Add transforms at different times
        for i in range(3):
            transform = Transform(
                translation=Vector3(float(i), 0.0, 0.0),
                frame_id="world",
                child_frame_id="robot",
                ts=base_time + i * 0.5,
            )
            buffer.add(transform)

        # Test getting latest transform
        latest = buffer.get()
        assert latest is not None
        assert latest.translation.x == 2.0

        # Test getting transform at specific time
        middle = buffer.get(time_point=base_time + 0.75)
        assert middle is not None
        assert middle.translation.x == 2.0  # Closest to i=1

        # Test time tolerance
        result = buffer.get(time_point=base_time + 10.0, time_tolerance=0.1)
        assert result is None  # Outside tolerance

    def test_buffer_pruning(self) -> None:
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


class TestMultiTBuffer:
    def test_multiple_frame_pairs(self) -> None:
        ttbuffer = MultiTBuffer(buffer_size=10.0)

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

    def test_graph(self) -> None:
        ttbuffer = MultiTBuffer(buffer_size=10.0)

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

        print(ttbuffer.graph())

    def test_get_latest_transform(self) -> None:
        ttbuffer = MultiTBuffer()

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
        latest = ttbuffer.get("world", "robot")
        assert latest is not None
        assert latest.translation.x == 2.0

    def test_get_transform_at_time(self) -> None:
        ttbuffer = MultiTBuffer()
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
        result = ttbuffer.get("world", "robot", time_point=middle_time)
        assert result is not None
        # At t=1.25, it's equidistant from i=2 (t=1.0) and i=3 (t=1.5)
        # The implementation picks the later one when equidistant
        assert result.translation.x == 3.0

    def test_time_tolerance(self) -> None:
        ttbuffer = MultiTBuffer()
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
        result = ttbuffer.get("world", "robot", time_point=base_time + 0.1, time_tolerance=0.2)
        assert result is not None

        # Outside tolerance
        result = ttbuffer.get("world", "robot", time_point=base_time + 0.5, time_tolerance=0.1)
        assert result is None

    def test_nonexistent_frame_pair(self) -> None:
        ttbuffer = MultiTBuffer()

        # Try to get transform for non-existent frame pair
        result = ttbuffer.get("foo", "bar")
        assert result is None

    def test_get_transform_search_direct(self) -> None:
        ttbuffer = MultiTBuffer()
        base_time = time.time()

        # Add direct transform
        transform = Transform(
            translation=Vector3(1.0, 0.0, 0.0),
            frame_id="world",
            child_frame_id="robot",
            ts=base_time,
        )
        ttbuffer.receive_transform(transform)

        # Search should return single transform
        result = ttbuffer.get_transform_search("world", "robot")
        assert result is not None
        assert len(result) == 1
        assert result[0].translation.x == 1.0

    def test_get_transform_search_chain(self) -> None:
        ttbuffer = MultiTBuffer()
        base_time = time.time()

        # Create transform chain: world -> robot -> sensor
        transform1 = Transform(
            translation=Vector3(1.0, 0.0, 0.0),
            frame_id="world",
            child_frame_id="robot",
            ts=base_time,
        )
        transform2 = Transform(
            translation=Vector3(0.0, 2.0, 0.0),
            frame_id="robot",
            child_frame_id="sensor",
            ts=base_time,
        )
        ttbuffer.receive_transform(transform1, transform2)

        # Search should find chain
        result = ttbuffer.get_transform_search("world", "sensor")
        assert result is not None
        assert len(result) == 2
        assert result[0].translation.x == 1.0  # world -> robot
        assert result[1].translation.y == 2.0  # robot -> sensor

    def test_get_transform_search_complex_chain(self) -> None:
        ttbuffer = MultiTBuffer()
        base_time = time.time()

        # Create more complex graph:
        # world -> base -> arm -> hand
        #      \-> robot -> sensor
        transforms = [
            Transform(
                frame_id="world",
                child_frame_id="base",
                translation=Vector3(1.0, 0.0, 0.0),
                ts=base_time,
            ),
            Transform(
                frame_id="base",
                child_frame_id="arm",
                translation=Vector3(0.0, 1.0, 0.0),
                ts=base_time,
            ),
            Transform(
                frame_id="arm",
                child_frame_id="hand",
                translation=Vector3(0.0, 0.0, 1.0),
                ts=base_time,
            ),
            Transform(
                frame_id="world",
                child_frame_id="robot",
                translation=Vector3(2.0, 0.0, 0.0),
                ts=base_time,
            ),
            Transform(
                frame_id="robot",
                child_frame_id="sensor",
                translation=Vector3(0.0, 2.0, 0.0),
                ts=base_time,
            ),
        ]

        for t in transforms:
            ttbuffer.receive_transform(t)

        # Find path world -> hand (should go through base -> arm)
        result = ttbuffer.get_transform_search("world", "hand")
        assert result is not None
        assert len(result) == 3
        assert result[0].child_frame_id == "base"
        assert result[1].child_frame_id == "arm"
        assert result[2].child_frame_id == "hand"

    def test_get_transform_search_no_path(self) -> None:
        ttbuffer = MultiTBuffer()
        base_time = time.time()

        # Create disconnected transforms
        transform1 = Transform(frame_id="world", child_frame_id="robot", ts=base_time)
        transform2 = Transform(frame_id="base", child_frame_id="sensor", ts=base_time)
        ttbuffer.receive_transform(transform1, transform2)

        # No path exists
        result = ttbuffer.get_transform_search("world", "sensor")
        assert result is None

    def test_get_transform_search_with_time(self) -> None:
        ttbuffer = MultiTBuffer()
        base_time = time.time()

        # Add transforms at different times
        old_transform = Transform(
            frame_id="world",
            child_frame_id="robot",
            translation=Vector3(1.0, 0.0, 0.0),
            ts=base_time - 10.0,
        )
        new_transform = Transform(
            frame_id="world",
            child_frame_id="robot",
            translation=Vector3(2.0, 0.0, 0.0),
            ts=base_time,
        )
        ttbuffer.receive_transform(old_transform, new_transform)

        # Search at specific time
        result = ttbuffer.get_transform_search("world", "robot", time_point=base_time)
        assert result is not None
        assert result[0].translation.x == 2.0

        # Search with time tolerance
        result = ttbuffer.get_transform_search(
            "world", "robot", time_point=base_time + 1.0, time_tolerance=0.1
        )
        assert result is None  # Outside tolerance

    def test_get_transform_search_shortest_path(self) -> None:
        ttbuffer = MultiTBuffer()
        base_time = time.time()

        # Create graph with multiple paths:
        # world -> A -> B -> target (3 hops)
        # world -> target (direct, 1 hop)
        transforms = [
            Transform(frame_id="world", child_frame_id="A", ts=base_time),
            Transform(frame_id="A", child_frame_id="B", ts=base_time),
            Transform(frame_id="B", child_frame_id="target", ts=base_time),
            Transform(frame_id="world", child_frame_id="target", ts=base_time),
        ]

        for t in transforms:
            ttbuffer.receive_transform(t)

        # BFS should find the direct path (shortest)
        result = ttbuffer.get_transform_search("world", "target")
        assert result is not None
        assert len(result) == 1  # Direct path, not the 3-hop path
        assert result[0].child_frame_id == "target"

    def test_string_representations(self) -> None:
        # Test empty buffers
        empty_buffer = TBuffer()
        assert str(empty_buffer) == "TBuffer(empty)"

        empty_ttbuffer = MultiTBuffer()
        assert str(empty_ttbuffer) == "MultiTBuffer(empty)"

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

        # Test MultiTBuffer with multiple frame pairs
        ttbuffer = MultiTBuffer()
        transforms = [
            Transform(frame_id="world", child_frame_id="robot1", ts=base_time),
            Transform(frame_id="world", child_frame_id="robot2", ts=base_time + 0.5),
            Transform(frame_id="robot1", child_frame_id="sensor", ts=base_time + 1.0),
        ]

        for t in transforms:
            ttbuffer.receive_transform(t)

        ttbuffer_str = str(ttbuffer)
        print("\nMultiTBuffer string representation:")
        print(ttbuffer_str)

        assert "MultiTBuffer(3 buffers):" in ttbuffer_str
        assert "TBuffer(world -> robot1, 1 msgs" in ttbuffer_str
        assert "TBuffer(world -> robot2, 1 msgs" in ttbuffer_str
        assert "TBuffer(robot1 -> sensor, 1 msgs" in ttbuffer_str

    def test_get_with_transform_chain_composition(self) -> None:
        ttbuffer = MultiTBuffer()
        base_time = time.time()

        # Create transform chain: world -> robot -> sensor
        # world -> robot: translate by (1, 0, 0)
        transform1 = Transform(
            translation=Vector3(1.0, 0.0, 0.0),
            rotation=Quaternion(0.0, 0.0, 0.0, 1.0),  # Identity
            frame_id="world",
            child_frame_id="robot",
            ts=base_time,
        )

        # robot -> sensor: translate by (0, 2, 0) and rotate 90 degrees around Z
        import math

        # 90 degrees around Z: quaternion (0, 0, sin(45°), cos(45°))
        transform2 = Transform(
            translation=Vector3(0.0, 2.0, 0.0),
            rotation=Quaternion(0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4)),
            frame_id="robot",
            child_frame_id="sensor",
            ts=base_time,
        )

        ttbuffer.receive_transform(transform1, transform2)

        # Get composed transform from world to sensor
        result = ttbuffer.get("world", "sensor")
        assert result is not None

        # The composed transform should:
        # 1. Apply world->robot translation: (1, 0, 0)
        # 2. Apply robot->sensor translation in robot frame: (0, 2, 0)
        # Total translation: (1, 2, 0)
        assert abs(result.translation.x - 1.0) < 1e-6
        assert abs(result.translation.y - 2.0) < 1e-6
        assert abs(result.translation.z - 0.0) < 1e-6

        # Rotation should be 90 degrees around Z (same as transform2)
        assert abs(result.rotation.x - 0.0) < 1e-6
        assert abs(result.rotation.y - 0.0) < 1e-6
        assert abs(result.rotation.z - math.sin(math.pi / 4)) < 1e-6
        assert abs(result.rotation.w - math.cos(math.pi / 4)) < 1e-6

        # Frame IDs should be correct
        assert result.frame_id == "world"
        assert result.child_frame_id == "sensor"

    def test_get_with_longer_transform_chain(self) -> None:
        ttbuffer = MultiTBuffer()
        base_time = time.time()

        # Create longer chain: world -> base -> arm -> hand
        # Each adds a translation along different axes
        transforms = [
            Transform(
                translation=Vector3(1.0, 0.0, 0.0),  # Move 1 along X
                rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
                frame_id="world",
                child_frame_id="base",
                ts=base_time,
            ),
            Transform(
                translation=Vector3(0.0, 2.0, 0.0),  # Move 2 along Y
                rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
                frame_id="base",
                child_frame_id="arm",
                ts=base_time,
            ),
            Transform(
                translation=Vector3(0.0, 0.0, 3.0),  # Move 3 along Z
                rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
                frame_id="arm",
                child_frame_id="hand",
                ts=base_time,
            ),
        ]

        for t in transforms:
            ttbuffer.receive_transform(t)

        # Get composed transform from world to hand
        result = ttbuffer.get("world", "hand")
        assert result is not None

        # Total translation should be sum of all: (1, 2, 3)
        assert abs(result.translation.x - 1.0) < 1e-6
        assert abs(result.translation.y - 2.0) < 1e-6
        assert abs(result.translation.z - 3.0) < 1e-6

        # Rotation should still be identity (all rotations were identity)
        assert abs(result.rotation.x - 0.0) < 1e-6
        assert abs(result.rotation.y - 0.0) < 1e-6
        assert abs(result.rotation.z - 0.0) < 1e-6
        assert abs(result.rotation.w - 1.0) < 1e-6

        assert result.frame_id == "world"
        assert result.child_frame_id == "hand"
