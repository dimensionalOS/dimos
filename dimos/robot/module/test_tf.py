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

import pytest

import lcm
from dimos.msgs.geometry_msgs import Pose, PoseStamped, Quaternion, Transform, Vector3


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
