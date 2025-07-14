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

from lcm_msgs.geometry_msgs import Orientation, Pose, PoseStamped, Vector3
from lcm_msgs.std_msgs import Header


def test_vector_init():
    vector = Vector3(x=1.0, y=2.0, z=3.0)
    assert vector.x == 1.0
    assert vector.y == 2.0
    assert vector.z == 3.0


def test_vector_defaults():
    vector = Vector3()
    assert vector.x == 0.0
    assert vector.y == 0.0
    assert vector.z == 0.0


def test_vector_partial_defaults():
    vector = Vector3(x=1.0)
    assert vector.x == 1.0
    assert vector.y == 0.0
    assert vector.z == 0.0


def test_vector_encode_decode():
    msg = Vector3(x=1.0, y=2.0, z=3.0).encode()
    assert isinstance(msg, bytes)
    vector = Vector3.decode(msg)
    assert vector.x == 1.0
    assert vector.y == 2.0
    assert vector.z == 3.0


def test_pose_stamped_init():
    pose = PoseStamped(
        pose=Pose(
            position=Vector3(1.0, 2.0, 3.0),
            orientation=Orientation(1.0, 2.0, 3.0, 4.0),
        ),
        header=Header(stamp=1234567890, frame_id="test_frame"),
    )
