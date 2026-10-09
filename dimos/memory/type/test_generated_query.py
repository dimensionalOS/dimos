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

# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import (
    Point,
    Pose,
    PoseStamped,
    PoseWithCovariance,
    Quaternion,
    Transform,
    TransformStamped,
    Twist,
    TwistWithCovariance,
    Vector3,
)
from dimos_generated.nav_msgs.msg import Odometry
from dimos_generated.std_msgs.msg import Header
import numpy as np
import pytest

from dimos.memory.store.memory import MemoryStore


@pytest.mark.parametrize(
    "center",
    [
        Point(x=1, y=0.0, z=0.0),
        Vector3(x=1, y=0.0, z=0.0),
        (1, 0, 0),
        np.array([1, 0, 0]),
        Pose(position=Point(x=1, y=0.0, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)),
        PoseStamped(
            pose=Pose(
                position=Point(x=1, y=0.0, z=0.0),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
            header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
        ),
        TransformStamped(
            transform=Transform(
                translation=Vector3(x=1, y=0.0, z=0.0),
                rotation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
            header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
            child_frame_id="",
        ),
        Odometry(
            pose=PoseWithCovariance(
                pose=Pose(
                    position=Point(x=1, y=0.0, z=0.0),
                    orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                ),
                covariance=np.zeros(36, dtype=np.float64),
            ),
            header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
            child_frame_id="",
            twist=TwistWithCovariance(
                twist=Twist(
                    linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)
                ),
                covariance=np.zeros(36, dtype=np.float64),
            ),
        ),
    ],
)
def test_near_accepts_generated_nested_positions_and_retains_radius_boundary(center):
    with MemoryStore() as store:
        stream = store.stream("values", str)
        stream.append("center", ts=1, pose=(1, 0, 0))
        stream.append("boundary", ts=2, pose=(2, 0, 0))
        stream.append("outside", ts=3, pose=(2.001, 0, 0))
        stream.append("poseless", ts=4)
        assert [obs.data for obs in stream.near(center, radius=1)] == ["center", "boundary"]
