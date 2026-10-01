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

from dimos_generated.geometry_msgs.msg import (
    Point,
    Pose,
    PoseStamped,
    PoseWithCovariance,
    Transform,
    TransformStamped,
    Vector3,
)
from dimos_generated.nav_msgs.msg import Odometry
import numpy as np
import pytest

from dimos.memory.store.memory import MemoryStore


@pytest.mark.parametrize(
    "center",
    [
        Point(x=1),
        Vector3(x=1),
        (1, 0, 0),
        np.array([1, 0, 0]),
        Pose(position=Point(x=1)),
        PoseStamped(pose=Pose(position=Point(x=1))),
        TransformStamped(transform=Transform(translation=Vector3(x=1))),
        Odometry(pose=PoseWithCovariance(pose=Pose(position=Point(x=1)))),
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
