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
from dimos_generated.geometry_msgs.msg import Point, Quaternion
from dimos_generated.nav_msgs.msg import Odometry
import pytest

from dimos.msgs.geometry import transform_from_odometry


@pytest.mark.parametrize("seconds,nanoseconds", [(0, 0), (-1, 500000000), (1700000000, 123456789)])
def test_odometry_transform_preserves_frames_stamp_and_pose(seconds, nanoseconds):
    message = Odometry(child_frame_id="robot/lidar")
    message.header.frame_id = "robot/odom"
    message.header.stamp = Time(sec=seconds, nanosec=nanoseconds)
    message.pose.pose.position = Point(x=1, y=2, z=3)
    message.pose.pose.orientation = Quaternion(z=0.6, w=0.8)

    transformed = transform_from_odometry(message)

    assert transformed.header.frame_id == "robot/odom"
    assert transformed.child_frame_id == "robot/lidar"
    assert transformed.header.stamp.sec == seconds
    assert transformed.header.stamp.nanosec == nanoseconds
    assert (
        transformed.transform.translation.x,
        transformed.transform.translation.y,
        transformed.transform.translation.z,
    ) == (1, 2, 3)
    assert transformed.transform.rotation == message.pose.pose.orientation
    transformed.header.frame_id = "changed"
    transformed.transform.rotation.w = 0
    assert message.header.frame_id == "robot/odom"
    assert message.pose.pose.orientation.w == 0.8
