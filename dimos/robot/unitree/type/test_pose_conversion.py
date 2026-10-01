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

import copy

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import PoseStamped
from dimos_generated.std_msgs.msg import Header
import pytest

from dimos.robot.unitree.type.odometry import pose_from_webrtc_odometry, raw_odometry_msg_sample


@pytest.mark.parametrize("seconds,nanoseconds", [(-1, 987654321), (1700000000, 123456789)])
def test_device_pose_preserves_original_ros_header(seconds, nanoseconds):
    raw = copy.deepcopy(raw_odometry_msg_sample)
    raw["data"]["header"]["stamp"] = {"sec": seconds, "nanosec": nanoseconds}
    message = pose_from_webrtc_odometry(raw)
    decoded = PoseStamped.decode(message.encode())
    assert decoded.header == Header(frame_id="odom", stamp=Time(sec=seconds, nanosec=nanoseconds))
    assert decoded.pose.position.x == 5.961965
    assert decoded.pose.position.y == -2.916958
    assert decoded.pose.position.z == 0.319509
    assert decoded.pose.orientation.w == -0.242112
    message.header.frame_id = "changed"
    assert raw["data"]["header"]["frame_id"] == "odom"


def test_explicit_arrival_header_is_copied():
    header = Header(frame_id="world", stamp=Time(sec=5, nanosec=123))
    message = pose_from_webrtc_odometry(raw_odometry_msg_sample, header=header)
    assert message.header == header
    header.stamp.nanosec = 999
    assert message.header.stamp.nanosec == 123
