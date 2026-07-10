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

from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.limx.tron1 import protocol


def test_build_request_twist() -> None:
    twist = Twist(linear=Vector3(0.1, -0.2, 0.0), angular=Vector3(0.0, 0.0, 1.2))
    req = protocol.build_request_twist(twist, max_vx=0.2, max_vy=0.4, max_yaw=2.4)
    assert req["x"] == 0.5
    assert req["y"] == -0.5
    assert req["z"] == 0.5


def test_parse_notify_imu() -> None:
    payload = {
        "timestamp": 123000,
        "gyro": [0.1, 0.2, 0.3],
        "acc": [1.0, 2.0, 3.0],
        "quat": [0.0, 0.0, 0.0, 0.0],
    }
    imu = protocol.parse_notify_imu(payload, frame_id="imu_link")
    assert imu.ts == 123.0
    assert imu.frame_id == "imu_link"
    assert imu.orientation == Quaternion(0.0, 0.0, 0.0, 1.0)
    assert imu.angular_velocity.x == 0.1
    assert imu.linear_acceleration.z == 3.0


def test_parse_notify_odom() -> None:
    payload = {
        "timestamp": 456000,
        "pose_position": [1.0, 2.0, 0.0],
        "pose_orientation": [0.0, 0.0, 0.0, 1.0],
        "twist_linear": [0.5, 0.0, 0.0],
        "twist_angular": [0.0, 0.0, 0.1],
    }
    odom, pose = protocol.parse_notify_odom(payload, frame_id="odom", child_frame_id="base_link")
    assert odom.ts == 456.0
    assert odom.frame_id == "odom"
    assert odom.child_frame_id == "base_link"
    assert odom.x == 1.0
    assert odom.vx == 0.5
    assert pose.frame_id == "odom"
    assert pose.x == 1.0
