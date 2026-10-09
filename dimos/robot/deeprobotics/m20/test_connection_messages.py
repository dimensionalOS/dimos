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
    PoseWithCovariance,
    Quaternion,
    Twist,
    TwistWithCovariance,
    Vector3,
)
from dimos_generated.nav_msgs.msg import Odometry
from dimos_generated.std_msgs.msg import Bool, Header
from dimos_message_build.registry import decode as cdr_decode, encode as cdr_encode
import numpy as np

from dimos.robot.deeprobotics.m20.connection import M20Connection


def test_generated_forwarding_preserves_readiness_pose_clock_and_zero_stop(monkeypatch):
    module = M20Connection()
    velocities, poses = [], []
    monkeypatch.setattr(module.cmd_vel, "publish", velocities.append)
    monkeypatch.setattr(module.odom, "publish", poses.append)
    command = Twist(linear=Vector3(x=0.1, y=0.0, z=0.0), angular=Vector3(z=0.2, x=0.0, y=0.0))
    try:
        assert module.move(command) is False
        module._on_command_ready(Bool(data=True))
        assert module.move(command) is True
        source = Odometry(
            header=Header(frame_id="map", stamp=Time(sec=1, nanosec=123456789)),
            pose=PoseWithCovariance(
                pose=Pose(
                    position=Point(x=2, y=0.0, z=0.0),
                    orientation=Quaternion(w=1, x=0.0, y=0.0, z=0.0),
                ),
                covariance=np.zeros(36, dtype=np.float64),
            ),
            child_frame_id="",
            twist=TwistWithCovariance(
                twist=Twist(
                    linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)
                ),
                covariance=np.zeros(36, dtype=np.float64),
            ),
        )
        module._on_odometry(source)
        assert poses[0].header == source.header and poses[0].pose == source.pose.pose
        module.stop_movement()
        assert velocities[-1] == Twist(
            linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)
        )
        assert cdr_decode(cdr_encode(velocities[0]), Twist) == command
    finally:
        module.stop()
