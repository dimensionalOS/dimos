# Copyright 2025-2026 Dimensional Inc.
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

from types import SimpleNamespace
from unittest.mock import Mock

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import (
    Point,
    Pose,
    PoseStamped,
    PoseWithCovariance,
    Quaternion,
    Twist,
    TwistStamped,
    TwistWithCovariance,
    Vector3,
)
from dimos_generated.nav_msgs.msg import Odometry
from dimos_generated.std_msgs.msg import Header, Int32
from dimos_message_build.registry import decode as cdr_decode, encode as cdr_encode
import numpy as np
import pytest

from dimos.robot.unitree.b1.connection import B1ConnectionModule, MockB1ConnectionModule, RobotMode
from dimos.robot.unitree.b1.joystick_module import JoystickModule


def test_b1_odometry_forwarding_preserves_generated_pose_and_exact_header():
    source = Odometry(
        header=Header(frame_id="map", stamp=Time(sec=1700000000, nanosec=123456789)),
        pose=PoseWithCovariance(
            pose=Pose(
                position=Point(x=1, y=2, z=3), orientation=Quaternion(w=1, x=0.0, y=0.0, z=0.0)
            ),
            covariance=np.zeros(36, dtype=np.float64),
        ),
        child_frame_id="",
        twist=TwistWithCovariance(
            twist=Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)),
            covariance=np.zeros(36, dtype=np.float64),
        ),
    )
    received = []
    proxy = SimpleNamespace(odom_pose=SimpleNamespace(publish=received.append))
    B1ConnectionModule._publish_odom_pose(proxy, source)
    result = cdr_decode(cdr_encode(received[0]), PoseStamped)
    assert result.header == source.header
    assert result.pose == source.pose.pose


@pytest.mark.parametrize("mode", [RobotMode.WALK, RobotMode.STAND])
def test_b1_generated_command_mapping_preserves_mode_clamps_and_source(mode):
    module = MockB1ConnectionModule()
    try:
        module.handle_mode(cdr_decode(cdr_encode(Int32(data=mode)), Int32))
        message = TwistStamped(
            header=Header(frame_id="base_link", stamp=Time(sec=1700000000, nanosec=123456789)),
            twist=Twist(linear=Vector3(x=2, y=-2, z=2), angular=Vector3(x=2, y=-2, z=4)),
        )
        before = cdr_encode(message)
        module.handle_twist_stamped(cdr_decode(before, TwistStamped))
        command = module._current_cmd
        assert module.current_mode == mode and command.mode == mode
        assert command.lx == -1 and command.ly == 1
        assert command.rx == (1 if mode == RobotMode.WALK else -1)
        assert command.ry == (0 if mode == RobotMode.WALK else -1)
        assert cdr_encode(message) == before
        assert module.socket is None
    finally:
        module.stop()


def test_b1_joystick_stop_publishes_generated_zero_without_starting_loop(monkeypatch):
    module = JoystickModule()
    thread = Mock()
    module._thread = thread
    received = []
    monkeypatch.setattr(module.twist_out, "publish", received.append)
    monkeypatch.setattr("dimos.robot.unitree.b1.joystick_module.time.time", lambda: 1700000000.25)
    module.stop()
    assert len(received) == 1
    value = cdr_decode(cdr_encode(received[0]), TwistStamped)
    assert value.header.frame_id == "base_link"
    assert value.header.stamp == Time(sec=1700000000, nanosec=250000000)
    assert value.twist == Twist(
        linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)
    )
    thread.join.assert_called_once()
