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

"""Headset driving uses the existing WebXR transport and its loss-of-input stop."""

import pytest

from dimos.msgs.sensor_msgs.Joy import Joy
from dimos.robot.galaxea.r1pro.sim.teleop import R1ProWebXR


@pytest.fixture
def headset(mocker):
    module = R1ProWebXR()
    publish = mocker.patch.object(module.cmd_vel, "publish")
    clock = mocker.patch("dimos.teleop.webxr.module.time.monotonic", return_value=10.0)
    try:
        yield module, publish, clock
    finally:
        module.stop()


def joy(hand, x=0.0, y=0.0):
    return Joy(frame_id=hand, axes=[x, y, 0.0, 0.0], buttons=[0] * 6).lcm_encode()


def velocity(publish):
    command = publish.call_args.args[0]
    return command.linear.x, command.linear.y, command.angular.z


def test_sticks_drive_strafe_turn_and_release(headset):
    module, publish, _ = headset
    assert module._on_joy_bytes(joy("left", -0.5, -1.0))
    assert module._on_joy_bytes(joy("right", -1.0))
    assert velocity(publish) == pytest.approx((0.2, 0.1, 0.3))

    module._on_joy_bytes(joy("left", 0.1, -0.1))
    module._on_joy_bytes(joy("right", 0.1))
    assert velocity(publish) == (0.0, 0.0, 0.0)


def test_lost_left_controller_cannot_keep_driving_with_live_right_hand(headset):
    module, publish, clock = headset
    module._on_joy_bytes(joy("left", y=-1.0))
    clock.return_value = 10.4
    module._on_joy_bytes(joy("right"))
    with module._lock:
        module._expire_stale_state(clock.return_value)
    assert velocity(publish) == (0.0, 0.0, 0.0)
    module._on_joy_bytes(joy("right", -1.0))
    assert velocity(publish) == pytest.approx((0.0, 0.0, 0.3))


def test_disconnect_clears_sticks_before_reconnect(headset):
    module, publish, _ = headset
    module._on_joy_bytes(joy("left", y=-1.0))
    module._reset_controller_state()
    assert velocity(publish) == (0.0, 0.0, 0.0)
    module._on_joy_bytes(joy("right"))
    assert velocity(publish) == (0.0, 0.0, 0.0)


def test_malformed_controller_packet_stops_base(headset):
    module, publish, _ = headset
    module._on_joy_bytes(joy("left", y=-1.0))
    assert not module._on_joy_bytes(Joy(frame_id="left", axes=[], buttons=[]).lcm_encode())
    assert velocity(publish) == (0.0, 0.0, 0.0)
