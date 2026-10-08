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

"""Gamepad cockpit for the Go2: drive from a browser, with the sticks or the keyboard.

``unitree-go2-joystick-record`` with the web cockpit in place of pygame and Rerun. The
teleop panel takes a gamepad (Steam Deck, Legion Go, any pad the browser sees) or WASD,
and the bridge's ``tele_cmd_vel`` feeds the Go2's ``cmd_vel`` directly; the raw pad state
(continuous axes, buttons) goes out as ``joystick: Joy``. No mapper or navigation, so
nothing grows with the distance covered, and the dog's own obstacle avoidance is off so the
sticks are obeyed as given. ``--record`` keeps the raw streams, the commands
the dog received and the operator's sticks.

Usage:
    dimos --record run unitree-go2-gamepad-cockpit --robot-ip 192.168.12.1 --local-relay
    dimos --replay --replay-loop run unitree-go2-gamepad-cockpit --local-relay
"""

from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.web.cockpit import Battery, Col, Map3D, Row, Teleop, Video, cockpit

_web = cockpit(
    layout=Col(
        Row(
            Video("color_image", title="camera"),
            # the dog's own accumulated local cloud, voxelized around its pose
            Map3D(cloud="lidar", pose="odom", res=0.1, max_hz=2.0, title="lidar"),
        ),
        # a strip: two sticks and the speed
        Teleop(max_linear=0.5, max_angular=0.8, joystick="joystick", title="drive"),
        shares=[5, 1],
    ),
    # charge over the run and the time left, from the firmware's lowstate
    pages=[Battery()],
)
# The joystick channel makes cockpit() generate a bridge subclass; remap by that class.
_bridge = _web.blueprints[0].module

unitree_go2_gamepad_cockpit = autoconnect(
    GO2Connection.blueprint(),
    _web.remappings([(_bridge, "tele_cmd_vel", "cmd_vel")]),
).global_config(n_workers=2, robot_model="unitree_go2", obstacle_avoidance=False)
