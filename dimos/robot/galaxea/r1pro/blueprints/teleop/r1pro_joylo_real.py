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

"""Robot-side OpenArm/Joy-Con teleop through CC's Galaxea ROS connection.

Run the operator-only blueprint on the laptop, in the same Zenoh domain.
Starts paused. The existing vendor stack must be started and checked explicitly.
"""

import math

from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.robot.galaxea.r1pro.blueprints.teleop.r1pro_joylo_inputs import _control_blueprint
from dimos.robot.galaxea.r1pro.connection import R1ProConnection, R1ProConnectionConfig
from dimos.robot.galaxea.r1pro.joylo import ARMS, TORSO, JoyloConfig
from dimos.robot.galaxea.r1pro.motion import motion_groups

_declaration = R1ProConnectionConfig(
    boot_vendor_stack=False,
    tracking_speed=1.0,
    enable_wrist_color=True,
    # Forward the camera's compressed frames as they arrive. RGB-D has its own
    # slower processing rate and must not set the operator video's cadence.
    color_publish_hz=0.0,
    enable_wrist_depth=False,
    publish_odom_tf=True,
)
_profile = JoyloConfig(
    leader_angle_period=math.tau,
    torso_link_lengths=(0.4, 0.3),
    base_speed=0.3,
    yaw_speed=0.6,
)


def make_joylo_real(
    *, publish_odom_tf: bool = True, enable_wrist_color: bool = True, webxr: bool = False
) -> Blueprint:
    """Compose hardware control with an explicit owner for odometry and wrist video."""
    declaration = _declaration.model_copy(
        update={"publish_odom_tf": publish_odom_tf, "enable_wrist_color": enable_wrist_color}
    )
    return _control_blueprint(
        R1ProConnection.blueprint(**declaration.model_dump()),
        R1ProConnection,
        _declaration,
        joylo=_profile.model_dump(),
        motion_limits=motion_groups(),
        velocity_limits={**dict.fromkeys(ARMS, 1.0), **dict.fromkeys(TORSO, 0.5)},
        webxr=webxr,
    ).global_config(viewer="none", transport="zenoh", n_workers=3)


r1pro_joylo_real = autoconnect(make_joylo_real())
