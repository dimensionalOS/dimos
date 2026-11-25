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

from dimos.core.coordination.blueprints import Blueprint, MetaBlueprint, autoconnect
from dimos.robot.galaxea.r1pro.blueprints.basic.r1pro_coordinator import _wrist_cameras
from dimos.robot.galaxea.r1pro.blueprints.teleop.r1pro_joylo_inputs import (
    _camera_transports,
    _control_blueprint,
)
from dimos.robot.galaxea.r1pro.connection import R1ProConnection, R1ProConnectionConfig
from dimos.robot.galaxea.r1pro.joylo import ARMS, TORSO, JoyloConfig
from dimos.robot.galaxea.r1pro.motion import motion_groups

_declaration = R1ProConnectionConfig(
    boot_vendor_stack=False,
    tracking_speed=1.0,
    publish_odom_tf=True,
)
_profile = JoyloConfig(
    leader_angle_period=math.tau,
    torso_link_lengths=(0.4, 0.3),
    base_speed=0.3,
    yaw_speed=0.6,
)


def make_joylo_real(
    *,
    publish_odom_tf: bool = True,
    enable_wrist_color: bool = True,
    webxr: bool = False,
    arm_rate: float = 0.75,
    arm_ceiling: float = 1.0,
    tracking_speed: float = 1.0,
) -> MetaBlueprint:
    """Compose hardware control with an explicit owner for odometry and wrist video.

    Args:
        arm_rate: Fastest each arm joint follows its target at start, in rad/s.
        arm_ceiling: Highest arm_rate the operator may set while running, in rad/s.
        tracking_speed: Speed cap sent to the robot's own joint tracker, in rad/s.
    """
    declaration = _declaration.model_copy(
        update={"publish_odom_tf": publish_odom_tf, "tracking_speed": tracking_speed}
    )
    connection: Blueprint | MetaBlueprint = R1ProConnection.blueprint(**declaration.model_dump())
    if enable_wrist_color:
        # The connection no longer carries wrist video: each D405's colour comes
        # straight off V4L2 (640x480 JPEG at 30 fps), on the same stream names.
        connection = autoconnect(
            connection,
            autoconnect(*_wrist_cameras())
            .remappings(
                [
                    ("wrist_left", "jpeg_out", "wrist_left_color"),
                    ("wrist_right", "jpeg_out", "wrist_right_color"),
                ]
            )
            .transports(_camera_transports),
        )
    return _control_blueprint(
        connection,
        R1ProConnection,
        _declaration,
        joylo=_profile.model_dump(),
        motion_limits=motion_groups(arm_rate=arm_rate, arm_ceiling=arm_ceiling),
        velocity_limits={**dict.fromkeys(ARMS, 1.0), **dict.fromkeys(TORSO, 0.5)},
        webxr=webxr,
    ).global_config(viewer="none", transport="zenoh", n_workers=3)


r1pro_joylo_real = autoconnect(make_joylo_real())
