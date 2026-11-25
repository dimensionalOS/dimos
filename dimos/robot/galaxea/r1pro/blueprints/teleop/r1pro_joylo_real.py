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

from dimos.control.contract.limits import PD_INTERFACES
from dimos.core.coordination.blueprints import Blueprint, MetaBlueprint, autoconnect
from dimos.robot.galaxea.r1pro.blueprints.basic.r1pro_coordinator import _wrist_cameras
from dimos.robot.galaxea.r1pro.blueprints.teleop.r1pro_joylo_inputs import (
    _camera_transports,
    _control_blueprint,
)
from dimos.robot.galaxea.r1pro.connection import (
    R1ProConnection,
    R1ProConnectionConfig,
    _R1ProConnection,
)
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
    impedance: bool = False,
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
    arm_task = None
    connection_type: type[_R1ProConnection] = R1ProConnection
    if impedance:
        from dimos.robot.galaxea.r1pro.impedance import joylo_impedance_task
        from dimos.robot.galaxea.r1pro.wim_impedance import R1ProWimImpedanceConnection

        arm_task = joylo_impedance_task(arm_rate)
        declaration = declaration.model_copy(
            update={
                "joint_commands": {
                    **declaration.joint_commands,
                    **dict.fromkeys(ARMS, PD_INTERFACES),
                },
                "max_stiffness": arm_task.params["stiffness"],
                "max_damping": arm_task.params["damping"],
            }
        )
        connection_type = R1ProWimImpedanceConnection
    connection: Blueprint | MetaBlueprint = connection_type.blueprint(**declaration.model_dump())
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
        connection_type,
        declaration,
        arm_impedance=arm_task,
        joylo={**_profile.model_dump(), "release_arm_impedance_on_pause": impedance},
        motion_limits=motion_groups(arm_rate=arm_rate, arm_ceiling=arm_ceiling),
        velocity_limits={**dict.fromkeys(ARMS, 1.0), **dict.fromkeys(TORSO, 0.5)},
        webxr=webxr,
    ).global_config(viewer="none", transport="zenoh", n_workers=3)


r1pro_joylo_real = autoconnect(make_joylo_real())
