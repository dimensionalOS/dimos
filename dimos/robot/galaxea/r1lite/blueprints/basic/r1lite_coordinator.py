#!/usr/bin/env python3
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

"""R1 Lite ControlCoordinator: R1LiteConnection module with transport_lcm bridges.

The upper body goes through TransportWholeBodyAdapter and the swerve chassis
through TransportTwistAdapter. Chassis software control needs RC mode 5.

    dimos run r1lite-coordinator
    dimos --viewer rerun run r1lite-coordinator
"""

from __future__ import annotations

from collections.abc import Sequence
from typing import Any

from dimos.control.components import HardwareComponent, HardwareType, make_twist_base_joints
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.protocol.pubsub.impl.lcmpubsub import LCM
from dimos.robot.galaxea.r1lite.connection import R1LITE_UPPER_BODY_JOINTS, R1LiteConnection
from dimos.visualization.vis_module import vis_module

_chassis_joints = make_twist_base_joints("chassis")


def _r1lite_rerun_blueprint() -> Any:
    import rerun as rr
    import rerun.blueprint as rrb

    main_tab = rrb.Horizontal(
        rrb.Vertical(
            rrb.Spatial2DView(origin="world/r1lite/wrist_left_color", name="Left wrist"),
            rrb.Spatial2DView(origin="world/r1lite/wrist_right_color", name="Right wrist"),
            rrb.Spatial2DView(origin="world/r1lite/head_left_color", name="Head (L)"),
        ),
        rrb.Spatial3DView(
            origin="world",
            name="3D",
            background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
            line_grid=rrb.LineGrid3D(
                plane=rr.components.Plane3D.XY.with_distance(0.5),
            ),
        ),
        column_shares=[1, 2],
        name="Main",
    )

    stereo_tab = rrb.Grid(
        rrb.Spatial2DView(origin="world/r1lite/head_left_color", name="Head left"),
        rrb.Spatial2DView(origin="world/r1lite/head_right_color", name="Head right"),
        rrb.Spatial2DView(origin="world/r1lite/wrist_left_depth", name="L wrist depth"),
        rrb.Spatial2DView(origin="world/r1lite/wrist_right_depth", name="R wrist depth"),
        grid_columns=2,
        name="Stereo + depth",
    )

    return rrb.Blueprint(
        rrb.Tabs(main_tab, stereo_tab),
        rrb.TimePanel(state="hidden"),
        rrb.SelectionPanel(state="hidden"),
    )


# Per-entity frame-rate caps: the connection publishes six uncompressed image
# streams; unthrottled a viewer falls behind live and consumes GBs.
_CAMERA_MAX_HZ = {
    "world/r1lite/head_left_color": 10.0,
    "world/r1lite/head_right_color": 2.0,
    "world/r1lite/wrist_left_color": 5.0,
    "world/r1lite/wrist_right_color": 5.0,
    "world/r1lite/wrist_left_depth": 2.0,
    "world/r1lite/wrist_right_depth": 2.0,
}

_rerun_config = {
    "blueprint": _r1lite_rerun_blueprint,
    "pubsubs": [LCM()],
    "max_hz": _CAMERA_MAX_HZ,
    "memory_limit": "1GB",
}


def r1lite_standard_tasks() -> list[TaskConfig]:
    """Servo holder for the upper body plus the chassis velocity task."""
    return [
        TaskConfig(
            name="servo_r1lite",
            type="servo",
            joint_names=R1LITE_UPPER_BODY_JOINTS,
            priority=10,
        ),
        TaskConfig(
            name="vel_chassis",
            type="velocity",
            joint_names=_chassis_joints,
            priority=10,
        ),
    ]


def r1lite_control_base(extra_tasks: Sequence[TaskConfig] = ()) -> Any:
    """R1LiteConnection wired to the ControlCoordinator over transport_lcm.

    extra_tasks are appended to the standard servo and chassis velocity tasks;
    the quest teleop blueprint uses this to add its per-arm IK tasks.
    """
    return (
        autoconnect(
            R1LiteConnection.blueprint(),
            ControlCoordinator.blueprint(
                hardware=[
                    HardwareComponent(
                        hardware_id="r1lite",
                        hardware_type=HardwareType.WHOLE_BODY,
                        joints=R1LITE_UPPER_BODY_JOINTS,
                        adapter_type="transport_lcm",
                    ),
                    HardwareComponent(
                        hardware_id="chassis",
                        hardware_type=HardwareType.BASE,
                        joints=_chassis_joints,
                        adapter_type="transport_lcm",
                    ),
                ],
                tasks=[*r1lite_standard_tasks(), *extra_tasks],
            ),
        )
        # Rename so the chassis adapter owns the canonical cmd_vel/odom names.
        .remappings(
            [
                (R1LiteConnection, "cmd_vel", "chassis_cmd_vel"),
                (R1LiteConnection, "odom", "chassis_odom"),
            ]
        )
        .transports(
            {
                ("motor_states", JointState): LCMTransport("/r1lite/motor_states", JointState),
                ("imu_chassis", Imu): LCMTransport("/r1lite/imu", Imu),
                ("imu_torso", Imu): LCMTransport("/r1lite/imu_torso", Imu),
                ("motor_command", MotorCommandArray): LCMTransport(
                    "/r1lite/motor_command", MotorCommandArray
                ),
                ("chassis_cmd_vel", Twist): LCMTransport("/chassis/cmd_vel", Twist),
                ("chassis_odom", PoseStamped): LCMTransport("/chassis/odom", PoseStamped),
                ("cmd_vel", Twist): LCMTransport("/cmd_vel", Twist),
                ("twist_command", Twist): LCMTransport("/cmd_vel", Twist),
                ("gripper_left_command", JointState): LCMTransport(
                    "/r1lite/gripper_left_command", JointState
                ),
                ("gripper_right_command", JointState): LCMTransport(
                    "/r1lite/gripper_right_command", JointState
                ),
                ("gripper_left_state", JointState): LCMTransport(
                    "/r1lite/gripper_left_state", JointState
                ),
                ("gripper_right_state", JointState): LCMTransport(
                    "/r1lite/gripper_right_state", JointState
                ),
                ("head_left_color", Image): LCMTransport("/r1lite/head_left_color", Image),
                ("head_right_color", Image): LCMTransport("/r1lite/head_right_color", Image),
                ("wrist_left_color", Image): LCMTransport("/r1lite/wrist_left_color", Image),
                ("wrist_left_depth", Image): LCMTransport("/r1lite/wrist_left_depth", Image),
                ("wrist_right_color", Image): LCMTransport("/r1lite/wrist_right_color", Image),
                ("wrist_right_depth", Image): LCMTransport("/r1lite/wrist_right_depth", Image),
                ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
                ("joint_command", JointState): LCMTransport("/r1lite/joint_command", JointState),
            }
        )
    )


def r1lite_vis() -> Any:
    """Viewer module with the R1 Lite rerun layout and camera rate caps."""
    return vis_module(
        viewer_backend=global_config.viewer,
        rerun_config=_rerun_config,
    )


r1lite_coordinator = autoconnect(
    r1lite_control_base(),
    r1lite_vis(),
)
