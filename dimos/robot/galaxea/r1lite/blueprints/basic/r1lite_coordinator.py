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

"""R1 Lite ControlCoordinator: R1LiteConnection Module + transport_lcm bridges.

Mirrors ``r1pro_coordinator.py`` (which mirrors the G1 wiring). Whole-body
upper body (16-DOF: 4 torso read-only + 2x6 arms) goes through
``TransportWholeBodyAdapter``; the holonomic swerve chassis (3-DOF) goes
through ``TransportTwistAdapter``. No R1Lite-specific adapter code.

Robot-side prerequisites (scripts/r1lite_test/RUNBOOK.md):
    robot cold-booted with e-stop released; R1LITEBody.d stack up with the
    GELLO teleop session killed; RC ON with all switches in position 1
    (mode 5 = software may drive the chassis).

Usage:
    dimos run r1lite-coordinator                # no viewer
    dimos --viewer rerun run r1lite-coordinator # composes the rerun bridge
    dimos --viewer rerun-web run r1lite-coordinator
"""

from __future__ import annotations

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

_chassis_joints = make_twist_base_joints("chassis")


def _r1lite_rerun_blueprint() -> Any:
    """Two-tab viewer: main (wrists + head-left + 3D) and stereo/depth grid.

    Entity paths assume the bridge's default ``entity_prefix="world"`` — so
    LCM topic ``/r1lite/head_left_color`` lands at ``world/r1lite/head_left_color``.
    """
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


_rerun_config = {
    "blueprint": _r1lite_rerun_blueprint,
    "pubsubs": [LCM()],
}


_r1lite_base = (
    autoconnect(
        R1LiteConnection.blueprint(),
        ControlCoordinator.blueprint(
            tick_rate=100,
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
            tasks=[
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
            ],
        ),
    )
    # Module's `cmd_vel`/`odom` collide with the chassis transport adapter's
    # /{hw}/cmd_vel + /{hw}/odom topics — rename so the adapter (hw_id="chassis")
    # owns the canonical names.
    .remappings(
        [
            (R1LiteConnection, "cmd_vel", "chassis_cmd_vel"),
            (R1LiteConnection, "odom", "chassis_odom"),
        ]
    )
    .transports(
        {
            # WholeBody bridge (hw_id="r1lite"). TransportWholeBodyAdapter
            # subscribes /{hw}/imu — only one IMU goes there.
            ("motor_states", JointState): LCMTransport("/r1lite/motor_states", JointState),
            ("imu_chassis", Imu): LCMTransport("/r1lite/imu", Imu),
            ("imu_torso", Imu): LCMTransport("/r1lite/imu_torso", Imu),
            ("motor_command", MotorCommandArray): LCMTransport(
                "/r1lite/motor_command", MotorCommandArray
            ),
            # Twist bridge (hw_id="chassis").
            ("chassis_cmd_vel", Twist): LCMTransport("/chassis/cmd_vel", Twist),
            ("chassis_odom", PoseStamped): LCMTransport("/chassis/odom", PoseStamped),
            # Public Twist bus on /cmd_vel — `cmd_vel` covers any module's Out
            # (KeyboardTeleop, phone teleop, etc.); `twist_command` is the
            # ControlCoordinator's matching In. Both pinned to the same LCM
            # topic so any Twist publisher drives the chassis.
            ("cmd_vel", Twist): LCMTransport("/cmd_vel", Twist),
            ("twist_command", Twist): LCMTransport("/cmd_vel", Twist),
            # Grippers (0-100 native units; no R1 Pro equivalent).
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
            # Sensor pass-throughs — downstream consumers (rerun bridge,
            # perception modules, etc.) attach to these topics directly.
            ("head_left_color", Image): LCMTransport("/r1lite/head_left_color", Image),
            ("head_right_color", Image): LCMTransport("/r1lite/head_right_color", Image),
            ("wrist_left_color", Image): LCMTransport("/r1lite/wrist_left_color", Image),
            ("wrist_left_depth", Image): LCMTransport("/r1lite/wrist_left_depth", Image),
            ("wrist_right_color", Image): LCMTransport("/r1lite/wrist_right_color", Image),
            ("wrist_right_depth", Image): LCMTransport("/r1lite/wrist_right_depth", Image),
            # ControlCoordinator outs.
            ("joint_state", JointState): LCMTransport(
                "/coordinator/joint_state", JointState
            ),
            ("joint_command", JointState): LCMTransport("/r1lite/joint_command", JointState),
        }
    )
)


# Compose the rerun bridge when the user asks for it via `dimos --viewer rerun`.
# Same gating as the Go2/G1/R1 Pro blueprints: "rerun", "rerun-web",
# "rerun-connect" all light up; headless otherwise.
if global_config.viewer.startswith("rerun"):
    from dimos.visualization.rerun.bridge import RerunBridgeModule, _resolve_viewer_mode

    r1lite_coordinator = autoconnect(
        _r1lite_base,
        RerunBridgeModule.blueprint(viewer_mode=_resolve_viewer_mode(), **_rerun_config),
    )
else:
    r1lite_coordinator = _r1lite_base


__all__ = ["r1lite_coordinator"]
