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

"""Booster K1 ControlCoordinator — basic blueprint + coordinator via LCM transport adapter.

Like unitree_go2_coordinator, a holonomic twist base (vx, vy, wz) is driven through
ControlCoordinator's velocity task and the `transport_lcm` adapter, which republishes
the base velocity on /booster_k1/cmd_vel for the connection to consume. The K1 has no
odometry, so (unlike the Go2) there is no odom wiring.

This builds on `booster_k1_basic` (not bare K1Connection), so it carries the rerun
viewer + camera. Both viewer-side WASD surfaces (the dimos-viewer hovering overlay via
RerunWebSocketServer, and the Dashboard via WebsocketVisModule) are remapped from their
dead-end `tele_cmd_vel` onto `/cmd_vel`, which the coordinator's twist_command consumes.
So you can drive the bot through the coordinator straight from the rerun window, and
watch the camera, in one launch. Layer KeyboardTeleop on top for a pygame window too —
see booster_k1_coordinator_keyboard_teleop.

Control path:
    hovering WASD / Dashboard / pygame -> /cmd_vel -> Coordinator.twist_command
        -> velocity task -> transport_lcm adapter -> /booster_k1/cmd_vel -> K1Connection.move()

Usage:
    dimos --robot-ip <ip> --viewer rerun --rerun-open native run booster-k1-coordinator
"""

from __future__ import annotations

from dimos.control.components import HardwareComponent, HardwareType, make_twist_base_joints
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.booster.k1.blueprints.basic.booster_k1_basic import booster_k1_basic
from dimos.robot.booster.k1.connection import K1Connection
from dimos.visualization.rerun.websocket_server import RerunWebSocketServer
from dimos.web.websocket_vis.websocket_vis_module import WebsocketVisModule

_k1_joints = make_twist_base_joints("booster_k1")

booster_k1_coordinator = (
    autoconnect(
        booster_k1_basic,
        ControlCoordinator.blueprint(
            hardware=[
                HardwareComponent(
                    hardware_id="booster_k1",
                    hardware_type=HardwareType.BASE,
                    joints=_k1_joints,
                    adapter_type="transport_lcm",
                ),
            ],
            tasks=[
                TaskConfig(
                    name="vel_booster_k1",
                    type="velocity",
                    joint_names=_k1_joints,
                    priority=10,
                ),
            ],
        ),
    )
    .remappings(
        [
            # Free up the bare `cmd_vel` name for the teleop/twist publishers; the
            # connection now listens on /booster_k1/cmd_vel (what the adapter emits).
            (K1Connection, "cmd_vel", "k1_cmd_vel"),
            # Route the viewer-side WASD surfaces onto /cmd_vel so they feed the
            # coordinator's twist_command instead of a dead-end topic.
            (WebsocketVisModule, "tele_cmd_vel", "cmd_vel"),
            (RerunWebSocketServer, "tele_cmd_vel", "cmd_vel"),
        ]
    )
    .transports(
        {
            ("cmd_vel", Twist): LCMTransport("/cmd_vel", Twist),
            ("twist_command", Twist): LCMTransport("/cmd_vel", Twist),
            ("k1_cmd_vel", Twist): LCMTransport("/booster_k1/cmd_vel", Twist),
            ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
        }
    )
    .global_config(n_workers=6, robot_model="booster_k1")
)

__all__ = ["booster_k1_coordinator"]
