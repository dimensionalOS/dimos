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

"""OpenArm/Joy-Con inputs and transport contract, independent of the simulator."""

import math
from typing import Any

from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.stream import Transport, TransportSpec
from dimos.core.transport import ZenohTransport
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.Joy import Joy
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.protocol.pubsub.impl.zenohpubsub import QOS_CONTROL, Topic
from dimos.teleop.joycon.joycon_module import JoyConTeleopModule
from dimos.teleop.leader.calibration import DEFAULT_LEADER_MOTOR_NAMES, LEADER_SIDES
from dimos.teleop.leader.mapping import JointMapping, SideMapping
from dimos.teleop.leader.module import LeaderTeleopModule

# Preserve calibrated leader radians for absolute joint tracking. Follower limits are applied
# by R1ProJoylo and again by the coordinator/connection, not in this reader.
OPENARM_R1PRO_MAPPINGS = {
    side: SideMapping(
        joints=tuple(
            JointMapping(
                motor=motor, joint=f"r1pro/{side}_arm_joint{i}", lower=-math.tau, upper=math.tau
            )
            for i, motor in enumerate(DEFAULT_LEADER_MOTOR_NAMES, 1)
        )
    )
    for side in LEADER_SIDES
}

_transports: dict[tuple[str, type], TransportSpec | Transport[Any]] = {
    (name, kind): ZenohTransport.spec(Topic(f"dimos/r1pro/joylo/{name}", kind, qos=QOS_CONTROL))
    for name, kind in (
        ("leader_readings", JointState),
        ("joycon_sample", Joy),
        ("leader_joint_command", JointState),
        ("robot_feedback", JointState),
        ("r1pro_position_command", JointState),
        ("r1pro_velocity_command", JointState),
        ("left_gripper_command", Float32),
        ("right_gripper_command", Float32),
        ("cmd_vel", Twist),
    )
}


def _operator_blueprint(left_only: bool = False) -> Blueprint:
    return autoconnect(
        LeaderTeleopModule.blueprint(
            mappings=OPENARM_R1PRO_MAPPINGS,
            tick_period_s=0.01,
            # JoyLo must see the current, unsmoothed readings to detect jumps
            # after input loss. The generic reader's rejection
            # against its last accepted pose otherwise makes recovery impossible.
            max_joint_jump_radians=None,
            command_filter_cutoff_hz=None,
            enabled_sides=("left",) if left_only else None,
        ).remappings(
            [
                (LeaderTeleopModule, "leader_joint_command", "leader_readings"),
                (LeaderTeleopModule, "teleop_buttons", "unused_leader_buttons"),
                (LeaderTeleopModule, "left_gripper_command", "unused_leader_left_gripper"),
                (LeaderTeleopModule, "right_gripper_command", "unused_leader_right_gripper"),
            ]
        ),
        JoyConTeleopModule.blueprint(publish_legacy_commands=False).remappings(
            [
                (JoyConTeleopModule, "teleop_buttons", "unused_joycon_buttons"),
                (JoyConTeleopModule, "left_gripper_command", "unused_joycon_left_gripper"),
                (JoyConTeleopModule, "right_gripper_command", "unused_joycon_right_gripper"),
                (JoyConTeleopModule, "cmd_vel", "unused_joycon_velocity"),
            ]
        ),
    ).transports(_transports)


teleop_openarm_r1pro_joylo = _operator_blueprint().global_config(
    viewer="none", transport="zenoh", zenoh_mode="client", serve_coordinator_rpc=False
)
