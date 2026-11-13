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

from typing import Any

from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.stream import Transport, TransportSpec
from dimos.core.transport import ZenohTransport
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.Joy import Joy
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.protocol.pubsub.impl.zenohpubsub import QOS_CONTROL, Topic
from dimos.robot.galaxea.r1pro.leader import R1ProLeaderConnection
from dimos.teleop.joycon.joycon_module import JoyConTeleopModule

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
        R1ProLeaderConnection.blueprint(
            enabled_sides=("left",) if left_only else ("left", "right"),
        ).remappings([(R1ProLeaderConnection, "joint_state", "leader_readings")]),
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
