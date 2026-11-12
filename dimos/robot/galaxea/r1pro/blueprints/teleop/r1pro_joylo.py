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

"""OpenArm + Joy-Con input and R1 Pro sim2 bindings; see docs/development/r1pro-joylo.md."""

import math
from typing import Any

from dimos.control.connection.connection_module import ConnectionConfig
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.stream import Transport, TransportSpec
from dimos.core.transport import ZenohTransport
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.Joy import Joy
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.protocol.pubsub.impl.zenohpubsub import QOS_LATEST_WINS, Topic
from dimos.robot.galaxea.r1pro.connection import R1ProConnectionConfig
from dimos.robot.galaxea.r1pro.joylo_coordinator import R1ProJoyloCoordinator, joylo_tasks
from dimos.robot.galaxea.r1pro.joylo_module import R1ProJoylo
from dimos.robot.galaxea.r1pro.sim.config import CONTROL, JOINTS, MODEL, SCENE
from dimos.robot.galaxea.r1pro.sim.connection import R1ProSimConnection
from dimos.robot.galaxea.r1pro.sim.control import R1ProSimControl
from dimos.robot.galaxea.r1pro.sim.joylo_input import JoyloSimInput
from dimos.robot.urdf_manager import UrdfManager
from dimos.sim2.module import SimModule
from dimos.sim2.physics.mujoco import MujocoBackend
from dimos.sim2.sensors.encoder.module import EncoderConfig, SimJointEncoderModule
from dimos.teleop.joycon.joycon_module import JoyConTeleopModule
from dimos.teleop.leader.calibration import DEFAULT_LEADER_MOTOR_NAMES, LEADER_SIDES
from dimos.teleop.leader.mapping import JointMapping, SideMapping
from dimos.teleop.leader.module import LeaderTeleopModule

# Preserve leader radians before relative alignment. Follower limits are applied
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
    (name, kind): ZenohTransport.spec(Topic(f"dimos/r1pro/joylo/{name}", kind, qos=QOS_LATEST_WINS))
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
            # and re-align after input loss. The generic reader's rejection
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


_declaration = R1ProConnectionConfig()


def _robot_blueprint(left_only: bool = False) -> Blueprint:
    return (
        autoconnect(
            UrdfManager.blueprint(models={"r1pro": MODEL}),
            SimModule.blueprint(
                scene=SCENE,
                backend=MujocoBackend,
                dt_ns=5_000_000,
                controllers=(CONTROL,),
                joint_ids=JOINTS,
                observation_hz=200,
                startup_timeout_s=120,
            ),
            SimJointEncoderModule.blueprint(sensor=EncoderConfig(JOINTS, hz=50)),
            R1ProSimControl.blueprint(coordinator_torso=True),
            R1ProSimConnection.blueprint(),
            R1ProJoylo.blueprint(
                enabled_sides=("left",) if left_only else ("left", "right"), arm_only=left_only
            ),
            R1ProJoyloCoordinator.blueprint(
                instance_name="ControlCoordinator",
                connections=[
                    ConnectionConfig(
                        source=_declaration.source,
                        model=_declaration.model,
                        joint_commands=_declaration.joint_commands,
                        deadman_timeout_s=_declaration.deadman_timeout_s,
                    )
                ],
                tasks=joylo_tasks(),
            ),
        )
        .remappings(
            [
                (SimJointEncoderModule, "joint_state", "encoder"),
                (R1ProSimControl, "joint_state", "encoder"),
                (UrdfManager, "coordinator_joint_state", "encoder"),
                (R1ProSimConnection, "joint_state", "robot_feedback"),
                (R1ProJoyloCoordinator, "r1pro_joint_state", "robot_feedback"),
                (R1ProSimConnection, "position_command", "r1pro_position_command"),
                (R1ProSimConnection, "velocity_command", "r1pro_velocity_command"),
                (R1ProSimControl, "cmd_vel", "servo_velocity"),
                (R1ProSimControl, "left_gripper_command", "servo_left_gripper"),
                (R1ProSimControl, "right_gripper_command", "servo_right_gripper"),
                (R1ProJoyloCoordinator, "twist_command", "cmd_vel"),
            ]
        )
        .transports(_transports)
    )


# macOS uses HIDAPI; Linux keeps its existing hid-nintendo input backend.
_operator = _operator_blueprint()
_robot = _robot_blueprint()
r1pro_joylo_sim2 = _robot.global_config(viewer="none", transport="zenoh", n_workers=6)
r1pro_joylo_sim2_mock = (
    autoconnect(_robot, JoyloSimInput.blueprint())
    .transports(_transports)
    .global_config(viewer="none", transport="zenoh", n_workers=6)
)
r1pro_openarm_joylo_sim2 = autoconnect(_robot, _operator).global_config(
    viewer="none", transport="zenoh", n_workers=6
)
teleop_openarm_r1pro_joylo = _operator.global_config(
    viewer="none", transport="zenoh", zenoh_mode="client", serve_coordinator_rpc=False
)

# One attached leader and one left Joy-Con; all other robot motion stays held.
r1pro_openarm_joylo_left_sim2 = autoconnect(
    _robot_blueprint(left_only=True), _operator_blueprint(left_only=True)
).global_config(viewer="none", transport="zenoh", n_workers=6)
