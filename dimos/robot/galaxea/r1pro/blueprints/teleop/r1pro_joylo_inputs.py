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

"""The parts of R1 teleoperation that the hardware and the simulation share.

A robot blueprint supplies only what differs: its connection to the robot and its
cameras. The operator devices, the headset page, JoyLo, the coordinator with its
tasks, the stream names and the transports are the same here for both, so a
frontend or a recorder meets one contract.
"""

from typing import Any

from dimos.control.connection.connection_module import ConnectionConfig
from dimos.control.motion_limits import MotionGroup
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.module import ModuleBase
from dimos.core.stream import Transport, TransportSpec
from dimos.core.transport import ZenohTransport
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.CompressedImage import CompressedImage
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.Joy import Joy
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.protocol.pubsub.impl.zenohpubsub import QOS_CONTROL, QOS_LATEST_WINS, Topic
from dimos.robot.galaxea.r1pro.joylo_coordinator import R1ProJoyloCoordinator, joylo_tasks
from dimos.robot.galaxea.r1pro.joylo_module import R1ProJoylo
from dimos.robot.galaxea.r1pro.leader import R1ProLeaderConnection
from dimos.robot.galaxea.r1pro.teleop_coordinator import R1ProTeleopCoordinator, shared_teleop_tasks
from dimos.robot.galaxea.r1pro.webxr import R1ProWebXR, R1ProWebXRJoystick
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

# Compressed color goes out newest-first, on the same keys from either robot.
_camera_transports: dict[tuple[str, type], TransportSpec | Transport[Any]] = {
    (name, CompressedImage): ZenohTransport.spec(
        f"dimos/{name}", CompressedImage, qos=QOS_LATEST_WINS
    )
    for name in (
        "head_left_color",
        "head_right_color",
        "wrist_left_color",
        "wrist_right_color",
        "color_image",
        "left_wrist",
        "right_wrist",
    )
}


def _operator_blueprint(left_only: bool = False, *, require_leaders: bool = True) -> Blueprint:
    return autoconnect(
        R1ProLeaderConnection.blueprint(
            enabled_sides=("left",) if left_only else ("left", "right"),
            # Hardware keeps the module's own default: no serial port is an error.
            **({} if require_leaders else {"required": False}),
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


def _webxr_blueprint() -> Blueprint:
    """CC's headset page, on the coordinator's WebXR ports; it runs beside the operator."""
    return R1ProWebXR.blueprint(body_tracking_mode="optional", input_timeout_s=0.3).remappings(
        [
            (R1ProWebXR, "left_controller_output", "left_cartesian_command"),
            (R1ProWebXR, "right_controller_output", "right_cartesian_command"),
            (R1ProWebXR, "left_gripper_command", "webxr_left_gripper"),
            (R1ProWebXR, "right_gripper_command", "webxr_right_gripper"),
        ]
    )


def _webxr_joystick_blueprint() -> Blueprint:
    """The headset sticks as body velocity; it runs beside the coordinator."""
    return R1ProWebXRJoystick.blueprint().remappings(
        [(R1ProWebXRJoystick, "base_twist", "webxr_twist_command")]
    )


def _control_blueprint(
    connection: Blueprint,
    connection_module: type[ModuleBase],
    declaration: ConnectionConfig,
    *,
    joylo: dict[str, Any],
    motion_limits: dict[str, MotionGroup],
    velocity_limits: dict[str, float],
    webxr: bool = True,
) -> Blueprint:
    """JoyLo and the one coordinator on a robot connection, hardware or simulated.

    Args:
        connection: The connection module's blueprint, with whatever stands behind
            it (in simulation: the scene, the physics and the servo).
        connection_module: The module in it that takes the coordinator's commands.
        declaration: That connection's contract: source, model and command joints.
        joylo: R1ProJoylo's settings.
        motion_limits: The coordinator's named speed groups.
        velocity_limits: Per-joint trajectory speeds, in rad/s.
        webxr: Add the headset tasks and the leader/headset handover; without it
            the coordinator takes the leaders only.
    """
    coordinator = R1ProTeleopCoordinator if webxr else R1ProJoyloCoordinator
    return (
        autoconnect(
            connection,
            R1ProJoylo.blueprint(**joylo),
            coordinator.blueprint(
                instance_name="ControlCoordinator",
                connections=[
                    ConnectionConfig(
                        source=declaration.source,
                        model=declaration.model,
                        joint_commands=declaration.joint_commands,
                        deadman_timeout_s=declaration.deadman_timeout_s,
                    )
                ],
                motion_limits=motion_limits,
                tasks=shared_teleop_tasks(velocity_limits)
                if webxr
                else joylo_tasks(velocity_limits),
            ),
        )
        .remappings(
            [
                (connection_module, "joint_state", "robot_feedback"),
                (coordinator, "r1pro_joint_state", "robot_feedback"),
                (connection_module, "position_command", "r1pro_position_command"),
                (connection_module, "velocity_command", "r1pro_velocity_command"),
                (coordinator, "twist_command", "cmd_vel"),
                (R1ProJoylo, "cmd_vel", "joylo_cmd_vel"),
                (coordinator, "joylo_twist_command", "joylo_cmd_vel"),
            ]
        )
        .transports(_transports)
    )


teleop_openarm_r1pro_joylo = _operator_blueprint().global_config(
    viewer="none", transport="zenoh", zenoh_mode="client", serve_coordinator_rpc=False
)
