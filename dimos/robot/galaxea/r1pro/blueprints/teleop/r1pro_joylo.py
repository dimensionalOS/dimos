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
from pathlib import Path

from dimos.control.connection.connection_module import ConnectionConfig
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.robot.assets.model import UsdModel
from dimos.robot.galaxea.r1pro.blueprints.teleop.r1pro_joylo_inputs import (
    _operator_blueprint,
    _transports,
)
from dimos.robot.galaxea.r1pro.connection import R1ProConnectionConfig
from dimos.robot.galaxea.r1pro.joylo_coordinator import R1ProJoyloCoordinator, joylo_tasks
from dimos.robot.galaxea.r1pro.joylo_module import R1ProJoylo
from dimos.robot.galaxea.r1pro.motion import motion_groups
from dimos.robot.galaxea.r1pro.sim.config import (
    ARM_RATE,
    CONTROL_MODEL,
    JOINTS,
    MODEL,
    SCENE,
    TORSO_RATE,
    UPPER_JOINTS,
    UPPER_RATES,
)
from dimos.robot.galaxea.r1pro.sim.connection import R1ProSimConnection
from dimos.robot.galaxea.r1pro.sim.control import R1ProSimControl
from dimos.robot.galaxea.r1pro.sim.joylo_input import JoyloSimInput
from dimos.robot.galaxea.r1pro.sim.model import TORSO_LINK_LENGTHS
from dimos.robot.usd_manager import UsdManager
from dimos.sim2.module import SimModule
from dimos.sim2.physics.mujoco import MujocoBackend
from dimos.sim2.sensors.encoder.module import EncoderConfig, SimJointEncoderModule

_declaration = R1ProConnectionConfig(model=CONTROL_MODEL)


def _robot_blueprint(
    left_only: bool = False,
    *,
    scene: Path = SCENE,
) -> Blueprint:
    return (
        autoconnect(
            UsdManager.blueprint(model=UsdModel.from_file(scene)),
            SimModule.blueprint(
                scene_manager=UsdManager,
                backend=MujocoBackend,
                dt_ns=2_000_000,
                joint_ids=JOINTS,
                observation_hz=250,
            ),
            SimJointEncoderModule.blueprint(sensor=EncoderConfig(JOINTS, hz=250)),
            R1ProSimControl.blueprint(coordinator_torso=True),
            R1ProSimConnection.blueprint(model=CONTROL_MODEL),
            R1ProJoylo.blueprint(
                enabled_sides=("left",) if left_only else ("left", "right"),
                arm_only=left_only,
                model=MODEL,
                leader_angle_period=math.tau,
                torso_link_lengths=TORSO_LINK_LENGTHS,
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
                motion_limits=motion_groups(
                    arm_rate=ARM_RATE,
                    arm_ceiling=ARM_RATE,
                    torso_rate=TORSO_RATE,
                    torso_ceiling=TORSO_RATE,
                    base_speed=0.1,
                    yaw_speed=0.25,
                ),
                tasks=joylo_tasks(dict(zip(UPPER_JOINTS, UPPER_RATES, strict=True))),
            ),
        )
        .remappings(
            [
                (SimJointEncoderModule, "joint_state", "encoder"),
                (R1ProSimControl, "joint_state", "encoder"),
                (UsdManager, "coordinator_joint_state", "encoder"),
                (R1ProSimConnection, "joint_state", "robot_feedback"),
                (R1ProJoyloCoordinator, "r1pro_joint_state", "robot_feedback"),
                (R1ProSimConnection, "position_command", "r1pro_position_command"),
                (R1ProSimConnection, "velocity_command", "r1pro_velocity_command"),
                (R1ProSimControl, "cmd_vel", "servo_velocity"),
                (R1ProSimControl, "left_gripper_command", "servo_left_gripper"),
                (R1ProSimControl, "right_gripper_command", "servo_right_gripper"),
                (R1ProJoyloCoordinator, "twist_command", "cmd_vel"),
                (R1ProJoylo, "cmd_vel", "joylo_cmd_vel"),
                (R1ProJoyloCoordinator, "joylo_twist_command", "joylo_cmd_vel"),
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

# One attached leader and one left Joy-Con; all other robot motion stays held.
r1pro_openarm_joylo_left_sim2 = autoconnect(
    _robot_blueprint(left_only=True), _operator_blueprint(left_only=True)
).global_config(viewer="none", transport="zenoh", n_workers=6)
