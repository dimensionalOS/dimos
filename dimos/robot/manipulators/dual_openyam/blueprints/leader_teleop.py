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

"""Two YAM leader arms drive the dual OpenYAM through the leader teleop module."""

from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.global_config import global_config
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.visualization.viser.config import ViserVisualizationConfig
from dimos.robot.manipulators.damiao.damiao_blueprint import arm_connection, arm_remaps
from dimos.robot.manipulators.dual_openyam.config import (
    DUAL_OPENYAM_HOME_PER_ARM,
    dual_openyam_arm_joints,
    dual_openyam_control_model,
    dual_openyam_model_config,
)
from dimos.robot.manipulators.dual_openyam.connection import DualOpenYamConnection
from dimos.teleop.leader.calibration import LEADER_SIDES, LeaderSide
from dimos.teleop.leader.coordinator import LeaderCoordinator, leader_gripper_tasks, leader_task
from dimos.teleop.leader.mapping import GripperMapping, JointMapping, SideMapping
from dimos.teleop.leader.module import LeaderTeleopModule

# dual_openyam.urdf position limits for joint1 to joint6; both arms share them.
DUAL_OPENYAM_JOINT_LIMITS = (
    (-2.618, 3.1416),
    (0.0, 3.6652),
    (0.0, 3.1416),
    (-1.693, 1.5708),
    (-1.5708, 1.5708),
    (-2.0944, 2.0944),
)
# Leader servo IDs 1 to 6 follow the joints, 7 is the gripper trigger.
DUAL_OPENYAM_LEADER_MOTORS = tuple(f"joint_{index}" for index in range(1, 7))
DUAL_OPENYAM_LEADER_TRIGGER_MOTOR = "joint_7"
# Trigger angle from released to fully squeezed on the Dimensional leaders.
DUAL_OPENYAM_LEADER_TRIGGER_TRAVEL_RADIANS = 1.5
# Following a human arm needs more than the URDF's 2 rad/s.
DUAL_OPENYAM_LEADER_VELOCITY_RAD_S = 3.0
DUAL_OPENYAM_LEADER_WRIST_VELOCITY_RAD_S = 4.0
DUAL_OPENYAM_WRIST_JOINTS = ("joint4", "joint5", "joint6")
# Measured on the OpenArm leaders: a 6 Hz low-pass halves wrist roughness for
# about 10 ms of lag, and a target every coordinator tick halves each ramp.
DUAL_OPENYAM_LEADER_FILTER_CUTOFF_HZ = 6.0
DUAL_OPENYAM_LEADER_TICK_PERIOD_S = 0.01
DUAL_OPENYAM_GRIPPER_TASKS = {
    "left_manipulator": "left_arm_gripper",
    "right_manipulator": "right_arm_gripper",
}


def _side_mapping(side: LeaderSide) -> SideMapping:
    return SideMapping(
        joints=tuple(
            JointMapping(motor=motor, joint=f"arm/{joint}", lower=lower, upper=upper)
            for motor, joint, (lower, upper) in zip(
                DUAL_OPENYAM_LEADER_MOTORS,
                dual_openyam_arm_joints(side),
                DUAL_OPENYAM_JOINT_LIMITS,
                strict=True,
            )
        ),
        gripper=GripperMapping(
            motor=DUAL_OPENYAM_LEADER_TRIGGER_MOTOR,
            travel_radians=DUAL_OPENYAM_LEADER_TRIGGER_TRAVEL_RADIANS,
        ),
    )


MAPPINGS = {side: _side_mapping(side) for side in LEADER_SIDES}
DUAL_OPENYAM_LEADER_JOINTS = [
    joint for side in LEADER_SIDES for joint in MAPPINGS[side].joint_names()
]


def dual_openyam_leader_velocity_limits() -> dict[str, float]:
    return {
        joint: (
            DUAL_OPENYAM_LEADER_WRIST_VELOCITY_RAD_S
            if joint.endswith(DUAL_OPENYAM_WRIST_JOINTS)
            else DUAL_OPENYAM_LEADER_VELOCITY_RAD_S
        )
        for joint in DUAL_OPENYAM_LEADER_JOINTS
    }


def dual_openyam_leader_module() -> Blueprint:
    """The leader module; --simulation reads the mock leaders parked at the home pose."""
    if global_config.simulation:
        return LeaderTeleopModule.blueprint(
            mappings=MAPPINGS,
            command_filter_cutoff_hz=DUAL_OPENYAM_LEADER_FILTER_CUTOFF_HZ,
            tick_period_s=DUAL_OPENYAM_LEADER_TICK_PERIOD_S,
            reader="mock",
            mock_positions=dict(
                zip(DUAL_OPENYAM_LEADER_MOTORS, DUAL_OPENYAM_HOME_PER_ARM, strict=True)
            ),
        )
    return LeaderTeleopModule.blueprint(
        mappings=MAPPINGS,
        command_filter_cutoff_hz=DUAL_OPENYAM_LEADER_FILTER_CUTOFF_HZ,
        tick_period_s=DUAL_OPENYAM_LEADER_TICK_PERIOD_S,
    )


def dual_openyam_leader_control() -> Blueprint:
    """The trajectory task fed by the leader port, plus a jaw per side, over the two-bus connection."""
    return autoconnect(
        LeaderCoordinator.blueprint(
            instance_name="ControlCoordinator",
            model=dual_openyam_control_model(robot="arm"),
            tasks=[
                leader_task(
                    DUAL_OPENYAM_LEADER_JOINTS,
                    velocity_limits=dual_openyam_leader_velocity_limits(),
                ),
                *leader_gripper_tasks(DualOpenYamConnection.robot.grippers),
            ],
            zero_stiffness_joints=[],
        ),
        arm_connection(DualOpenYamConnection),
    ).remappings(arm_remaps(DualOpenYamConnection, LeaderCoordinator))


# The leader ports given select the sides; --simulation mocks both the leaders and the CAN.
dual_openyam_leader_teleop = autoconnect(
    dual_openyam_leader_module(),
    dual_openyam_leader_control(),
    ManipulationModule.blueprint(
        gripper_tasks=DUAL_OPENYAM_GRIPPER_TASKS,
        model=dual_openyam_model_config(robot="arm"),
        visualization=ViserVisualizationConfig(),
    ),
)
