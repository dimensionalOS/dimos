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

"""Two YAM leader arms drive the dual OpenYAM.

The leaders' Feetech connection publishes their joint angles, the leader teleop
module maps them onto the arms, and the coordinator's trajectory and gripper
tasks follow the result.
"""

from dataclasses import replace
from typing import ClassVar

from dimos.control.coordinator import TaskConfig
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.global_config import global_config
from dimos.core.stream import In
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.visualization.viser.config import ViserVisualizationConfig
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.damiao.damiao_blueprint import (
    DamiaoCoordinator,
    arm_connection,
    arm_remaps,
)
from dimos.robot.manipulators.dual_openyam.config import (
    DUAL_OPENYAM_HOME_PER_ARM,
    DUAL_OPENYAM_LEADER_MOTORS,
    dual_openyam_arm_joints,
    dual_openyam_control_model,
    dual_openyam_model_config,
)
from dimos.robot.manipulators.dual_openyam.connection import (
    DualOpenYamConnection,
    DualOpenYamLeaderConnection,
)
from dimos.robot.manipulators.feetech.robot import joint_name
from dimos.teleop.leader.mapping import (
    LEADER_SIDES,
    GripperMapping,
    JointMapping,
    LeaderSide,
    SideMapping,
)
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
# The leader connection publishes every joint as "leader/<side>_<motor>".
DUAL_OPENYAM_LEADER_SOURCE = "leader"
# Trigger angle from released to fully squeezed on the Dimensional leaders.
DUAL_OPENYAM_LEADER_TRIGGER_TRAVEL_RADIANS = 1.5
# Following a human arm needs more than the URDF's 2 rad/s.
DUAL_OPENYAM_LEADER_VELOCITY_RAD_S = 3.0
DUAL_OPENYAM_LEADER_WRIST_VELOCITY_RAD_S = 4.0
DUAL_OPENYAM_WRIST_JOINTS = ("joint4", "joint5", "joint6")
# Measured on the OpenArm leaders: a 6 Hz low-pass halves wrist roughness for
# about 10 ms of lag, and a reading every coordinator tick halves each ramp.
DUAL_OPENYAM_LEADER_FILTER_CUTOFF_HZ = 6.0
DUAL_OPENYAM_LEADER_RATE_HZ = 100.0
DUAL_OPENYAM_GRIPPER_TASKS = {
    "left_manipulator": "left_arm_gripper",
    "right_manipulator": "right_arm_gripper",
}


class DualOpenYamLeaderCoordinator(DamiaoCoordinator):
    """The dual OpenYAM coordinator, plus the port the leader arms' joint targets arrive on."""

    # The 100 Hz tick loop must not share a Python process with cameras, Viser
    # and recorders: on a Jetson that cost a quarter of its ticks and the arm
    # moved in steps.
    dedicated_worker: ClassVar[bool] = True

    leader_joint_command: In[JointState]


def _leader_joint(side: LeaderSide, motor: str) -> str:
    return f"{DUAL_OPENYAM_LEADER_SOURCE}/{joint_name(side, motor)}"


def _side_mapping(side: LeaderSide) -> SideMapping:
    *arm_motors, trigger = DUAL_OPENYAM_LEADER_MOTORS
    return SideMapping(
        joints=tuple(
            JointMapping(
                leader_joint=_leader_joint(side, motor),
                joint=f"arm/{joint}",
                lower=lower,
                upper=upper,
            )
            for motor, joint, (lower, upper) in zip(
                arm_motors,
                dual_openyam_arm_joints(side),
                DUAL_OPENYAM_JOINT_LIMITS,
                strict=True,
            )
        ),
        gripper=GripperMapping(
            leader_joint=_leader_joint(side, trigger),
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


# The follower's jaws by leader side; each is named <side>_gripper on the robot.
_JAWS = {jaw.name.removesuffix("_gripper"): jaw for jaw in DualOpenYamConnection.robot.grippers}


def dual_openyam_leaders() -> Blueprint:
    """The leader arms' connection; --simulation parks mock leaders at the home pose."""
    mock_positions = (
        {
            joint_name(side, motor): home
            for side in LEADER_SIDES
            # The arm joints at home; the trigger reads 0, released.
            for motor, home in zip(
                DUAL_OPENYAM_LEADER_MOTORS[:-1], DUAL_OPENYAM_HOME_PER_ARM, strict=True
            )
        }
        if global_config.simulation
        else None
    )
    return DualOpenYamLeaderConnection.blueprint(
        source=DUAL_OPENYAM_LEADER_SOURCE,
        state_rate_hz=DUAL_OPENYAM_LEADER_RATE_HZ,
        mock_positions=mock_positions,
    ).remappings([(DualOpenYamLeaderConnection, "joint_state", "leader_joint_state")])


def dual_openyam_leader_module() -> Blueprint:
    """The mapping from the leaders' readings to the arms' targets."""
    return LeaderTeleopModule.blueprint(
        mappings=MAPPINGS,
        command_filter_cutoff_hz=DUAL_OPENYAM_LEADER_FILTER_CUTOFF_HZ,
        reading_period_s=1.0 / DUAL_OPENYAM_LEADER_RATE_HZ,
    )


def dual_openyam_leader_control() -> Blueprint:
    """The trajectory task fed by the leader port, plus a jaw per side, over the two-bus connection."""
    return autoconnect(
        DualOpenYamLeaderCoordinator.blueprint(
            instance_name="ControlCoordinator",
            model=dual_openyam_control_model(robot="arm"),
            tasks=[
                # The trajectory task follows the leader's targets instead of joint_command.
                replace(
                    joint_trajectory_task(
                        DUAL_OPENYAM_LEADER_JOINTS,
                        velocity_limits=dual_openyam_leader_velocity_limits(),
                    ),
                    stream_bind={"joint_command": "leader_joint_command"},
                ),
                *[
                    TaskConfig(
                        name=f"{side}_arm_gripper",
                        type="gripper",
                        joint_names=[f"arm/{_JAWS[side].joint}"],
                        priority=20,
                        params={"open_at": _JAWS[side].open_at},
                        stream_bind={"gripper_command": f"{side}_gripper_command"},
                    )
                    for side in LEADER_SIDES
                ],
            ],
            zero_stiffness_joints=[],
        ),
        arm_connection(DualOpenYamConnection),
    ).remappings(arm_remaps(DualOpenYamConnection, DualOpenYamLeaderCoordinator))


# The leader serial ports given select the sides; --simulation mocks the leaders and the CAN.
dual_openyam_leader_teleop = autoconnect(
    dual_openyam_leaders(),
    dual_openyam_leader_module(),
    dual_openyam_leader_control(),
    ManipulationModule.blueprint(
        gripper_tasks=DUAL_OPENYAM_GRIPPER_TASKS,
        model=dual_openyam_model_config(robot="arm"),
        visualization=ViserVisualizationConfig(),
    ),
)
