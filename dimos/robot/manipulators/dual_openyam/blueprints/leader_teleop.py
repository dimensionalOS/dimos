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

The leaders' Feetech connection publishes the arms' joint targets and jaw
openings; the coordinator's trajectory and gripper tasks follow them.
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
    arm_declaration,
    arm_remaps,
)
from dimos.robot.manipulators.dual_openyam.config import (
    DUAL_OPENYAM_HOME_PER_ARM,
    DUAL_OPENYAM_LEADERS,
    DUAL_OPENYAM_SIDES,
    dual_openyam_arm_joints,
    dual_openyam_model_config,
)
from dimos.robot.manipulators.dual_openyam.connection import (
    DualOpenYamConnection,
    DualOpenYamLeaderConnection,
)

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
# Every arm joint the leaders drive, as the coordinator names it.
DUAL_OPENYAM_LEADER_JOINTS = [
    f"arm/{motor.joint}" for bus in DUAL_OPENYAM_LEADERS.buses for motor in bus.motors
]


class DualOpenYamLeaderCoordinator(DamiaoCoordinator):
    """The dual OpenYAM coordinator, plus the port the leader arms' joint targets arrive on."""

    # The 100 Hz tick loop must not share a Python process with cameras, Viser
    # and recorders: on a Jetson that cost a quarter of its ticks and the arm
    # moved in steps.
    dedicated_worker: ClassVar[bool] = True

    leader_joint_command: In[JointState]


def dual_openyam_leader_velocity_limits() -> dict[str, float]:
    return {
        joint: (
            DUAL_OPENYAM_LEADER_WRIST_VELOCITY_RAD_S
            if joint.endswith(DUAL_OPENYAM_WRIST_JOINTS)
            else DUAL_OPENYAM_LEADER_VELOCITY_RAD_S
        )
        for joint in DUAL_OPENYAM_LEADER_JOINTS
    }


# The follower's jaws by side; each is named <side>_gripper on the robot.
_JAWS = {jaw.name.removesuffix("_gripper"): jaw for jaw in DualOpenYamConnection.robot.grippers}


def dual_openyam_leaders() -> Blueprint:
    """The leader arms' connection; --simulation without ports parks mock leaders at home."""
    mock_positions = (
        {
            joint: home
            for side in DUAL_OPENYAM_SIDES
            for joint, home in zip(
                dual_openyam_arm_joints(side), DUAL_OPENYAM_HOME_PER_ARM, strict=True
            )
        }
        if global_config.simulation
        else None
    )
    return DualOpenYamLeaderConnection.blueprint(
        state_rate_hz=DUAL_OPENYAM_LEADER_RATE_HZ,
        filter_cutoff_hz=DUAL_OPENYAM_LEADER_FILTER_CUTOFF_HZ,
        mock_positions=mock_positions,
    ).remappings([(DualOpenYamLeaderConnection, "joint_state", "leader_joint_command")])


def dual_openyam_leader_control() -> Blueprint:
    """The trajectory task fed by the leader port, plus a jaw per side, over the two-bus connection."""
    return autoconnect(
        DualOpenYamLeaderCoordinator.blueprint(
            instance_name="ControlCoordinator",
            connections=[arm_declaration(DualOpenYamConnection)],
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
                    for side in DUAL_OPENYAM_SIDES
                ],
            ],
            zero_stiffness_joints=[],
        ),
        arm_connection(DualOpenYamConnection),
    ).remappings(arm_remaps(DualOpenYamConnection, DualOpenYamLeaderCoordinator))


# The leader serial ports given select the sides; --simulation mocks the CAN, and
# the leaders too when no serial port is given.
dual_openyam_leader_teleop = autoconnect(
    dual_openyam_leaders(),
    dual_openyam_leader_control(),
    ManipulationModule.blueprint(
        gripper_tasks=DUAL_OPENYAM_GRIPPER_TASKS,
        model=dual_openyam_model_config(robot="arm"),
        visualization=ViserVisualizationConfig(),
    ),
)
