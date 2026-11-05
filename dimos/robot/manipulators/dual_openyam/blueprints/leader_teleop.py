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
"""Leader-follower teleop of the dual OpenYAM from two YAM leader arms."""

from dimos.control.coordinator import TaskConfig
from dimos.control.tasks.trajectory_task.trajectory_task import JOINT_TRAJECTORY_TASK_NAME
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.visualization.viser.config import ViserVisualizationConfig
from dimos.robot.manipulators.damiao.damiao_blueprint import connect_damiao
from dimos.robot.manipulators.dual_openyam.config import DUAL_OPENYAM_SIDES
from dimos.robot.manipulators.dual_openyam.connection import DualOpenYamConnection
from dimos.teleop.yam_leader.teleop_module import YamLeaderTeleopModule

# Leader-follower needs human arm speeds; the URDF's 2 rad/s felt slow on the rig.
DUAL_OPENYAM_LEADER_VELOCITY_RAD_S = 3.0
DUAL_OPENYAM_LEADER_WRIST_VELOCITY_RAD_S = 4.0
DUAL_OPENYAM_WRIST_JOINTS = ("joint4", "joint5", "joint6")
# Named, not scouted; the operator machine overrides it with --zenoh-connect.
DUAL_OPENYAM_ROUTER = "tcp/127.0.0.1:7447"

_dual_openyam = DualOpenYamConnection.planning_model("arm")


def dual_openyam_leader_velocity_limits() -> dict[str, float]:
    return {
        joint: (
            DUAL_OPENYAM_LEADER_WRIST_VELOCITY_RAD_S
            if joint.endswith(DUAL_OPENYAM_WRIST_JOINTS)
            else DUAL_OPENYAM_LEADER_VELOCITY_RAD_S
        )
        for joint in _dual_openyam.joint_names
    }


def dual_openyam_leader_tasks() -> list[TaskConfig]:
    """Streamed joint targets for both arms plus one gripper task per side."""
    return [
        TaskConfig(
            name=JOINT_TRAJECTORY_TASK_NAME,
            type="trajectory",
            joint_names=list(_dual_openyam.joint_names),
            priority=10,
            params={
                "start_position_tolerance": 0.05,
                "velocity_limits": dual_openyam_leader_velocity_limits(),
            },
        ),
        *[
            TaskConfig(
                name=f"{side}_arm_gripper",
                type="gripper",
                joint_names=[f"arm/{DualOpenYamConnection.gripper_joints[f'{side}_gripper']}"],
                priority=20,
                stream_bind={"gripper_command": f"{side}_gripper_command"},
            )
            for side in DUAL_OPENYAM_SIDES
        ],
    ]


def _follower() -> Blueprint:
    return autoconnect(
        connect_damiao(DualOpenYamConnection, tasks=dual_openyam_leader_tasks()),
        ManipulationModule.blueprint(model=_dual_openyam, visualization=ViserVisualizationConfig()),
    )


# The leader ports given select the sides; --simulation swaps the CAN connection for a mock.
teleop_dual_openyam_leader = autoconnect(YamLeaderTeleopModule.blueprint(), _follower())

# Leaders on the operator machine (--robot-ip <robot>); the robot owns the Coordinator name.
teleop_dual_openyam_leader_operator = YamLeaderTeleopModule.blueprint().global_config(
    zenoh_mode="client", serve_coordinator_rpc=False
)

teleop_dual_openyam_leader_follower = _follower().global_config(zenoh_connect=DUAL_OPENYAM_ROUTER)
