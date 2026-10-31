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

"""Coordinator composition for Damiao connection modules."""

from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.robot.assets.model import RobotModel
from dimos.robot.manipulators.common.damiao_connection import DamiaoConnection
from dimos.robot.urdf_manager import UrdfManager


class DamiaoCoordinator(ControlCoordinator):
    """One connection named arm, including a bimanual robot on two CAN buses."""

    arm_position_command: Out[JointState]
    arm_joint_state: In[JointState]
    left_gripper_command: In[Float32]
    right_gripper_command: In[Float32]


def damiao_connection_control(
    connection: type[DamiaoConnection],
    model: RobotModel,
    joints: list[str],
    grippers: tuple[str, ...],
) -> Blueprint:
    """Route trajectories and normalized grippers through a standard connection."""
    names = [f"arm/{name}" for name in (*joints, *grippers)]
    return autoconnect(
        DamiaoCoordinator.blueprint(
            instance_name="ControlCoordinator",
            model=model,
            tasks=[
                joint_trajectory_task(names),
                *[
                    TaskConfig(
                        name=f"arm_{name}",
                        type="gripper",
                        joint_names=[f"arm/{name}"],
                        priority=20,
                        stream_bind={"gripper_command": f"{side}_gripper_command"},
                    )
                    for name, side in zip(
                        grippers,
                        ("left", "right") if len(grippers) == 2 else ("right",),
                        strict=True,
                    )
                ],
            ],
        ),
        connection.blueprint(source="arm"),
        UrdfManager.blueprint(model=model),
    ).remappings(
        [
            (connection, "position_command", "arm_position_command"),
            (DamiaoCoordinator, "arm_joint_state", "joint_state"),
        ]
    )
