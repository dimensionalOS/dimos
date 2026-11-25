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

"""JoyLo ports on CC's ordinary connection-backed coordinator."""

from dataclasses import replace
from typing import ClassVar

from dimos_lcm.control_msgs import DynamicJointState

from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.robot.galaxea.r1pro.config import R1PRO_BODY_AXES
from dimos.robot.galaxea.r1pro.joylo import JAWS, TORSO, UPPER


class JoyloInputs(ControlCoordinator):
    dedicated_worker: ClassVar[bool] = True
    joylo_twist_command: In[Twist]
    leader_joint_command: In[JointState]
    left_gripper_command: In[Float32]
    right_gripper_command: In[Float32]
    r1pro_joint_state: In[JointState]


class R1ProJoyloCoordinator(JoyloInputs):
    r1pro_position_command: Out[JointState]
    r1pro_velocity_command: Out[JointState]


class R1ProJoyloMotorCoordinator(JoyloInputs):
    r1pro_motor_command: Out[DynamicJointState]


def joylo_tasks(
    velocity_limits: dict[str, float] | None = None, *, arm_impedance: TaskConfig | None = None
) -> list[TaskConfig]:
    """Assign leader arm targets to the selected follower and torso targets to trajectory control."""
    trajectory_joints = UPPER if arm_impedance is None else TORSO
    rates = velocity_limits or dict.fromkeys(UPPER, 0.3)
    return [
        replace(
            joint_trajectory_task(
                trajectory_joints,
                velocity_limits={joint: rates[joint] for joint in trajectory_joints},
            ),
            stream_bind={"joint_command": "leader_joint_command"},
        ),
        *([arm_impedance] if arm_impedance else []),
        TaskConfig(
            name="vel_chassis",
            type="twist_velocity",
            joint_names=list(R1PRO_BODY_AXES),
            params={"axes": ["linear.x", "linear.y", "angular.z"], "timeout": 0.2},
            stream_bind={"twist_command": "joylo_twist_command"},
        ),
        TaskConfig(
            name="desktop_chassis",
            type="twist_velocity",
            joint_names=list(R1PRO_BODY_AXES),
            priority=20,
            params={
                "axes": ["linear.x", "linear.y", "angular.z"],
                "timeout": 0.2,
                "zero_on_timeout": False,
            },
        ),
        *(
            TaskConfig(
                name=f"{side}_gripper",
                type="gripper",
                joint_names=[joint],
                stream_bind={"gripper_command": f"{side}_gripper_command"},
            )
            for side, joint in zip(("left", "right"), JAWS, strict=True)
        ),
    ]
