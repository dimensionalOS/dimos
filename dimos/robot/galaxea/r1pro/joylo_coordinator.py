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

from typing import ClassVar

from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.robot.galaxea.r1pro.config import R1PRO_BODY_AXES
from dimos.robot.galaxea.r1pro.joylo import ARMS, JAWS, TORSO, UPPER
from dimos.teleop.leader.coordinator import leader_task


class R1ProJoyloCoordinator(ControlCoordinator):
    dedicated_worker: ClassVar[bool] = True
    joylo_twist_command: In[Twist]
    leader_joint_command: In[JointState]
    left_gripper_command: In[Float32]
    right_gripper_command: In[Float32]
    r1pro_position_command: Out[JointState]
    r1pro_velocity_command: Out[JointState]
    r1pro_joint_state: In[JointState]

    _tracking_limits: dict[str, float] | None = None

    @rpc
    def set_tracking_limits(self, arm_rate: float, torso_rate: float) -> None:
        """Apply the shared tracking rates to leader, browser and RPC joint targets."""
        limits = {**dict.fromkeys(ARMS, arm_rate), **dict.fromkeys(TORSO, torso_rate)}
        with self._task_lock:
            if self._trajectory_task is not None:
                self._trajectory_task.set_velocity_limits(limits)
            self._tracking_limits = limits

    def _setup_from_config(self) -> None:
        super()._setup_from_config()
        with self._task_lock:
            if self._tracking_limits is not None and self._trajectory_task is not None:
                self._trajectory_task.set_velocity_limits(self._tracking_limits)


def joylo_tasks(velocity_limits: dict[str, float] | None = None) -> list[TaskConfig]:
    # One canonical trajectory owner: the profile composes the arms and torso.
    return [
        leader_task(UPPER, velocity_limits=velocity_limits or dict.fromkeys(UPPER, 0.3)),
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
