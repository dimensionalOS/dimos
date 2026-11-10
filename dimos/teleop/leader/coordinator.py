# Copyright 2025-2026 Dimensional Inc.
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

"""The coordinator side of leader teleop: a port for the leader and a task that outranks policy.

The leader's targets reach their own ``trajectory`` task instance through
``stream_bind``, at a priority above the trajectory task a policy or the planner
uses, so the leader wins every joint it claims while it publishes and hands the
joints back when it stops, as a WebXR grip does.
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping

from dimos.control.coordinator import TaskConfig
from dimos.core.stream import In
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.damiao.damiao_blueprint import DamiaoCoordinator
from dimos.teleop.leader.calibration import LeaderSide

LEADER_JOINT_COMMAND = "leader_joint_command"
LEADER_TASK_NAME = "leader"
# Above the trajectory task's 10, the lane a policy or the planner streams on.
LEADER_TASK_PRIORITY = 20
GRIPPER_TASK_PRIORITY = 20


class LeaderCoordinator(DamiaoCoordinator):
    """A Damiao deployment whose coordinator also reads the leader's joint targets."""

    leader_joint_command: In[JointState]


def leader_task(
    joint_names: Iterable[str],
    *,
    velocity_limits: Mapping[str, float] | None = None,
    priority: int = LEADER_TASK_PRIORITY,
    start_position_tolerance: float = 0.05,
) -> TaskConfig:
    """The trajectory task the leader drives, bound to its own port."""
    params: dict[str, object] = {"start_position_tolerance": start_position_tolerance}
    if velocity_limits:
        params["velocity_limits"] = dict(velocity_limits)
    return TaskConfig(
        name=LEADER_TASK_NAME,
        type="trajectory",
        joint_names=list(joint_names),
        priority=priority,
        params=params,
        stream_bind={"joint_command": LEADER_JOINT_COMMAND},
    )


def leader_gripper_tasks(
    gripper_joint_by_side: Mapping[LeaderSide, str], *, priority: int = GRIPPER_TASK_PRIORITY
) -> list[TaskConfig]:
    """One gripper task per leader side, each reading that side's gripper command."""
    return [
        TaskConfig(
            name=f"{side}_arm_gripper",
            type="gripper",
            joint_names=[joint],
            priority=priority,
            stream_bind={"gripper_command": f"{side}_gripper_command"},
        )
        for side, joint in gripper_joint_by_side.items()
    ]
