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

"""The coordinator side of leader teleop: a port for the leader and the task that reads it.

A coordinator has one trajectory task, ``joint_trajectory``; ``stream_bind``
points it at the leader's port, so the leader's targets stream into the lane
the planner and policies also use. Each guarded message is a jog that claims the
joints it names and releases them shortly after the leader stops.
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping
from typing import ClassVar

from dimos.control.coordinator import TaskConfig
from dimos.control.tasks.trajectory_task.trajectory_task import JOINT_TRAJECTORY_TASK_NAME
from dimos.core.stream import In
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.damiao.damiao_blueprint import DamiaoCoordinator
from dimos.robot.manipulators.damiao.robot import DamiaoGripper
from dimos.teleop.leader.calibration import LEADER_SIDES

LEADER_JOINT_COMMAND = "leader_joint_command"
# The coordinator allows one trajectory task and addresses it by this name.
LEADER_TASK_NAME = JOINT_TRAJECTORY_TASK_NAME
LEADER_TASK_PRIORITY = 10
GRIPPER_TASK_PRIORITY = 20


class LeaderCoordinator(DamiaoCoordinator):
    """A Damiao deployment whose coordinator also reads the leader's joint targets."""

    # The 100 Hz tick loop must not share a Python process with cameras, Viser
    # and recorders: on a Jetson that cost a quarter of its ticks and the arm
    # moved in steps.
    dedicated_worker: ClassVar[bool] = True

    leader_joint_command: In[JointState]


def leader_task(
    joint_names: Iterable[str],
    *,
    velocity_limits: Mapping[str, float] | None = None,
    priority: int = LEADER_TASK_PRIORITY,
    start_position_tolerance: float = 0.05,
) -> TaskConfig:
    """The coordinator's trajectory task, reading the leader's port instead of joint_command."""
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
    grippers: Iterable[DamiaoGripper], *, priority: int = GRIPPER_TASK_PRIORITY
) -> list[TaskConfig]:
    """One gripper task per jaw, each reading its side's opening from the leader.

    A jaw named ``<side>_gripper`` reads ``<side>_gripper_command``; ``open_at``
    comes from the robot so a jaw open at its lower end is not driven inverted.
    """
    tasks: list[TaskConfig] = []
    for gripper in grippers:
        side = gripper.name.removesuffix("_gripper")
        if side == gripper.name or side not in LEADER_SIDES:
            raise ValueError(f"gripper {gripper.name!r} is not named <side>_gripper")
        tasks.append(
            TaskConfig(
                name=f"{side}_arm_gripper",
                type="gripper",
                joint_names=[f"arm/{gripper.joint}"],
                priority=priority,
                params={"open_at": gripper.open_at},
                stream_bind={"gripper_command": f"{side}_gripper_command"},
            )
        )
    return tasks
