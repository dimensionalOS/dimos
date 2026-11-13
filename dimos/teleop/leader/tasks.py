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

"""Coordinator tasks that make a follower arm copy the leader arms.

The robot's blueprint gives its coordinator an ``In[JointState]`` port named
``leader_joint_command``; ``leader_task`` points the coordinator's trajectory task
at that port, and ``leader_gripper_task`` drives one follower jaw from one
leader's trigger. Nothing here depends on the follower's motor brand.
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping

from dimos.control.coordinator import TaskConfig
from dimos.control.tasks.gripper_task.gripper_task import OpenAt
from dimos.control.tasks.trajectory_task.trajectory_task import JOINT_TRAJECTORY_TASK_NAME
from dimos.teleop.leader.calibration import LeaderSide

# The coordinator port the leader's joint targets arrive on.
LEADER_JOINT_COMMAND = "leader_joint_command"
# The coordinator allows one trajectory task and addresses it by this name.
LEADER_TASK_NAME = JOINT_TRAJECTORY_TASK_NAME
LEADER_TASK_PRIORITY = 10
GRIPPER_TASK_PRIORITY = 20


def leader_task(
    joint_names: Iterable[str],
    *,
    velocity_limits: Mapping[str, float] | None = None,
    priority: int = LEADER_TASK_PRIORITY,
    start_position_tolerance: float = 0.05,
) -> TaskConfig:
    """Make the follower's joints follow the leader's joint targets.

    Args:
        joint_names: Follower joints the leader drives, with their connection
            prefix, e.g. "arm/left_joint1".
        velocity_limits: Top speed per joint, in rad/s. None keeps the
            trajectory task's default of 1 rad/s.
        priority: Task priority; higher wins a joint when two tasks want it.
        start_position_tolerance: How far a joint may be from the leader's
            first target before the move is refused, in radians.
    """
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


def leader_gripper_task(
    side: LeaderSide,
    joint: str,
    *,
    open_at: OpenAt = "upper",
    priority: int = GRIPPER_TASK_PRIORITY,
) -> TaskConfig:
    """Open and close one follower jaw with the trigger on the same side's leader.

    Args:
        side: Which leader's trigger drives the jaw, "left" or "right".
        joint: The follower's finger joint, with its connection prefix, e.g.
            "arm/left_gripper".
        open_at: Which end of the joint's range is fully open, "lower" or
            "upper". The wrong end inverts the jaw: it closes when the trigger
            is released.
        priority: Task priority; higher wins the jaw when two tasks want it.
    """
    return TaskConfig(
        name=f"{side}_arm_gripper",
        type="gripper",
        joint_names=[joint],
        priority=priority,
        params={"open_at": open_at},
        stream_bind={"gripper_command": f"{side}_gripper_command"},
    )
