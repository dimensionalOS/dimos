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

from dimos.control.tasks.trajectory_task.trajectory_task import JOINT_TRAJECTORY_TASK_NAME
from dimos.teleop.leader.tasks import (
    LEADER_JOINT_COMMAND,
    LEADER_TASK_PRIORITY,
    leader_gripper_task,
    leader_task,
)

JOINTS = ["arm/left_joint1", "arm/left_joint2", "arm/right_joint1"]


def test_leader_task_is_the_trajectory_task_reading_the_leader_port() -> None:
    task = leader_task(JOINTS, velocity_limits={"arm/left_joint1": 3.0})
    assert task.type == "trajectory"
    assert task.name == JOINT_TRAJECTORY_TASK_NAME
    assert task.priority == LEADER_TASK_PRIORITY == 10
    assert task.joint_names == JOINTS
    assert task.stream_bind == {"joint_command": LEADER_JOINT_COMMAND}
    assert task.params == {
        "start_position_tolerance": 0.05,
        "velocity_limits": {"arm/left_joint1": 3.0},
    }
    assert "velocity_limits" not in leader_task(JOINTS).params


def test_gripper_task_binds_the_jaw_to_its_side_and_keeps_its_open_end() -> None:
    task = leader_gripper_task("right", "arm/right_gripper", open_at="lower")
    assert (task.name, task.type, task.joint_names, task.priority) == (
        "right_arm_gripper",
        "gripper",
        ["arm/right_gripper"],
        20,
    )
    assert task.params == {"open_at": "lower"}
    assert task.stream_bind == {"gripper_command": "right_gripper_command"}
