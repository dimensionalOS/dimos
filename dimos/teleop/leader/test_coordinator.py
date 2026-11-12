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

from typing import get_args, get_origin, get_type_hints

import pytest

from dimos.control.tasks.trajectory_task.trajectory_task import JOINT_TRAJECTORY_TASK_NAME
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.damiao.damiao_blueprint import DamiaoCoordinator
from dimos.robot.manipulators.damiao.robot import DamiaoGripper, DamiaoMotor
from dimos.teleop.leader.coordinator import (
    LEADER_JOINT_COMMAND,
    LEADER_TASK_PRIORITY,
    LeaderCoordinator,
    leader_gripper_tasks,
    leader_task,
)
from dimos.teleop.leader.module import LeaderTeleopModule

JOINTS = ["arm/left_joint1", "arm/left_joint2", "arm/right_joint1"]


def test_coordinator_reads_the_port_the_leader_module_publishes() -> None:
    assert issubclass(LeaderCoordinator, DamiaoCoordinator)
    assert LeaderCoordinator.dedicated_worker is True
    reader = get_type_hints(LeaderCoordinator)[LEADER_JOINT_COMMAND]
    writer = get_type_hints(LeaderTeleopModule)[LEADER_JOINT_COMMAND]
    assert (get_origin(reader), get_args(reader)) == (In, (JointState,))
    assert (get_origin(writer), get_args(writer)) == (Out, (JointState,))


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


def _jaw(name: str, joint: str, open_at: str) -> DamiaoGripper:
    return DamiaoGripper(
        name=name,
        joint=joint,
        open_at=open_at,  # type: ignore[arg-type]
        motor=DamiaoMotor(name, "DM4310", 0x08, 0x18),
        opening="decreasing_position",
        current=0.15,
    )


def test_gripper_tasks_bind_each_jaw_to_its_side_and_keep_its_open_end() -> None:
    tasks = leader_gripper_tasks(
        [
            _jaw("left_gripper", "left_finger_joint1", "upper"),
            _jaw("right_gripper", "right_gripper", "lower"),
        ]
    )
    assert [
        (t.name, t.type, t.joint_names, t.priority, t.params, t.stream_bind) for t in tasks
    ] == [
        (
            "left_arm_gripper",
            "gripper",
            ["arm/left_finger_joint1"],
            20,
            {"open_at": "upper"},
            {"gripper_command": "left_gripper_command"},
        ),
        (
            "right_arm_gripper",
            "gripper",
            ["arm/right_gripper"],
            20,
            {"open_at": "lower"},
            {"gripper_command": "right_gripper_command"},
        ),
    ]
    with pytest.raises(ValueError, match="not named <side>_gripper"):
        leader_gripper_tasks([_jaw("gripper", "joint7", "upper")])
