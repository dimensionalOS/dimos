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

import threading

import pytest

from dimos.control.task import (
    BaseControlTask,
    ControlMode,
    CoordinatorState,
    JointCommandOutput,
    JointStateSnapshot,
    ResourceClaim,
)
from dimos.control.tasks.trajectory_task.trajectory_task import (
    JointTrajectoryTask,
    JointTrajectoryTaskConfig,
    TrajectoryExecutionStatus,
)
from dimos.control.tick_loop import TickLoop
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint


@pytest.mark.parametrize("single_point", [False, True])
def test_execution_after_teleop_starts_from_measured_position(mocker, single_point):
    task = JointTrajectoryTask(JointTrajectoryTaskConfig(joint_names=["joint"], priority=20))
    teleop = mocker.Mock(spec=BaseControlTask)
    teleop.name = "teleop"
    teleop.is_active.return_value = False
    teleop.claim.return_value = ResourceClaim(joints=frozenset({"joint"}), priority=10)
    teleop.compute.return_value = JointCommandOutput(
        joint_names=["joint"], positions=[-1.0], mode=ControlMode.SERVO_POSITION
    )
    loop = TickLoop(
        100, {}, threading.Lock(), {task.name: task, "teleop": teleop}, threading.Lock(), {}
    )
    measured = JointStateSnapshot(joint_positions={"joint": 0.0})
    mocker.patch.object(loop, "_read_all_hardware", return_value=(measured, {}))
    route = mocker.spy(loop, "_route_to_hardware")
    clock = mocker.patch("dimos.control.tick_loop.time.perf_counter", return_value=0.1)
    first = JointTrajectory(joint_names=["joint"], points=[TrajectoryPoint(positions=[0.1])])
    assert task.execute(first, {}).status is TrajectoryExecutionStatus.ACCEPTED
    loop._tick()
    assert route.call_args.args[0]["joint"][0] == pytest.approx(0.1)
    assert not task.is_active()

    teleop.is_active.return_value = True
    clock.return_value = 0.2
    loop._tick()
    assert route.call_args.args[0]["joint"] == (-1.0, ControlMode.SERVO_POSITION, "teleop")
    measured.joint_positions["joint"] = -1.0
    teleop.is_active.return_value = False
    points = [TrajectoryPoint(positions=[0.1])]
    if not single_point:
        points = [
            TrajectoryPoint(positions=[-1.0]),
            TrajectoryPoint(positions=[0.1], time_from_start=2.0),
        ]
    assert (
        task.execute(
            JointTrajectory(joint_names=["joint"], points=points), measured.joint_positions
        ).status
        is TrajectoryExecutionStatus.ACCEPTED
    )
    clock.return_value = 0.21
    loop._tick()
    expected = -0.99 if single_point else -1.0
    assert route.call_args.args[0]["joint"][0] == pytest.approx(expected)


def test_completed_joint_reanchors_while_other_joint_keeps_command_continuity():
    task = JointTrajectoryTask(JointTrajectoryTaskConfig(joint_names=["finished", "running"]))
    measured = JointStateSnapshot(joint_positions={"finished": 0.0, "running": 0.0})
    initial = JointTrajectory(
        joint_names=["finished", "running"], points=[TrajectoryPoint(positions=[0.1, 1.0])]
    )
    assert task.execute(initial, {}).status is TrajectoryExecutionStatus.ACCEPTED
    output = task.compute(CoordinatorState(joints=measured, t_now=0.1, dt=0.1))
    assert output is not None
    assert output.positions == pytest.approx([0.1, 0.1])
    measured.joint_positions["finished"] = -1.0
    replacement = JointTrajectory(
        joint_names=["finished"],
        points=[
            TrajectoryPoint(positions=[-1.0]),
            TrajectoryPoint(positions=[0.1], time_from_start=2.0),
        ],
    )
    assert (
        task.execute(replacement, measured.joint_positions).status
        is TrajectoryExecutionStatus.ACCEPTED
    )
    output = task.compute(CoordinatorState(joints=measured, t_now=0.2, dt=0.1))
    assert output is not None
    assert output.positions == pytest.approx([-1.0, 0.2])


@pytest.mark.parametrize(
    "positions, expected",
    [
        ({}, TrajectoryExecutionStatus.START_STATE_UNAVAILABLE),
        ({"joint": -1.0}, TrajectoryExecutionStatus.START_STATE_MISMATCH),
    ],
)
def test_completed_trajectory_does_not_bypass_start_validation(positions, expected):
    task = JointTrajectoryTask(JointTrajectoryTaskConfig(joint_names=["joint"]))
    initial = JointTrajectory(joint_names=["joint"], points=[TrajectoryPoint(positions=[0.1])])
    assert task.execute(initial, {}).status is TrajectoryExecutionStatus.ACCEPTED
    task.compute(
        CoordinatorState(
            joints=JointStateSnapshot(joint_positions={"joint": 0.0}), t_now=0.1, dt=0.1
        )
    )
    trajectory = JointTrajectory(
        joint_names=["joint"],
        points=[
            TrajectoryPoint(positions=[0.1]),
            TrajectoryPoint(positions=[1.0], time_from_start=2.0),
        ],
    )
    assert task.execute(trajectory, positions).status is expected
