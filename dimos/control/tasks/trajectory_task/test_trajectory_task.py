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

from unittest.mock import MagicMock

import pytest

from dimos.control.components import HardwareComponent, HardwareType
from dimos.control.hardware_interface import ConnectedHardware
from dimos.control.task import (
    CoordinatorState,
    JointStateSnapshot,
)
from dimos.control.tasks.trajectory_task.trajectory_task import (
    JointTrajectoryTask,
    JointTrajectoryTaskConfig,
    TrajectoryExecutionStatus,
)
from dimos.hardware.manipulators.spec import ManipulatorAdapter
from dimos.hardware.spec import JointLimits
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint


def make_trajectory_task(joint_names: list[str]) -> JointTrajectoryTask:
    adapter = MagicMock(spec=ManipulatorAdapter)
    adapter.get_limits.return_value = JointLimits(
        position_lower=[-10.0] * len(joint_names),
        position_upper=[10.0] * len(joint_names),
        velocity_max=[10.0] * len(joint_names),
    )
    component = HardwareComponent(
        hardware_id="trajectory_test",
        hardware_type=HardwareType.MANIPULATOR,
        joints=joint_names,
    )
    hardware = ConnectedHardware(adapter=adapter, component=component)
    return JointTrajectoryTask(
        JointTrajectoryTaskConfig(joint_names=joint_names),
        hardware={component.hardware_id: hardware},
    )


@pytest.mark.parametrize("single_point", [False, True])
@pytest.mark.parametrize("preempted", [False, True])
def test_jtt_does_not_pull_joint_back_after_another_task_moves_it(single_point, preempted):
    task = make_trajectory_task(["joint"])
    state = CoordinatorState(
        joints=JointStateSnapshot(joint_positions={"joint": 0.0}), t_now=0.1, dt=0.1
    )
    first_target = 1.0 if preempted else 0.1
    first = JointTrajectory(
        joint_names=["joint"], points=[TrajectoryPoint(positions=[first_target])]
    )
    assert task.execute(first, {}).status is TrajectoryExecutionStatus.ACCEPTED
    output = task.compute(state)
    assert output is not None
    assert output.positions == pytest.approx([0.1])
    if preempted:
        task.on_preempted("other_task", frozenset({"joint"}))
    assert not task.is_active()

    # Another task moves the joint away from JTT's previous command.
    state = CoordinatorState(
        joints=JointStateSnapshot(joint_positions={"joint": -1.0}), t_now=1.0, dt=0.01
    )
    assert task.compute(state) is None
    points = [TrajectoryPoint(positions=[-1.2])]
    if not single_point:
        points = [
            TrajectoryPoint(positions=[-1.0]),
            TrajectoryPoint(positions=[-1.2], time_from_start=0.1),
        ]
    assert (
        task.execute(
            JointTrajectory(joint_names=["joint"], points=points), state.joints.joint_positions
        ).status
        is TrajectoryExecutionStatus.ACCEPTED
    )

    # The new motion goes farther away from the old command. Every output
    # must move toward the new target, bounded from the new starting pose.
    previous = -1.0
    for tick in range(30):
        state.t_now = 1.0 + tick * state.dt
        output = task.compute(state)
        if output is None:
            break
        commanded = output.positions[0]
        assert -state.dt - 1e-9 <= commanded - previous <= 1e-9
        previous = commanded
    assert previous == pytest.approx(-1.2)
    assert not task.is_active()
    assert task.compute(state) is None


def test_completed_joint_reanchors_while_other_joint_keeps_command_continuity():
    task = make_trajectory_task(["finished", "running"])
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
    task = make_trajectory_task(["joint"])
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
