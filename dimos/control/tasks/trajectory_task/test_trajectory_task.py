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

import pytest

from dimos.control._control_test_helpers import execute_with_state
from dimos.control.task import (
    CoordinatorState,
    JointStateSnapshot,
)
from dimos.control.tasks.trajectory_task.trajectory_task import (
    JointTrajectoryTask,
    JointTrajectoryTaskConfig,
    TrajectoryExecutionStatus,
)
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.msgs.trajectory_msgs.TrajectoryStatus import TrajectoryState


@pytest.mark.parametrize("single_point", [False, True])
@pytest.mark.parametrize("preempted", [False, True])
def test_jtt_does_not_pull_joint_back_after_another_task_moves_it(single_point, preempted):
    task = JointTrajectoryTask(JointTrajectoryTaskConfig(joint_names=["joint"]))
    state = CoordinatorState(
        joints=JointStateSnapshot(joint_positions={"joint": 0.0}), t_now=0.1, dt=0.1
    )
    first_target = 1.0 if preempted else 0.1
    first = JointTrajectory(
        joint_names=["joint"], points=[TrajectoryPoint(positions=[first_target])]
    )
    assert execute_with_state(task, first, {}).status is TrajectoryExecutionStatus.ACCEPTED
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
        execute_with_state(
            task,
            JointTrajectory(joint_names=["joint"], points=points),
            state.joints.joint_positions,
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
    task = JointTrajectoryTask(JointTrajectoryTaskConfig(joint_names=["finished", "running"]))
    measured = JointStateSnapshot(joint_positions={"finished": 0.0, "running": 0.0})
    initial = JointTrajectory(
        joint_names=["finished", "running"], points=[TrajectoryPoint(positions=[0.1, 1.0])]
    )
    assert execute_with_state(task, initial, {}).status is TrajectoryExecutionStatus.ACCEPTED
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
        execute_with_state(task, replacement, measured.joint_positions).status
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
    assert execute_with_state(task, initial, {}).status is TrajectoryExecutionStatus.ACCEPTED
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
    assert execute_with_state(task, trajectory, positions).status is expected


def test_each_joint_run_starts_on_its_first_tick_and_status_queries_do_not_start_it():
    task = JointTrajectoryTask(
        JointTrajectoryTaskConfig(joint_names=["a", "b"], velocity_limits={"a": 10.0, "b": 10.0})
    )
    first = JointTrajectory(
        joint_names=["a", "b"],
        points=[
            TrajectoryPoint(positions=[0.0, 0.0], time_from_start=0.0),
            TrajectoryPoint(positions=[1.0, 1.0], time_from_start=2.0),
        ],
    )
    assert (
        execute_with_state(task, first, {"a": 0.0, "b": 0.0}).status
        == TrajectoryExecutionStatus.ACCEPTED
    )
    pending = task.get_status(1000.0)
    assert (pending.progress, pending.time_elapsed, pending.time_remaining) == (0.0, 0.0, 2.0)
    feedback = JointStateSnapshot(joint_positions={"a": 0.0, "b": 0.0})
    assert task.compute(CoordinatorState(joints=feedback, t_now=10.0, dt=0.5)) is not None
    output = task.compute(CoordinatorState(joints=feedback, t_now=10.5, dt=0.5))
    assert output is not None and output.positions == [0.25, 0.25]
    replacement = JointTrajectory(
        joint_names=["a"],
        points=[
            TrajectoryPoint(positions=[0.25], time_from_start=0.0),
            TrajectoryPoint(positions=[0.5], time_from_start=1.0),
        ],
    )
    assert execute_with_state(task, replacement, {}).status == TrajectoryExecutionStatus.ACCEPTED
    assert task.get_status(1000.0).progress == 0.0
    output = task.compute(CoordinatorState(joints=feedback, t_now=11.0, dt=0.5))
    assert output is not None and output.positions == [0.25, 0.5]
    status = task.get_status(11.5)
    assert (status.progress, status.time_elapsed, status.time_remaining) == (0.5, 1.5, 0.5)


def _ramp():
    return JointTrajectory(
        joint_names=["joint"],
        points=[
            TrajectoryPoint(positions=[0.0]),
            TrajectoryPoint(positions=[1.0], time_from_start=2.0),
        ],
    )


@pytest.mark.parametrize(
    "keepalive, expected",
    [
        (False, TrajectoryState.ABORTED),
        (True, TrajectoryState.EXECUTING),
    ],
)
def test_deadman_aborts_trajectory_when_keepalive_stops(keepalive, expected, mocker):
    task = JointTrajectoryTask(JointTrajectoryTaskConfig(joint_names=["joint"]))
    joints = JointStateSnapshot(joint_positions={"joint": 0.0})
    assert (
        execute_with_state(task, _ramp(), joints.joint_positions, deadman_timeout=0.5).status
        == TrajectoryExecutionStatus.ACCEPTED
    )
    receipt_clock = mocker.patch("dimos.control.tasks.trajectory_clock.time.perf_counter")
    for tick in range(1, 101):
        t_now = tick * 0.01
        if keepalive:
            receipt_clock.return_value = t_now
            task.keepalive()
        task.compute(CoordinatorState(joints=joints, t_now=t_now, dt=0.01))
    assert task.get_state() == expected


def test_joint_deadman_boundary_and_next_unsupervised_run():
    task = JointTrajectoryTask(JointTrajectoryTaskConfig(joint_names=["joint"]))
    joints = JointStateSnapshot(joint_positions={"joint": 0.0})
    assert (
        execute_with_state(task, _ramp(), joints.joint_positions, deadman_timeout=0.5).status
        == TrajectoryExecutionStatus.ACCEPTED
    )
    assert task.compute(CoordinatorState(joints=joints, t_now=10.0, dt=0.01)) is not None
    assert task.compute(CoordinatorState(joints=joints, t_now=10.5, dt=0.01)) is not None
    assert task.compute(CoordinatorState(joints=joints, t_now=10.5001, dt=0.01)) is None
    assert task.get_state() == TrajectoryState.ABORTED
    assert (
        execute_with_state(task, _ramp(), joints.joint_positions).status
        == TrajectoryExecutionStatus.ACCEPTED
    )
    task.compute(CoordinatorState(joints=joints, t_now=20.0, dt=0.01))
    assert task.compute(CoordinatorState(joints=joints, t_now=21.0, dt=0.01)) is not None
    assert task.get_state() == TrajectoryState.EXECUTING


@pytest.mark.parametrize("timeout", [0.0, -1.0, float("nan"), float("inf")])
def test_invalid_deadman_timeout_is_rejected(timeout):
    task = JointTrajectoryTask(JointTrajectoryTaskConfig(joint_names=["joint"]))
    assert (
        execute_with_state(task, _ramp(), {"joint": 0.0}, deadman_timeout=timeout).status
        == TrajectoryExecutionStatus.INVALID_TRAJECTORY
    )
    assert not task.is_active()


@pytest.mark.parametrize(
    "supervised, expected",
    [
        ("arm", TrajectoryState.ABORTED),
        ("grip", TrajectoryState.EXECUTING),
    ],
)
def test_deadman_covers_only_the_joints_it_was_sent_with(supervised, expected):
    task = JointTrajectoryTask(JointTrajectoryTaskConfig(joint_names=["arm", "grip"]))
    joints = JointStateSnapshot(joint_positions={"arm": 0.0, "grip": 0.0})
    arm = JointTrajectory(
        joint_names=["arm"],
        points=[
            TrajectoryPoint(positions=[0.0]),
            TrajectoryPoint(positions=[1.0], time_from_start=2.0),
        ],
    )
    grip = JointTrajectory(
        joint_names=["grip"],
        points=[
            TrajectoryPoint(positions=[0.0]),
            TrajectoryPoint(positions=[0.05], time_from_start=0.05),
        ],
    )
    for name, trajectory in (("arm", arm), ("grip", grip)):
        timeout = 0.5 if name == supervised else None
        assert (
            execute_with_state(
                task, trajectory, joints.joint_positions, deadman_timeout=timeout
            ).status
            == TrajectoryExecutionStatus.ACCEPTED
        )
    for tick in range(1, 101):
        task.compute(CoordinatorState(joints=joints, t_now=tick * 0.01, dt=0.01))
    assert task.get_state() == expected
