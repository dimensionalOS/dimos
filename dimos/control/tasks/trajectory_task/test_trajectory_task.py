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

from dimos.control.task import (
    CoordinatorState,
    JointStateSnapshot,
)
from dimos.control.tasks.trajectory_task.trajectory_task import (
    JointTrajectoryTask,
    JointTrajectoryTaskConfig,
    TrajectoryExecutionStatus,
)
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint


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


def test_continuous_stream_follows_a_lagging_joint_without_teeth() -> None:
    dt, limit, lead, sag, tau = 0.01, 3.0, 0.3, 0.05, 0.13
    allowance = limit * dt
    task = JointTrajectoryTask(
        JointTrajectoryTaskConfig(
            joint_names=["arm/a"],
            velocity_limits={"arm/a": limit},
            continuous_stream=True,
            stream_lead_limits={"arm/a": lead},
        )
    )
    # A follower that lags its command by tau and sags by a constant offset.
    t, target, measured, command = 0.0, 0.0, 0.0, 0.0
    lags: list[float] = []

    def tick(speed: float, *, send: bool = True, blocked: bool = False) -> float:
        nonlocal t, target, measured, command
        t += dt
        target += speed * dt
        if send:
            task.on_joint_command(
                JointState(frame_id="dimos/guarded-joint", name=["arm/a"], position=[target]), t
            )
        state = CoordinatorState(
            joints=JointStateSnapshot(joint_positions={"arm/a": measured}), t_now=t, dt=dt
        )
        output = task.compute(state)
        step = 0.0
        if output is not None:
            step = output.positions[0] - command
            command = output.positions[0]
        assert abs(command - measured) <= lead + 1e-12
        lags.append(target - measured)
        if not blocked:
            measured += dt / tau * (command - sag - measured)
        return step

    # Moving, and blocked: never a step back, never past the velocity limit or the lead.
    for _ in range(100):
        assert -1e-12 <= tick(1.5) <= allowance + 1e-12
    assert max(lags) > 5 * allowance
    for _ in range(50):
        assert -1e-12 <= tick(1.5, blocked=True) <= allowance + 1e-12
    assert command == pytest.approx(measured + lead)
    for _ in range(200):
        assert -1e-12 <= tick(0.0) <= allowance + 1e-12
    assert command == pytest.approx(target)
    assert measured < command - allowance

    # After a silence longer than the gap timeout the stream starts from the reading.
    for _ in range(20):
        tick(0.0, send=False)
    held = measured
    tick(50.0)
    assert command == pytest.approx(held + allowance)
