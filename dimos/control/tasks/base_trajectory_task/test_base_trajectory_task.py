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

import math

import pytest

from dimos.control.task import CoordinatorState, JointStateSnapshot
from dimos.control.tasks.base_trajectory_task.base_trajectory_task import (
    BaseTrajectoryTask,
    BaseTrajectoryTaskConfig,
)
from dimos.control.tasks.trajectory_task.trajectory_task import TrajectoryExecutionStatus
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.msgs.trajectory_msgs.TrajectoryStatus import TrajectoryState
from dimos.utils.trigonometry import angle_diff

JOINTS = ["base/vx", "base/vy", "base/wz"]
DT = 0.01
ZERO = [0.0, 0.0, 0.0]


class _TwistBase:
    """Integrates body-frame velocity into world odometry, reporting yaw wrapped."""

    def __init__(self, x=0.0, y=0.0, yaw=0.0, gain=1.0):
        self.x, self.y, self.yaw = x, y, yaw
        self.gain = gain
        self.odometry = None

    def state(self, t):
        positions = self.odometry
        if positions is None:
            positions = dict(zip(JOINTS, (self.x, self.y, angle_diff(self.yaw, 0.0)), strict=True))
        return CoordinatorState(
            joints=JointStateSnapshot(joint_positions=positions), t_now=t, dt=DT
        )

    def step(self, command):
        vx, vy, wz = (self.gain * v for v in command.velocities)
        self.x += (math.cos(self.yaw) * vx - math.sin(self.yaw) * vy) * DT
        self.y += (math.sin(self.yaw) * vx + math.cos(self.yaw) * vy) * DT
        self.yaw += wz * DT


def _line(start, goal, duration, n=60):
    velocity = [(g - s) / duration for s, g in zip(start, goal, strict=True)]
    points = [
        TrajectoryPoint(
            time_from_start=duration * i / (n - 1),
            positions=[s + (g - s) * i / (n - 1) for s, g in zip(start, goal, strict=True)],
            velocities=velocity,
        )
        for i in range(n)
    ]
    return JointTrajectory(points=points, joint_names=["x", "y", "yaw"])


def _run(task, base, seconds, t=100.0):
    outputs = []
    for _ in range(round(seconds / DT)):
        output = task.compute(base.state(t))
        outputs.append(output)
        if output is not None:
            base.step(output)
        t += DT
    return outputs, t


def _task(**overrides):
    return BaseTrajectoryTask(
        "base_traj", BaseTrajectoryTaskConfig(joint_names=JOINTS, **overrides)
    )


def test_follows_a_timed_trajectory_across_the_yaw_seam():
    """The plan's yaw is unwrapped, a full turn past odometry's; tracking must not care."""
    base = _TwistBase(yaw=3.0)
    task = _task()
    turn = 2 * math.pi
    plan = _line((0.0, 0.0, 3.0 + turn), (1.0, 0.5, 3.6 + turn), duration=4.0)
    task.execute(plan)

    _, t = _run(task, base, seconds=2.0)
    reference, _ = plan.sample(2.0)
    assert math.hypot(base.x - reference[0], base.y - reference[1]) < 0.01
    assert abs(angle_diff(base.yaw, reference[2])) < 0.01

    _run(task, base, seconds=4.0, t=t)
    assert task.get_status().state == TrajectoryState.COMPLETED
    assert math.hypot(base.x - 1.0, base.y - 0.5) < 0.05
    assert abs(angle_diff(base.yaw, 3.6)) < 0.1


def test_feedback_pulls_an_offset_base_onto_the_plan():
    """Heading near pi/2, where a wrong rotation sign in the feedback pushes the base away."""
    base = _TwistBase(x=0.05, y=-0.05, yaw=1.55)
    task = _task()
    plan = _line((0.0, 0.0, 1.5), (1.0, 0.5, 2.1), duration=4.0)
    task.execute(plan)

    _run(task, base, seconds=2.0)

    reference, _ = plan.sample(2.0)
    assert math.hypot(base.x - reference[0], base.y - reference[1]) < 0.02
    assert abs(angle_diff(base.yaw, reference[2])) < 0.02


def test_a_base_that_runs_ahead_settles_back_onto_the_goal():
    """The plan ends moving; settling must not keep adding its last velocity."""
    base = _TwistBase(gain=1.5)
    task = _task()
    task.execute(_line((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), duration=4.0))

    _run(task, base, seconds=6.0)

    assert task.get_status().state == TrajectoryState.COMPLETED
    assert abs(base.x - 1.0) < 0.05


def test_a_second_trajectory_starts_on_its_first_tick():
    """A new plan cuts the last run's stop hold; a late start would put the base behind the arm."""
    base = _TwistBase()
    task = _task()
    task.execute(_line((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), duration=2.0))
    _, t = _run(task, base, seconds=2.2)
    assert task.get_status().state == TrajectoryState.COMPLETED

    plan = _line((base.x, base.y, base.yaw), (base.x + 1.0, base.y, base.yaw), 2.0)
    task.execute(plan)
    _, t = _run(task, base, seconds=1.0, t=t)
    assert abs(base.x - plan.sample(1.0)[0][0]) < 0.01

    _run(task, base, seconds=2.0, t=t)
    assert task.get_status().state == TrajectoryState.COMPLETED
    assert abs(base.x - 2.0) < 0.05


def test_rejects_trajectories_it_cannot_follow():
    nan_velocity = _line((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), duration=2.0)
    nan_velocity.points[5].velocities = [math.nan, 0.0, 0.0]
    never_ends = _line((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), duration=2.0)
    never_ends.points[-1].time_from_start = math.inf
    two_columns = JointTrajectory(
        points=[TrajectoryPoint(positions=[0.0, 0.0], velocities=[0.0, 0.0])]
    )
    diagonal_too_fast = _line((0.0, 0.0, 0.0), (1.0, 1.0, 0.0), duration=1.0)

    for plan in (nan_velocity, never_ends, two_columns, diagonal_too_fast):
        task = _task(max_linear=1.0)
        assert task.execute(plan).status == TrajectoryExecutionStatus.INVALID_TRAJECTORY
        assert not task.is_active()


def test_rejects_a_trajectory_that_does_not_start_at_the_base():
    base = _TwistBase()
    task = _task()
    task.execute(_line((1.0, 0.0, 0.0), (2.0, 0.0, 0.0), duration=2.0))

    _run(task, base, seconds=1.0)

    status = task.get_status()
    assert status.state == TrajectoryState.ABORTED
    assert "starts" in status.error
    assert math.hypot(base.x, base.y) < 1e-9


def test_preemption_aborts_and_zeroes_the_joints_the_winner_left():
    """The tick loop re-notifies every tick; the hold must not restart."""
    base = _TwistBase()
    task = _task(stop_hold_s=0.5)
    task.execute(_line((0.0, 0.0, 0.0), (2.0, 0.0, 0.0), duration=4.0))
    _, t = _run(task, base, seconds=0.5)

    outputs = []
    for _ in range(100):
        task.on_preempted("turn_only", frozenset(JOINTS[2:]))
        assert task.get_status().state == TrajectoryState.ABORTED
        outputs.append(task.compute(base.state(t)))
        t += DT

    held = [o for o in outputs if o is not None]
    assert 45 <= len(held) <= 55
    assert all(o.velocities == ZERO for o in held)
    assert outputs[-1] is None


def test_cancel_aborts_at_once_and_holds_zero_velocity():
    """One zero does not stop a gliding base, so a cancel streams zeros for the hold."""
    base = _TwistBase()
    task = _task(stop_hold_s=0.5)
    task.execute(_line((0.0, 0.0, 0.0), (2.0, 0.0, 0.0), duration=4.0))
    _, t = _run(task, base, seconds=1.0)

    assert task.cancel()
    assert not task.cancel()
    assert task.get_status().state == TrajectoryState.ABORTED
    outputs, _ = _run(task, base, seconds=0.6, t=t)

    assert all(o.velocities == ZERO for o in outputs[:50])
    assert outputs[-1] is None


@pytest.mark.parametrize(
    "loss, goal",
    [
        ("missing", (2.0, 0.0, 0.0)),
        ("nan", (2.0, 0.0, 0.0)),
        ("frozen", (2.0, 0.0, 0.0)),
        ("frozen", (0.0, 0.0, 2.0)),
    ],
)
def test_lost_odometry_aborts_and_stops_the_base(loss, goal):
    base = _TwistBase()
    task = _task()
    task.execute(_line((0.0, 0.0, 0.0), goal, duration=2.0))
    _, t = _run(task, base, seconds=0.5)

    base.odometry = {
        "missing": {},
        "nan": dict.fromkeys(JOINTS, math.nan),
        "frozen": dict(base.state(t).joints.joint_positions),
    }[loss]
    outputs, _ = _run(task, base, seconds=1.2, t=t)

    assert task.get_status().state == TrajectoryState.ABORTED
    assert [o for o in outputs if o is not None][-1].velocities == ZERO
