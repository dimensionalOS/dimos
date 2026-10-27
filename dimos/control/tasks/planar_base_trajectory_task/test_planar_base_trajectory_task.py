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
from dimos.control.tasks.planar_base_trajectory_task.planar_base_trajectory_task import (
    PlanarBaseTrajectoryTask,
    PlanarBaseTrajectoryTaskConfig,
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
    """Integrates body-frame velocity into world odometry, reporting yaw wrapped.

    ``gain`` is the plant's response per unit command, one value or (vx, vy, wz).
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, gain=1.0):
        self.x, self.y, self.yaw = x, y, yaw
        self.gain = gain if isinstance(gain, tuple) else (gain, gain, gain)
        self.odometry = None

    def state(self, t):
        positions = self.odometry
        if positions is None:
            positions = dict(zip(JOINTS, (self.x, self.y, angle_diff(self.yaw, 0.0)), strict=True))
        return CoordinatorState(
            joints=JointStateSnapshot(joint_positions=positions), t_now=t, dt=DT
        )

    def step(self, command):
        vx, vy, wz = (k * v for k, v in zip(self.gain, command.velocities, strict=True))
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
    return PlanarBaseTrajectoryTask(
        "base_traj", PlanarBaseTrajectoryTaskConfig(joint_names=JOINTS, **overrides)
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


def test_a_relative_trajectory_runs_from_wherever_the_base_stands():
    """Planned in the base's own frame (its first point is zero), the motion is laid down from
    the odometry pose on its first tick: forward is the way the base faces."""
    base = _TwistBase(x=3.0, y=-1.0, yaw=1.2)
    task = _task(relative=True)
    result = task.execute(
        _line((0.0, 0.0, 0.0), (0.5, 0.2, 0.3), duration=2.0),
        {"base/vx": 3.0, "base/vy": -1.0, "base/wz": 1.2},
    )
    assert result.status == TrajectoryExecutionStatus.ACCEPTED

    _run(task, base, seconds=3.0)

    assert task.get_status().state == TrajectoryState.COMPLETED
    cos_yaw, sin_yaw = math.cos(1.2), math.sin(1.2)
    assert base.x == pytest.approx(3.0 + cos_yaw * 0.5 - sin_yaw * 0.2, abs=0.03)
    assert base.y == pytest.approx(-1.0 + sin_yaw * 0.5 + cos_yaw * 0.2, abs=0.03)
    assert angle_diff(base.yaw, 1.5) == pytest.approx(0.0, abs=0.05)


def test_a_relative_trajectory_must_still_start_at_zero():
    base = _TwistBase(x=3.0)
    task = _task(relative=True)
    result = task.execute(_line((1.0, 0.0, 0.0), (2.0, 0.0, 0.0), duration=2.0))
    assert result.status == TrajectoryExecutionStatus.ACCEPTED, "checked on the first tick"

    _run(task, base, seconds=0.5)

    assert task.get_status().state == TrajectoryState.ABORTED
    assert "starts" in task.get_status().error
    assert base.x == pytest.approx(3.0)


# Alfred's FlowBase: it translates 0.77x and yaws 2.85x what it is told.
_ALFRED_PLANT = (0.77, 0.77, 2.85)
_ALFRED_COMMAND_GAINS = tuple(1.0 / k for k in _ALFRED_PLANT)


def test_command_gains_scale_the_command_after_the_limits():
    """The limits are physical, so they clamp before the gain: a base told to go at most
    0.5 m/s through a 2x gain is told 1.0, not 1.1 when feedback adds to a saturated plan."""
    plan = _line((0.0, 0.0, 0.0), (1.0, 0.0, 2.0), duration=2.0)  # 0.5 m/s, 1.0 rad/s: the limits
    scaled = _task(max_linear=0.5, max_angular=1.0, command_gains=(2.0, 2.0, 0.5))
    scaled.execute(plan)
    on_plan = scaled.compute(_TwistBase().state(100.0))
    assert on_plan.velocities == pytest.approx([1.0, 0.0, 0.5])

    scaled = _task(max_linear=0.5, max_angular=1.0, command_gains=(2.0, 2.0, 0.5))
    scaled.execute(plan)
    vx, vy, wz = scaled.compute(_TwistBase(x=-0.05, yaw=-0.05).state(100.0)).velocities
    assert (math.hypot(vx, vy), wz) == pytest.approx((1.0, 0.5))

    unscaled = _task(max_linear=0.5, max_angular=1.0)
    unscaled.execute(plan)
    vx, vy, wz = unscaled.compute(_TwistBase(x=-0.05, yaw=-0.05).state(100.0)).velocities
    assert (math.hypot(vx, vy), wz) == pytest.approx((0.5, 1.0))


def test_command_gains_let_a_base_with_a_measured_plant_track_a_turn():
    """Alfred's field failure: uncompensated, a yaw segment runs away from the plan 2.85x
    faster than kp=1 feedback can hold it and aborts; through 1/K it completes."""
    turn = _line((0.0, 0.0, 0.0), (0.0, 0.0, 1.2), duration=1.5)  # 0.8 rad/s

    raw = _task(max_angular=1.0)
    raw.execute(turn)
    _run(raw, _TwistBase(gain=_ALFRED_PLANT), seconds=3.0)
    assert raw.get_status().state == TrajectoryState.ABORTED
    assert "off the trajectory" in raw.get_status().error

    compensated = _task(max_angular=1.0, command_gains=_ALFRED_COMMAND_GAINS)
    compensated.execute(turn)
    base = _TwistBase(gain=_ALFRED_PLANT)
    _run(compensated, base, seconds=3.0)
    assert compensated.get_status().state == TrajectoryState.COMPLETED
    assert angle_diff(base.yaw, 1.2) == pytest.approx(0.0, abs=0.05)


def test_command_gains_default_to_unity():
    from types import SimpleNamespace

    from dimos.control.tasks.planar_base_trajectory_task.planar_base_trajectory_task import (
        PlanarBaseTrajectoryTaskParams,
        create_task,
    )

    assert PlanarBaseTrajectoryTaskConfig(joint_names=JOINTS).command_gains == (1.0, 1.0, 1.0)
    assert PlanarBaseTrajectoryTaskParams().command_gains == (1.0, 1.0, 1.0)

    def task_from(params):
        return create_task(
            SimpleNamespace(name="base", joint_names=JOINTS, priority=10, params=params), None
        )

    assert task_from({})._config.command_gains == (1.0, 1.0, 1.0)
    assert task_from({"command_gains": [1.3, 1.3, 0.35]})._config.command_gains == (
        1.3,
        1.3,
        0.35,
    )
    with pytest.raises(ValueError):
        task_from({"command_gains": [1.0, 1.0, 0.0]})

    plan = _line((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), duration=2.0)
    task = _task()
    task.execute(plan)
    assert task.compute(_TwistBase().state(100.0)).velocities == pytest.approx([0.5, 0.0, 0.0])
