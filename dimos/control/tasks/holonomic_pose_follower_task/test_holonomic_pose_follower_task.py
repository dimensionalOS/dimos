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

"""Holonomic full-pose follower: honors the commanded yaw (decoupled from the
travel direction), strafes, respects the envelope, and survives replans —
validated closed-loop against the in-repo FOPDT plant sim."""

from __future__ import annotations

import math
from types import SimpleNamespace

import pytest

from dimos.control.benchmarking.paths import (
    circle_offset_heading,
    hold_heading,
    square,
    strafe_line,
    straight_rotate,
)
from dimos.control.benchmarking.plant import (
    FopdtChannelParams,
    TwistBasePlantParams,
    TwistBasePlantSim,
)
from dimos.control.benchmarking.scoring import ExecutedTrajectory, TrajectoryTick, score_run
from dimos.control.benchmarking.tuning import TuningConfig
from dimos.control.task import ControlMode, CoordinatorState, JointStateSnapshot
from dimos.control.tasks.holonomic_pose_follower_task.holonomic_pose_follower_task import (
    DEFAULT_ARTIFACT_PATH,
    create_task,
)
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.utils.trigonometry import angle_diff

_JOINTS = ["go2/vx", "go2/vy", "go2/wz"]
_DT = 0.1


def _pose(x=0.0, y=0.0, yaw=0.0):
    return PoseStamped(
        position=Vector3(x, y, 0.0),
        orientation=Quaternion.from_euler(Vector3(0.0, 0.0, yaw)),
    )


def _state(x, y, yaw, t):
    return CoordinatorState(
        joints=JointStateSnapshot(
            joint_positions={_JOINTS[0]: x, _JOINTS[1]: y, _JOINTS[2]: yaw},
            joint_velocities=dict.fromkeys(_JOINTS, 0.0),
        ),
        t_now=t,
        dt=_DT,
    )


def _task(**params):
    return create_task(
        SimpleNamespace(
            name="holo", joint_names=_JOINTS, priority=10, params={"speed": 0.5, **params}
        ),
        None,
    )


def _plant():
    art = TuningConfig.from_json(DEFAULT_ARTIFACT_PATH)
    p = TwistBasePlantSim(
        TwistBasePlantParams(
            vx=FopdtChannelParams(art.plant.vx.K, art.plant.vx.tau, art.plant.vx.L),
            vy=FopdtChannelParams(art.plant.vy.K, art.plant.vy.tau, art.plant.vy.L),
            wz=FopdtChannelParams(art.plant.wz.K, art.plant.wz.tau, art.plant.wz.L),
        )
    )
    p.reset(0.0, 0.0, 0.0, _DT)
    return p


def _run_closed_loop(task, path, max_ticks=2400, on_tick=None):
    """Drive the task against the FOPDT sim; return the executed trace."""
    plant = _plant()
    assert task.start_path(path, _pose(plant.x, plant.y, plant.yaw))
    ticks = []
    for k in range(max_ticks):
        out = task.compute(_state(plant.x, plant.y, plant.yaw, t=k * _DT))
        vx, vy, wz = out.velocities if out is not None else (0.0, 0.0, 0.0)
        ticks.append(
            TrajectoryTick(
                t=k * _DT,
                pose=_pose(plant.x, plant.y, plant.yaw),
                cmd_twist=Twist(linear=Vector3(vx, vy, 0.0), angular=Vector3(0.0, 0.0, wz)),
                actual_twist=Twist(),
            )
        )
        if on_tick is not None:
            on_tick(k, task, plant)
        if task.get_state() == "arrived":
            break
        plant.step(vx, vy, wz, _DT)
    return plant, ExecutedTrajectory(ticks=ticks, arrived=task.get_state() == "arrived")


# Task protocol basics


def test_claims_velocity_mode_on_three_joints():
    task = _task()
    claim = task.claim()
    assert claim.joints == frozenset(_JOINTS)
    assert claim.mode == ControlMode.VELOCITY
    assert not task.is_active()


def test_set_path_arms_and_preemption_aborts():
    task = _task()
    task.start_path(straight_rotate(), _pose())
    assert task.is_active()
    task.on_preempted("teleop", frozenset(_JOINTS))
    assert task.get_state() == "aborted"


def test_streamed_path_arms_on_the_first_tick_with_a_pose():
    """The card handler carries no odom, so a streamed path is latched and armed
    on the first tick that has a pose — never dropped for arriving early."""
    task = _task()
    task.on_path(straight_rotate(), t_now=0.0)
    assert not task.is_active()  # latched, not armed
    task.compute(CoordinatorState(joints=JointStateSnapshot(), t_now=0.0, dt=_DT))
    assert not task.is_active()  # still no pose available
    task.compute(_state(0.0, 0.0, 0.0, t=_DT))
    assert task.is_active()


def test_streamed_speed_applies_before_the_path():
    task = _task()
    task.on_speed(Float32(data=0.8), t_now=0.0)
    assert task._config.speed == pytest.approx(0.8)


def test_set_speed_refused_while_active():
    task = _task()
    task.start_path(straight_rotate(), _pose())
    task.set_speed(0.9)
    assert task._config.speed == 0.5


def test_rejects_pure_rotation_path():
    task = _task()
    path = Path(poses=[_pose(0, 0, 0.0), _pose(0, 0, 1.0)])
    assert not task.start_path(path, _pose())


def test_compute_without_pose_commands_zero():
    task = _task()
    task.start_path(straight_rotate(), _pose())
    state = CoordinatorState(joints=JointStateSnapshot(), t_now=0.0, dt=_DT)
    out = task.compute(state)
    assert out is not None
    assert out.velocities == [0.0, 0.0, 0.0]


# The distinctive capability: commanded yaw decoupled from travel direction


def test_tracks_commanded_yaw_not_tangent_while_translating():
    """Straight +x while the commanded yaw ramps 0 -> 90deg: at mid-path the
    robot must be facing ~45deg although the tangent is 0."""
    task = _task()
    mid_yaws = []

    def snoop(k, task_, plant):
        if 1.4 <= plant.x <= 1.6:
            mid_yaws.append(plant.yaw)

    plant, executed = _run_closed_loop(task, straight_rotate(length=3.0), on_tick=snoop)
    assert executed.arrived
    assert mid_yaws, "robot never crossed mid-path"
    assert math.degrees(abs(sum(mid_yaws) / len(mid_yaws))) == pytest.approx(45.0, abs=10.0)
    assert abs(angle_diff(plant.yaw, math.pi / 2)) < 0.25
    score = score_run(straight_rotate(length=3.0), executed)
    assert score.cte_rms < 0.10
    assert score.heading_err_rms < 0.15  # vs COMMANDED yaw


def test_strafes_holding_commanded_yaw():
    """Lateral path with commanded yaw 0: must strafe (vy), not turn-and-drive."""
    task = _task()
    plant, executed = _run_closed_loop(task, strafe_line(length=2.0))
    assert executed.arrived
    max_yaw = max(abs(t.pose.orientation.euler[2]) for t in executed.ticks)
    assert max_yaw < 0.1, "robot rotated instead of strafing"
    assert plant.y == pytest.approx(2.0, abs=0.25)
    # The motion was carried by the lateral channel.
    assert max(abs(t.cmd_twist.linear.y) for t in executed.ticks) > 0.3
    assert max(abs(t.cmd_twist.linear.x) for t in executed.ticks) < 0.1


def test_crab_walks_square_holding_heading():
    """Square geometry with the commanded yaw held at 0: the robot must round
    all four corners by swapping body-frame velocity channels, never turning."""
    path = hold_heading(square(side=2.0), yaw=0.0)
    task = _task()
    plant, executed = _run_closed_loop(task, path)
    assert executed.arrived
    max_yaw = max(abs(t.pose.orientation.euler[2]) for t in executed.ticks)
    assert max_yaw < 0.1, "robot turned instead of crab-walking the square"
    score = score_run(path, executed)
    assert score.cte_rms < 0.15


def test_zero_gain_artifact_rejected_cleanly(tmp_path):
    """A zero plant gain must raise ValueError at start_path (the envelope/K
    divisions), not a raw ZeroDivisionError deep in the load."""
    import json

    art = json.loads(open(DEFAULT_ARTIFACT_PATH).read())
    art["plant"]["vx"]["K"] = 0.0
    bad = tmp_path / "zero_gain.json"
    bad.write_text(json.dumps(art))
    task = _task(artifact_path=str(bad))
    with pytest.raises(ValueError, match="invalid calibration artifact"):
        task.start_path(straight_rotate(length=2.0), _pose())


def test_holds_fixed_tangent_offset_around_circle():
    # At speed v the yaw error floor is ~(tau + L) * wz = 0.45 * v * kappa
    # (the artifact's documented plant floor); run at 0.3 so the floor
    # (~0.14 rad) is clearly below the bound and the CAPABILITY is what's
    # asserted, not the plant lag.
    path = circle_offset_heading(radius=1.0, offset=math.pi / 4)
    task = _task(speed=0.3)
    plant, executed = _run_closed_loop(task, path)
    assert executed.arrived
    score = score_run(path, executed)
    assert score.heading_err_rms < 0.2
    assert score.cte_rms < 0.15


# Progress indexing: no clock, no re-ramp


def test_stops_inside_the_goal_when_the_plant_runs_hot():
    """Arrival must mean AT REST inside tolerance, not merely passing through.

    Hardware 2026-07-13: the real base ran ~25% faster than the artifact's K,
    so the follower crossed the ring at cruise, sent one zero, and glided
    0.29 m past the goal — outside the benchmark's arrival circle. Model that
    mismatch (plant K=1.0 vs artifact ~0.8) and require a rest inside tolerance.
    """
    task = _task(speed=0.9)
    hot = TwistBasePlantSim(
        TwistBasePlantParams(
            vx=FopdtChannelParams(1.0, 0.3, 0.15),
            vy=FopdtChannelParams(1.0, 0.3, 0.15),
            wz=FopdtChannelParams(1.0, 0.3, 0.15),
        )
    )
    hot.reset(0.0, 0.0, 0.0, _DT)
    path = straight_rotate(length=5.0, yaw_end=0.0)
    assert task.start_path(path, _pose())
    for k in range(1200):
        out = task.compute(_state(hot.x, hot.y, hot.yaw, t=k * _DT))
        vx, vy, wz = out.velocities if out is not None else (0.0, 0.0, 0.0)
        if task.get_state() == "arrived":
            break
        hot.step(vx, vy, wz, _DT)
    assert task.get_state() == "arrived"
    goal = path.poses[-1].position
    rest_err = math.hypot(hot.x - goal.x, hot.y - goal.y)
    assert rest_err < task._config.goal_tolerance, f"rested {rest_err:.3f} m from goal"
    # Came to rest, not still coasting through.
    assert math.hypot(hot.vx, hot.vy) < 0.05


def test_replan_reprojects_without_reramp():
    """Swapping the path mid-run must keep the follower moving (re-project,
    not restart a speed ramp from zero)."""
    task = _task()
    plant = _plant()
    path_a = strafe_line(length=2.0)
    assert task.start_path(path_a, _pose(0, 0, 0))
    for k in range(30):
        out = task.compute(_state(plant.x, plant.y, plant.yaw, t=k * _DT))
        plant.step(*out.velocities, _DT)
    speed_before = math.hypot(plant.vx, plant.vy)
    assert speed_before > 0.3

    # Replan: a new strafe path starting at the robot (as a planner would emit).
    path_b = Path(poses=[_pose(plant.x, plant.y + 2.0 * i / 40, 0.0) for i in range(41)])
    task.start_path(path_b, _pose(plant.x, plant.y, plant.yaw))
    speeds = []
    for k in range(30, 40):
        out = task.compute(_state(plant.x, plant.y, plant.yaw, t=k * _DT))
        speeds.append(math.hypot(out.velocities[0], out.velocities[1]))
        plant.step(*out.velocities, _DT)
    assert min(speeds) > 0.3, f"re-ramped from rest after replan: {speeds}"


def test_reference_waits_for_a_stalled_robot():
    """If the plant is frozen the commanded velocity must stay bounded (the
    reference waits at the projection; no clock runs away)."""
    task = _task()
    task.start_path(straight_rotate(length=3.0), _pose())
    cmds = []
    for k in range(100):
        out = task.compute(_state(0.0, 0.0, 0.0, t=k * _DT))  # robot never moves
        cmds.append(out.velocities)
    vx_final = [c[0] for c in cmds[50:]]
    # Bounded: cruise + feedback clamp, gain-inverted — not growing with time.
    assert max(vx_final) == pytest.approx(max(vx_final[0:1]), rel=0.05)
    assert max(vx_final) < 1.0


# Envelope + calibration


def test_commands_respect_envelope_over_gain():
    """Commanded velocities never exceed envelope/K (the artifact ceilings)."""
    art = TuningConfig.from_json(DEFAULT_ARTIFACT_PATH)
    vp = art.velocity_profile
    task = _task(speed=2.0)  # ask far above the envelope
    plant, executed = _run_closed_loop(task, straight_rotate(length=3.0))
    for t in executed.ticks:
        assert abs(t.cmd_twist.linear.x) <= vp.max_linear_speed / art.plant.vx.K + 1e-6
        assert abs(t.cmd_twist.linear.y) <= vp.max_linear_speed / art.plant.vy.K + 1e-6
        assert abs(t.cmd_twist.angular.z) <= vp.max_angular_speed / art.plant.wz.K + 1e-6


def test_speed_regulator_slows_for_fast_yaw_stretch():
    """A commanded 90deg twist over 0.6 m demands dyaw/ds ~ 2.6 rad/m; at
    cruise 0.8 that would need wz ~ 2.1 rad/s >> the 1.18 cap. The regulator
    must slow translation through the stretch (to ~1.18/2.6 = 0.45 m/s) so
    the yaw stays feasible."""
    poses = [_pose(i * 0.1, 0.0, 0.0) for i in range(11)]
    poses += [_pose(1.0 + i * 0.1, 0.0, (math.pi / 2) * i / 6) for i in range(1, 7)]
    poses += [_pose(1.6 + i * 0.1, 0.0, math.pi / 2) for i in range(1, 11)]
    path = Path(poses=poses)
    task = _task(speed=0.8)
    v_by_x: dict[float, float] = {}

    def snoop(k, task_, plant):
        v_by_x[round(plant.x, 3)] = task_._v_path

    plant, executed = _run_closed_loop(task, path, on_tick=snoop)
    assert executed.arrived
    v_flat = max(v for x, v in v_by_x.items() if x < 0.7)
    v_twist = min(v for x, v in v_by_x.items() if 1.0 <= x <= 1.5)
    assert v_flat > 0.7, f"cruise never reached on the flat stretch: {v_flat}"
    assert v_twist < 0.5, f"regulator did not slow for the yaw stretch: {v_twist}"
    score = score_run(path, executed)
    assert score.heading_err_rms < 0.35


def test_configure_refused_while_active_and_accepts_unknown_kwargs():
    task = _task()
    assert task.configure(speed=0.7, k_angular=1.5)  # unknown kwarg ignored
    assert task._config.speed == 0.7
    task.start_path(straight_rotate(), _pose())
    assert not task.configure(speed=0.3)
