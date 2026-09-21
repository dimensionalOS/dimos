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

"""Unit tests for the regulated-pure-pursuit knobs added to PathFollowerTask:
adaptive lookahead, runtime yaw-rate clamp, the forward-only contract, and the
extended configure() signature."""

from __future__ import annotations

import math

from dimos.control.benchmarking.paths import circle, straight_line
from dimos.control.benchmarking.velocity_profile import VelocityProfileConfig
from dimos.control.task import CoordinatorState, JointStateSnapshot
from dimos.control.tasks.feedforward_gain_compensator import FeedforwardGainConfig
from dimos.control.tasks.path_follower_task.path_follower_task import (
    PathFollowerTask,
    PathFollowerTaskConfig,
)
from dimos.core.global_config import global_config as _gc
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3

_JOINTS = ["go2/vx", "go2/vy", "go2/wz"]


def _state(x: float, y: float, yaw: float, t: float = 0.0) -> CoordinatorState:
    return CoordinatorState(
        joints=JointStateSnapshot(
            joint_positions={_JOINTS[0]: x, _JOINTS[1]: y, _JOINTS[2]: yaw},
            joint_velocities={_JOINTS[0]: 0.0, _JOINTS[1]: 0.0, _JOINTS[2]: 0.0},
        ),
        t_now=t,
        dt=0.1,
    )


def _task(**overrides) -> PathFollowerTask:
    cfg = PathFollowerTaskConfig(joint_names=_JOINTS, control_frequency=10.0, **overrides)
    return PathFollowerTask("t", cfg, global_config=_gc)


def _start_aligned(task: PathFollowerTask, path) -> None:
    odom = PoseStamped(
        position=Vector3(path.poses[0].position.x, path.poses[0].position.y, 0.0),
        orientation=Quaternion.from_euler(Vector3(0.0, 0.0, path.poses[0].orientation.euler[2])),
    )
    assert task.start_path(path, odom)


# adaptive lookahead


def test_adaptive_lookahead_grows_from_min_with_speed():
    """L starts at lookahead_min (v_cur == 0 on tick 0) then climbs to
    ~scale * v, staying within [min, max]."""
    task = _task(speed=0.6, lookahead_speed_scale=1.0, lookahead_min=0.3, lookahead_max=0.9)
    path = straight_line(length=5.0)
    _start_aligned(task, path)

    # Tick 0: v_cur == 0 -> L pinned to lookahead_min.
    task.compute(_state(0.0, 0.0, 0.0))
    assert math.isclose(task._distancer._lookahead_dist, 0.3, abs_tol=1e-9)

    # Subsequent ticks: L == clip(scale * v_cur, min, max). With FF off, the
    # pursuit speed on a straight aligned path is the config speed (0.6).
    task.compute(_state(0.0, 0.0, 0.0))
    assert math.isclose(task._distancer._lookahead_dist, 0.6, abs_tol=1e-6)
    assert 0.3 <= task._distancer._lookahead_dist <= 0.9


def test_adaptive_lookahead_clamped_to_max():
    task = _task(speed=2.0, lookahead_speed_scale=1.0, lookahead_min=0.3, lookahead_max=0.7)
    path = straight_line(length=5.0)
    _start_aligned(task, path)
    for _ in range(3):
        task.compute(_state(0.0, 0.0, 0.0))
    assert math.isclose(task._distancer._lookahead_dist, 0.7, abs_tol=1e-9)


def test_fixed_lookahead_when_scale_zero():
    """scale == 0 keeps the fixed lookahead_dist (no adaptation)."""
    task = _task(speed=0.6, lookahead_speed_scale=0.0, lookahead_dist=0.5)
    path = straight_line(length=5.0)
    _start_aligned(task, path)
    for _ in range(4):
        task.compute(_state(0.0, 0.0, 0.0))
        assert math.isclose(task._distancer._lookahead_dist, 0.5, abs_tol=1e-9)


# yaw-rate clamp


def test_max_yaw_rate_clamps_commanded_wz():
    """A tight curve drives |wz| past the cap; the clamp holds it at the cap."""
    cap = 0.25
    task = _task(speed=0.9, max_yaw_rate=cap)
    path = circle(radius=0.5)  # high curvature -> large desired wz
    _start_aligned(task, path)
    saw_cap = False
    for k in range(15):
        out = task.compute(_state(0.0, 0.0, 0.0, t=k * 0.1))
        if out is not None:
            wz = out.velocities[2]
            assert abs(wz) <= cap + 1e-9
            if abs(abs(wz) - cap) < 1e-6:
                saw_cap = True
    assert saw_cap, "expected the yaw clamp to bind on a tight circle"


def test_no_yaw_clamp_when_none():
    task = _task(speed=0.9, max_yaw_rate=None)
    path = circle(radius=0.5)
    _start_aligned(task, path)
    # Without a clamp the bare PController clips wz to +/- speed (0.9), so
    # |wz| can exceed the 0.25 a clamp would have imposed.
    peak = 0.0
    for k in range(15):
        out = task.compute(_state(0.0, 0.0, 0.0, t=k * 0.1))
        if out is not None:
            peak = max(peak, abs(out.velocities[2]))
    assert peak > 0.25


# forward-only contract


def test_forward_only_holds_over_a_run():
    """Pursuit never strafes or reverses: vy == 0 and vx >= 0 every tick."""
    task = _task(
        speed=0.7,
        forward_only=True,
        lookahead_speed_scale=1.6,
        max_yaw_rate=1.18,
        ff_config=FeedforwardGainConfig(K_vx=0.8, K_vy=0.8, K_wz=0.9),
        velocity_profile_config=VelocityProfileConfig(max_linear_speed=0.7, min_speed=0.2),
    )
    path = circle(radius=1.0)
    _start_aligned(task, path)
    for k in range(30):
        out = task.compute(_state(0.0, 0.0, 0.0, t=k * 0.1))
        if out is None:
            continue
        vx, vy, wz = out.velocities
        assert vy == 0.0
        assert vx >= 0.0


# configure() signature


def test_configure_sets_rpp_knobs_and_ignores_unknown():
    task = _task()
    ok = task.configure(
        lookahead_min=0.2,
        lookahead_max=0.8,
        lookahead_speed_scale=1.4,
        max_yaw_rate=1.1,
        forward_only=True,
        some_trajtrack_only_kwarg=123,  # unknown -> logged, not fatal
    )
    assert ok
    assert task._config.lookahead_min == 0.2
    assert task._config.lookahead_max == 0.8
    assert task._config.lookahead_speed_scale == 1.4
    assert task._config.max_yaw_rate == 1.1
    assert task._config.forward_only is True


def test_configure_max_yaw_rate_none_clears_clamp():
    """max_yaw_rate uses a sentinel, so passing None explicitly clears it."""
    task = _task(max_yaw_rate=1.18)
    assert task.configure(max_yaw_rate=None)
    assert task._config.max_yaw_rate is None
    # Omitting it leaves the value untouched.
    task2 = _task(max_yaw_rate=1.18)
    assert task2.configure(speed=0.5)
    assert task2._config.max_yaw_rate == 1.18
