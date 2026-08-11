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

from __future__ import annotations

import math

import numpy as np
import pytest

from dimos.robot.unitree.g1.manip_last_mile import (
    MAX_ANGULAR,
    MAX_LATERAL,
    MAX_LINEAR,
    Twist2D,
    arrived,
    last_mile_twist,
    servo_step,
)
from dimos.robot.unitree.g1.manip_stance import PourReachMap, pot_in_base_frame


def _reach_map():
    return PourReachMap.load()


def test_walks_forward_when_the_pot_is_too_far_ahead():
    twist = last_mile_twist(seen=(1.2, 0.0), goal=(0.35, 0.0))
    assert twist.vx > 0.0
    assert twist.vy == pytest.approx(0.0)


def test_backs_off_when_it_has_walked_too_close():
    twist = last_mile_twist(seen=(0.15, 0.0), goal=(0.35, 0.0))
    assert twist.vx < 0.0


def test_strafes_towards_the_side_the_arm_wants_the_pot():
    # Goal offset is on the robot's right, the pot is dead ahead: the robot
    # has to shuffle left so the pot ends up on its right.
    twist = last_mile_twist(seen=(0.35, 0.0), goal=(0.35, -0.15))
    assert twist.vy > 0.0
    assert twist.vx == pytest.approx(0.0)


def test_turns_to_put_the_pot_at_the_goal_bearing():
    assert last_mile_twist(seen=(0.6, 0.6), goal=(0.35, 0.0)).wz > 0.0
    assert last_mile_twist(seen=(0.6, -0.6), goal=(0.35, 0.0)).wz < 0.0
    assert last_mile_twist(seen=(0.6, 0.0), goal=(0.35, 0.0)).wz == pytest.approx(0.0)
    # An off-nose goal is the whole point: sitting already at the goal's
    # bearing must not command a turn back towards the nose.
    goal = (0.33, -0.22)
    assert last_mile_twist(seen=(0.66, -0.44), goal=goal).wz == pytest.approx(0.0)
    # Pot dead ahead but wanted off to the right: turning left swings the
    # robot's nose past it, which is what carries the pot to that side.
    assert last_mile_twist(seen=(0.5, 0.0), goal=goal).wz > 0.0


def test_commands_stay_inside_the_gait_limits():
    twist = last_mile_twist(seen=(8.0, -8.0), goal=(0.35, 0.0))
    assert abs(twist.vx) <= MAX_LINEAR
    assert abs(twist.wz) <= MAX_ANGULAR
    # Sideways is capped harder than forwards: a sustained fast strafe puts
    # the robot on the floor, and the policy has no way back up.
    assert abs(twist.vy) <= MAX_LATERAL < MAX_LINEAR


def test_small_errors_still_command_a_walkable_speed():
    # A 2 cm error must not turn into a command the gait cannot execute.
    twist = last_mile_twist(seen=(0.37, 0.0), goal=(0.35, 0.0))
    assert twist.vx == 0.0 or abs(twist.vx) >= 0.05


def test_arrival_needs_margin_not_just_reachability():
    reach = _reach_map()
    inside = reach.best_offset(margin_cells=3)
    assert arrived(inside, reach)
    # A cell right on the region's edge is reachable but not somewhere to
    # stop: the robot's own stopping error would carry the pot out of it.
    edge = next(
        (reach.x0 + ix * reach.cell, reach.y0 + iy * reach.cell)
        for iy, ix in np.argwhere(reach.reachable)
        if reach.margin((reach.x0 + ix * reach.cell, reach.y0 + iy * reach.cell)) == 1
    )
    assert not arrived(edge, reach)


def test_does_not_stop_sideways_on_to_the_pot():
    reach = _reach_map()
    # Manufacture a sighting deep inside the region but far off the nose by
    # rotating a good offset about the robot; reach is the same, pose is not.
    good = reach.best_offset(margin_cells=3)
    radius = math.hypot(*good)
    sideways = (radius * math.cos(math.radians(80)), -radius * math.sin(math.radians(80)))
    if reach.contains(sideways, margin_cells=2):
        assert not arrived(sideways, reach)


def test_servo_stops_once_it_has_arrived():
    reach = _reach_map()
    assert servo_step(reach.best_offset(margin_cells=3), (0.35, -0.15), reach).is_stop
    assert not servo_step((1.5, 0.2), (0.35, -0.15), reach).is_stop


def test_servo_converges_from_a_metre_out():
    """The loop closes: stepping the robot by its own commands arrives."""
    reach = _reach_map()
    goal = reach.best_offset(margin_cells=3)
    pot = (2.4, 0.9)
    base, yaw = [0.9, 0.35], 0.0
    dt = 0.25
    for _ in range(400):
        seen = pot_in_base_frame(pot, (base[0], base[1]), yaw)
        twist = servo_step(seen, goal, reach)
        if twist.is_stop:
            break
        base[0] += (math.cos(yaw) * twist.vx - math.sin(yaw) * twist.vy) * dt
        base[1] += (math.sin(yaw) * twist.vx + math.cos(yaw) * twist.vy) * dt
        yaw += twist.wz * dt
    else:
        pytest.fail("last-mile servo never arrived")
    assert arrived(pot_in_base_frame(pot, (base[0], base[1]), yaw), reach)


def test_stop_twist_is_all_zero():
    assert Twist2D().is_stop
    assert not Twist2D(vx=0.1).is_stop
