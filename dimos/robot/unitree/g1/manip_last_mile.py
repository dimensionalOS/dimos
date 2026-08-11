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

"""Walking the G1 the last metre, until the pot is somewhere it can pour.

The goal is a region, not a point: the arm does not care where the robot
stands as long as the pot ends up inside the reach map's green core with
room to spare (:mod:`manip_stance`). Stopping at "close enough to a region"
rather than "converged on a point" is what keeps the walk short, because a
legged base cannot hold a centimetre-accurate position anyway.

The control law here is deliberately a P controller on the pot's position
in the robot's own frame. Nothing about it is specific to the simulator,
but it does assume the caller feeds it fresh sightings: the twist is a
velocity command that the WBC keeps executing for a second after the last
message, so a stalled feed walks the robot into furniture.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from dimos.robot.unitree.g1.manip_stance import PourReachMap, wrap_angle

# The policy takes vx/vy/wz directly with no clamping anywhere downstream,
# and the demo wants a shuffle rather than a stride, so the limits live here.
MAX_LINEAR = 0.25
# Sideways is the gait's weak axis: a sustained 0.25 m/s strafe topples the
# robot outright, where the same speed forwards walks fine.
MAX_LATERAL = 0.12
MAX_ANGULAR = 0.4
# Below this the gait produces foot scuffing rather than travel.
MIN_LINEAR = 0.06
MIN_ANGULAR = 0.08

LINEAR_GAIN = 0.8
ANGULAR_GAIN = 1.0


@dataclass(frozen=True)
class Twist2D:
    """A planar velocity command: forward, left, and turn."""

    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0

    @property
    def is_stop(self) -> bool:
        return self.vx == 0.0 and self.vy == 0.0 and self.wz == 0.0


def _deadband(value: float, floor: float, ceiling: float) -> float:
    """Clamp to the gait's usable range, or to a full stop below it."""
    if abs(value) < 1e-9:
        return 0.0
    magnitude = min(max(abs(value), floor), ceiling)
    return float(np.sign(value) * magnitude)


def last_mile_twist(seen: tuple[float, float], goal: tuple[float, float]) -> Twist2D:
    """Twist that moves the pot from where the robot sees it towards ``goal``.

    Both are the pot's XY in the robot's base frame, so the error is
    already in the frame the twist is expressed in: driving +x carries the
    robot forward, which moves the pot's sighting backwards. Hence the sign.

    Turning aims the pot at the goal's *bearing*, not at the robot's nose:
    the goal sits off to the arm's side, so turning to face the pot dead-on
    would fight the strafe that puts it there.
    """
    error_x = seen[0] - goal[0]
    error_y = seen[1] - goal[1]
    heading_error = wrap_angle(
        float(np.arctan2(seen[1], seen[0])) - float(np.arctan2(goal[1], goal[0]))
    )
    return Twist2D(
        vx=_deadband(LINEAR_GAIN * error_x, MIN_LINEAR, MAX_LINEAR),
        vy=_deadband(LINEAR_GAIN * error_y, MIN_LINEAR, MAX_LATERAL),
        wz=_deadband(ANGULAR_GAIN * heading_error, MIN_ANGULAR, MAX_ANGULAR),
    )


def arrived(
    seen: tuple[float, float],
    reach_map: PourReachMap,
    margin_cells: int = 3,
    max_bearing: float = np.deg2rad(45.0),
) -> bool:
    """Is the pot reachable from here, with room for the stop to overshoot?

    The margin is not paranoia: the gait undershoots and stops where it
    likes, and cells at the region's edge are exactly where the IK solves
    to within 2 cm of the target instead of the 1 cm it needs.

    The bearing check is not about reach -- the map already covers that --
    but about the pour poses, which are commanded with the tool pointing at
    the pot; a stance sideways-on to it pours across the robot's body.
    """
    if not reach_map.contains(seen, margin_cells=margin_cells):
        return False
    return abs(float(np.arctan2(seen[1], seen[0]))) <= max_bearing


def servo_step(
    seen: tuple[float, float],
    goal: tuple[float, float],
    reach_map: PourReachMap,
    margin_cells: int = 3,
) -> Twist2D:
    """One tick of the last mile: stop if arrived, otherwise close the gap."""
    if arrived(seen, reach_map, margin_cells=margin_cells):
        return Twist2D()
    return last_mile_twist(seen, goal)
