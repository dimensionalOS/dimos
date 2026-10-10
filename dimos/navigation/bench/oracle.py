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

"""Drives the sim Go2 along a known route from its true pose, to premap a scene or check a case."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np
from numpy.typing import NDArray

from dimos.simulation.go2_sim.world import Go2Sim

STILL = np.zeros(3)


@dataclass(frozen=True)
class Tracking:
    lookahead_m: float = 0.6
    speed: float = 0.5
    max_yaw_rate: float = 1.0
    yaw_gain: float = 2.0
    turn_in_place_rad: float = 1.0
    arrive_m: float = 0.3


class RouteTracker:
    """Pure pursuit along a route of ground points. Progress along the route never goes backward."""

    def __init__(self, route: NDArray[np.float64], tracking: Tracking = Tracking()) -> None:
        self._xy = np.asarray(route, dtype=np.float64)[:, :2]
        self._tracking = tracking
        self._i = 0

    def step(self, x: float, y: float, yaw: float) -> tuple[NDArray[np.float64], bool]:
        """The velocity command (vx, vy, wz) toward the route from this pose, and whether it has arrived."""
        here = np.array([x, y])
        ahead = self._xy[self._i :]
        self._i += int(np.argmin(np.linalg.norm(ahead - here, axis=1)))
        remaining = float(np.linalg.norm(self._xy[-1] - here))
        if remaining <= self._tracking.arrive_m:
            return STILL, True
        target = self._target(here)
        heading = math.remainder(math.atan2(*(target - here)[::-1]) - yaw, math.tau)
        wz = float(np.clip(self._tracking.yaw_gain * heading, -1, 1) * self._tracking.max_yaw_rate)
        if abs(heading) > self._tracking.turn_in_place_rad:
            return np.array([0.0, 0.0, wz]), False
        vx = (
            self._tracking.speed
            * math.cos(heading)
            * min(1.0, remaining / self._tracking.lookahead_m)
        )
        return np.array([vx, 0.0, wz]), False

    def _target(self, here: NDArray[np.float64]) -> NDArray[np.float64]:
        """The first route point at least a lookahead away, or the last one."""
        ahead = self._xy[self._i :]
        far = np.flatnonzero(np.linalg.norm(ahead - here, axis=1) >= self._tracking.lookahead_m)
        point: NDArray[np.float64] = ahead[far[0]] if len(far) else ahead[-1]
        return point


def yaw_of_rotation(rotation: NDArray[np.float64]) -> float:
    return float(math.atan2(rotation[1, 0], rotation[0, 0]))


def walk(
    sim: Go2Sim, route: NDArray[np.float64], timeout_s: float, tracking: Tracking = Tracking()
) -> bool:
    """Tick the sim along the route from where the robot stands. True once the tracker arrives."""
    tracker = RouteTracker(route, tracking)
    deadline = sim.t + timeout_s
    while sim.t < deadline:
        position, rotation = sim.base_pose()
        command, arrived = tracker.step(position[0], position[1], yaw_of_rotation(rotation))
        if arrived:
            return True
        sim.tick(command)
    return False
