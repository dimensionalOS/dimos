# Copyright 2025 Dimensional Inc.
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
import time
from datetime import datetime
from dataclasses import dataclass
from typing import Callable, Optional

import reactivex as rx
from reactivex.observable import Observable
from plum import dispatch
from reactivex import operators as ops

from dimos.robot.local_planner.local_planner import LocalPlanner
from dimos.types.costmap import Costmap
from dimos.types.path import Path
from dimos.types.position import Position
from dimos.types.vector import Vector, VectorLike, to_vector
from dimos.utils.logging_config import setup_logger
from dimos.utils.threadpool import get_scheduler

logger = setup_logger("dimos.robot.unitree.global_planner")


# experimental stream based vector based planner
# not production ready - just a sketch
@dataclass
class SimplePlanner(LocalPlanner):
    get_costmap: Callable[[], Costmap]
    get_robot_pos: Callable[[], Position]
    goal: Optional[Vector] = None
    path: Optional[Path] = None
    speed: float = 0.5
    slowdown_range = 0.5

    def consume_path_stream(self, observable: Observable[Path]) -> bool:
        observable.subscribe(self.set_path)
        return True

    def set_path(self, path: Path) -> bool:
        logger.info(f"Received new path: {path}")
        self.path = path
        if self.path:
            self.set_goal(path.head())

    def set_goal(self, goal: VectorLike, stop_event=None, goal_theta=None) -> bool:
        self.goal = to_vector(goal).to_2d()
        self.vis("local_goal", self.goal)
        logger.info(f"Setting goal: {self.goal}")

    def speed_ctrl(self, direction: Vector):
        dist = direction.length()
        if dist >= self.slowdown_range:
            f = 1.0
        else:
            f = dist / self.slowdown_range

        return self.speed * f

    def yaw_ctrl(self, direction, k_p: float = 0.3, omega_max: float = 1):
        alpha = math.atan2(-direction.y, direction.x)
        omega = max(-omega_max, min(omega_max, k_p * alpha))
        return Vector(0.0, 0.0, omega)

    def pop_path(self):
        self.set_path(self.path.tail())
        return False

    # dumb path popping filter (needs change)
    def filter(self, direction):
        length = direction.length()
        if self.path:
            if length < 0.5:
                self.pop_path()
                return False
        else:
            if length < 0.2:
                return False

        return direction

    def ctrl(self, direction):
        jaw = self.yaw_ctrl(direction)
        if jaw.length() > 0.25:
            return jaw
        else:
            return (direction.normalize() * self.speed_ctrl(direction)) + jaw

    def get_move_stream(self, frequency: float = 30.0) -> Observable[Vector]:
        return rx.interval(1.0 / frequency, scheduler=get_scheduler()).pipe(
            ops.filter(lambda _: (self.goal is not None) or (self.path is not None)),
            ops.map(lambda _: self.get_robot_pos()),
            ops.map(lambda odom: odom.vector_to(self.goal)),
            ops.filter(self.filter),
            ops.map(self.ctrl),
            # somewhere here we should detect potential obstacles in direction of travel
            #            ops.map(lambda direction: direction.normalize() * self.speed_ctrl(direction)),
            #            ops.map(lambda direction: direction + self.yaw_ctrl(direction)),
            ops.map(lambda direction: Vector(-direction.y, direction.x, direction.z)),
        )
