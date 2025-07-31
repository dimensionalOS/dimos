#!/usr/bin/env python3

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
import traceback
from typing import Callable, Optional

import reactivex as rx
from plum import dispatch
from reactivex import operators as ops

from dimos.core import TF, In, Module, Out, rpc

# from dimos.robot.local_planner.local_planner import LocalPlanner
from dimos.msgs.geometry_msgs import (
    Pose,
    PoseLike,
    PoseStamped,
    Transform,
    Twist,
    Vector3,
    VectorLike,
    to_pose,
)
from dimos.msgs.nav_msgs import Path
from dimos.msgs.tf2_msgs import TFMessage
from dimos.types.costmap import Costmap
from dimos.utils.logging_config import setup_logger
from dimos.utils.threadpool import get_scheduler

logger = setup_logger("dimos.robot.unitree.local_planner")


class SimplePlanner(Module):
    path: In[Path] = None
    movecmd: Out[Vector3] = None
    speed: float = 1.5

    tf: TF
    current_path: Optional[Path] = None

    def __init__(self):
        Module.__init__(self)
        self.tf = TF()

    def poppath(self, direction):
        if direction.position.length() < 1.0:
            if self.current_path and len(self.current_path) > 1:
                self.current_path = self.current_path.tail()
            return
        print(direction.position)
        return direction.position

    def move_stream(self, frequency: float = 10.0) -> rx.Observable:
        return rx.interval(1.0 / frequency).pipe(
            ops.filter(
                lambda _: self.current_path and len(self.current_path)
            ),  # do we have a target path?
            ops.map(lambda _: self.tf.get("base_link", "world")),
            ops.filter(lambda _: _),  # do we have a transform?
            ops.map(lambda tf: self.current_path.head() @ tf),
            ops.map(self.poppath),
            ops.filter(lambda _: _),
            ops.map(lambda direction_vector: direction_vector.normalize() * self.speed),
        )

    def set_goal(self, path: Path):
        self.current_path = path

    @rpc
    def start(self):
        self.path.subscribe(self.set_goal)

        def pub(v):
            print(v)
            self.movecmd.publish(v)

        self.move_stream(frequency=50).subscribe(pub)
