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

import numpy as np

from dimos.core.global_config import GlobalConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.navigation.replanning_a_star.controllers import PController
from dimos.navigation.replanning_a_star.global_planner import GlobalPlanner
from dimos.navigation.replanning_a_star.replan_limiter import ReplanLimiter


def _pose(yaw_degrees: float = 0.0) -> PoseStamped:
    return PoseStamped(
        position=Vector3(),
        orientation=Quaternion.from_euler(Vector3(0.0, 0.0, math.radians(yaw_degrees))),
    )


def test_large_heading_error_rotates_before_advancing() -> None:
    controller = PController(GlobalConfig(), speed=0.55, control_frequency=10.0)
    target = np.array(
        [math.cos(math.radians(75.0)), math.sin(math.radians(75.0))],
        dtype=np.float64,
    )

    command = controller.advance(target, _pose())

    assert command.linear.x == 0.0
    assert command.angular.z > 0.0


def test_moderate_heading_error_preserves_forward_motion() -> None:
    controller = PController(GlobalConfig(), speed=0.55, control_frequency=10.0)
    target = np.array(
        [math.cos(math.radians(45.0)), math.sin(math.radians(45.0))],
        dtype=np.float64,
    )

    command = controller.advance(target, _pose())

    assert command.linear.x > 0.0
    assert command.angular.z > 0.0


def test_stuck_detection_waits_six_seconds() -> None:
    assert GlobalPlanner._stuck_time_window == 6.0


def test_replan_limiter_allows_eight_local_retries() -> None:
    limiter = ReplanLimiter()
    position = Vector3()

    for _ in range(8):
        assert limiter.can_retry(position)
        limiter.will_retry()

    assert not limiter.can_retry(position)
