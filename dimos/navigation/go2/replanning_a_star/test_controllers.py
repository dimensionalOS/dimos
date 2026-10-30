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

import numpy as np
import pytest

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.navigation.go2.replanning_a_star.controllers import PController


@pytest.mark.parametrize("simulation", [False, True])
@pytest.mark.parametrize(
    ("yaw_error", "expected"), [(0, 0), (0.1, 0.2), (-0.1, -0.2), (3, 0.55), (-3, -0.55)]
)
def test_in_place_rotation_respects_normal_turn_limits(simulation, yaw_error, expected):
    controller = PController(simulation=simulation, speed=0.55, control_frequency=10)

    command = controller.rotate(yaw_error)

    assert command.angular.z == pytest.approx(expected)
    assert command.linear.length() == 0


@pytest.mark.parametrize("simulation", [False, True])
def test_turning_toward_a_path_behind_the_robot_respects_speed_cap(simulation):
    controller = PController(simulation=simulation, speed=0.3, control_frequency=10)

    command = controller.advance(np.array([-1.0, 0.1]), PoseStamped(frame_id="odom"))

    assert command.angular.z == pytest.approx(0.3)
    assert command.linear.length() == 0
