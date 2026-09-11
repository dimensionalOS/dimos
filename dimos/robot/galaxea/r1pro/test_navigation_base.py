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

"""Body-frame actuator integration and command-loss stopping."""

import numpy as np
import pytest

from dimos.robot.galaxea.r1pro.navigation_base import PlanarVelocityServo


def test_body_forward_at_quarter_turn_moves_world_y():
    pose = np.array([1.0, 2.0, np.pi / 2])
    servo = PlanarVelocityServo(pose)
    servo.command_twist(np.array([0.05, 0, 0]), 0)
    target = servo.step(pose, 0.1, 0.1)
    assert target[0] == pytest.approx(1)
    assert target[1] > 2
    assert target[2] == pytest.approx(np.pi / 2)


def test_expired_command_ramps_to_rest_without_new_commands():
    pose = np.zeros(3)
    servo = PlanarVelocityServo(pose)
    servo.command_twist(np.array([0.05, 0, 0.1]), 0)
    for tick in range(100):
        pose = servo.step(pose, 0.02, tick * 0.02)
    stopped = pose.copy()
    for tick in range(100, 150):
        pose = servo.step(pose, 0.02, tick * 0.02)
    assert pose == pytest.approx(stopped)
    assert 0 < pose[0] < 0.03


def test_blocked_base_integrator_and_command_speed_are_bounded():
    pose = np.zeros(3)
    servo = PlanarVelocityServo(pose)
    for tick in range(200):
        servo.command_twist(np.array([100, 100, 100]), tick * 0.02)
        target = servo.step(pose, 0.02, tick * 0.02)
    assert np.linalg.norm(servo.velocity[:2]) <= 0.08000001
    assert np.max(np.abs(target[:2])) <= 0.025
    assert abs(target[2]) <= 0.05
    servo.stop(pose)
    assert servo.step(pose, 0.02, 5) == pytest.approx(pose)


@pytest.mark.parametrize("command", [[float("nan"), 0, 0], [1, 2], [0, float("inf"), 0]])
def test_rejects_invalid_commands(command):
    with pytest.raises(ValueError, match="finite"):
        PlanarVelocityServo(np.zeros(3)).command_twist(np.asarray(command), 0)
