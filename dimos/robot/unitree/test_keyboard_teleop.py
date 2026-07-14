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

import pygame
import pytest

from dimos.robot.unitree.keyboard_teleop import twist_from_keys


@pytest.mark.parametrize(
    ("keys", "expected"),
    [
        pytest.param({pygame.K_w}, (0.5, 0.0, 0.0), id="w-forward"),
        pytest.param({pygame.K_UP}, (0.5, 0.0, 0.0), id="up-forward"),
        pytest.param({pygame.K_s}, (-0.5, 0.0, 0.0), id="s-backward"),
        pytest.param({pygame.K_DOWN}, (-0.5, 0.0, 0.0), id="down-backward"),
        pytest.param({pygame.K_a}, (0.0, 0.0, 0.8), id="a-turn-left"),
        pytest.param({pygame.K_LEFT}, (0.0, 0.0, 0.8), id="left-turn-left"),
        pytest.param({pygame.K_d}, (0.0, 0.0, -0.8), id="d-turn-right"),
        pytest.param({pygame.K_RIGHT}, (0.0, 0.0, -0.8), id="right-turn-right"),
        pytest.param({pygame.K_q}, (0.0, 0.5, 0.0), id="q-strafe-left"),
        pytest.param({pygame.K_e}, (0.0, -0.5, 0.0), id="e-strafe-right"),
        pytest.param(
            {pygame.K_LALT, pygame.K_a},
            (0.0, 0.5, 0.0),
            id="alt-a-strafe-left",
        ),
        pytest.param(
            {pygame.K_RALT, pygame.K_RIGHT},
            (0.0, -0.5, 0.0),
            id="alt-right-strafe-right",
        ),
        pytest.param({pygame.K_w, pygame.K_s}, (0.0, 0.0, 0.0), id="forward-opposites-cancel"),
        pytest.param({pygame.K_q, pygame.K_e}, (0.0, 0.0, 0.0), id="strafe-opposites-cancel"),
        pytest.param({pygame.K_a, pygame.K_d}, (0.0, 0.0, 0.0), id="turn-opposites-cancel"),
        pytest.param(
            {pygame.K_LALT, pygame.K_a, pygame.K_d},
            (0.0, 0.0, 0.0),
            id="alt-strafe-opposites-cancel",
        ),
    ],
)
def test_twist_from_movement_keys(keys: set[int], expected: tuple[float, float, float]) -> None:
    twist = twist_from_keys(keys, 0.5, 0.8, 2.0, 0.5)
    assert (twist.linear.x, twist.linear.y, twist.angular.z) == expected


@pytest.mark.parametrize(
    ("keys", "expected"),
    [
        pytest.param({pygame.K_w, pygame.K_LSHIFT}, (1.0, 0.0, 0.0), id="shift-boosts-move"),
        pytest.param({pygame.K_q, pygame.K_RCTRL}, (0.0, 0.25, 0.0), id="ctrl-slows-strafe"),
        pytest.param({pygame.K_d, pygame.K_RSHIFT}, (0.0, 0.0, -1.6), id="shift-boosts-turn"),
        pytest.param(
            {pygame.K_w, pygame.K_LSHIFT, pygame.K_LCTRL},
            (1.0, 0.0, 0.0),
            id="shift-takes-precedence",
        ),
    ],
)
def test_twist_from_keys_applies_speed_modifier(
    keys: set[int], expected: tuple[float, float, float]
) -> None:
    twist = twist_from_keys(keys, 0.5, 0.8, 2.0, 0.5)
    assert (twist.linear.x, twist.linear.y, twist.angular.z) == expected
