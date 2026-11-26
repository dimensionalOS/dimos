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

"""Network deadlines and patch boundaries must not leave a walking command active."""

import math

import numpy as np
import pytest

from dimos.experimental.gta_sim2.demo_interactive import bounded_velocity


@pytest.mark.parametrize("stamp", [0.0, 9.0, 11.0, math.nan, True])
def test_stale_future_and_invalid_controls_stop(stamp: float) -> None:
    velocity = bounded_velocity(
        {"issued": stamp, "forward": 1},
        10.0,
        np.zeros(2),
        0,
        np.array([-6.0, -6.0]),
        np.array([6.0, 6.0]),
    )
    assert velocity == (0, 0, 0)


def test_patch_guard_blocks_outward_but_allows_escape_in_robot_frame() -> None:
    bounds = np.array([-6.0, -6.0]), np.array([6.0, 6.0])
    # A robot facing +world Y at the northern limit cannot move forward.
    forward = bounded_velocity(
        {"issued": 10.0, "forward": 1}, 10.1, np.array([0.0, 6.0]), math.pi / 2, *bounds
    )
    np.testing.assert_allclose(forward, [0, 0, 0], atol=1e-12)
    # Backing away remains possible and yaw commands remain bounded.
    backward = bounded_velocity(
        {"issued": 10.0, "forward": -10, "turn": 10},
        10.1,
        np.array([0.0, 6.0]),
        math.pi / 2,
        *bounds,
    )
    np.testing.assert_allclose(backward, [-0.2, 0, -0.4], atol=1e-12)
