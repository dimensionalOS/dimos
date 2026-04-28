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

import pytest

from dimos.navigation.stair_navigation import generate_stair_waypoints


def test_generate_stair_waypoints_has_expected_shape() -> None:
    waypoints = generate_stair_waypoints(
        start_x=0.0,
        start_y=0.0,
        start_z=0.0,
        goal_x=2.0,
        goal_y=1.0,
        total_height=0.6,
        steps=3,
    )

    assert len(waypoints) == 3
    assert waypoints[-1].position.x == pytest.approx(2.0)
    assert waypoints[-1].position.y == pytest.approx(1.0)
    assert waypoints[-1].position.z == pytest.approx(0.6)
    assert waypoints[0].position.z == pytest.approx(0.2)
    assert waypoints[0].orientation.w != 0.0


def test_generate_stair_waypoints_rejects_non_positive_inputs() -> None:
    with pytest.raises(ValueError, match="steps"):
        generate_stair_waypoints(
            start_x=0.0,
            start_y=0.0,
            start_z=0.0,
            goal_x=1.0,
            goal_y=1.0,
            total_height=0.6,
            steps=0,
        )

    with pytest.raises(ValueError, match="non-zero"):
        generate_stair_waypoints(
            start_x=0.0,
            start_y=0.0,
            start_z=0.0,
            goal_x=1.0,
            goal_y=1.0,
            total_height=0.0,
            steps=2,
        )


def test_generate_stair_waypoints_supports_descending() -> None:
    waypoints = generate_stair_waypoints(
        start_x=2.0,
        start_y=0.0,
        start_z=1.2,
        goal_x=0.0,
        goal_y=0.0,
        total_height=-0.6,
        steps=3,
    )

    assert waypoints[-1].position.x == pytest.approx(0.0)
    assert waypoints[-1].position.z == pytest.approx(0.6)
