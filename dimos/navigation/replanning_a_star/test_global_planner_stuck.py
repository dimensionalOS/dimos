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

from dimos.navigation.replanning_a_star.global_planner import _has_stuck_path_progress


@pytest.mark.parametrize(
    ("elapsed_s", "progress_m", "command_m_s", "goal_distance_m", "expected"),
    [
        (8.0, 0.19, 0.2, 1.0, True),
        (8.0, 0.2, 0.2, 1.0, False),
        (8.0, 0.19, 0.1, 1.0, False),
        (8.0, 0.19, 0.2, 0.5, False),
        (7.9, 0.19, 0.2, 1.0, False),
    ],
)
def test_stuck_requires_no_path_progress_while_commanded_to_move(
    elapsed_s: float,
    progress_m: float,
    command_m_s: float,
    goal_distance_m: float,
    expected: bool,
) -> None:
    assert (
        _has_stuck_path_progress(
            elapsed_s=elapsed_s,
            path_progress_delta_m=progress_m,
            command_linear_x_m_s=command_m_s,
            goal_distance_m=goal_distance_m,
            window_s=8.0,
            min_path_progress_m=0.2,
            min_command_linear_m_s=0.1,
            goal_exclusion_m=0.5,
        )
        is expected
    )
