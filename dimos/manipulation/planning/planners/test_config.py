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

"""Tests for typed manipulation planner configuration."""

from pydantic import ValidationError
import pytest

from dimos.manipulation.planning.planners.config import RoboPlanCartesianPathConfig


def test_roboplan_cartesian_path_config_defaults_to_time_optimal_and_allows_bounded() -> None:
    assert RoboPlanCartesianPathConfig().speed_mode == "time_optimal"
    assert RoboPlanCartesianPathConfig(speed_mode="bounded").speed_mode == "bounded"


@pytest.mark.parametrize(
    "path_config",
    [
        {"dt": 0.0},
        {"velocity_scale": 1.1},
        {"acceleration_scale": 0.0},
        {"toppra_blend_deviation": -0.01},
    ],
)
def test_roboplan_cartesian_path_config_rejects_invalid_limits(
    path_config: dict[str, float],
) -> None:
    with pytest.raises(ValidationError):
        RoboPlanCartesianPathConfig(**path_config)  # type: ignore[arg-type]
