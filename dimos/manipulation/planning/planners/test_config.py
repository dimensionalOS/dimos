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

from pydantic import TypeAdapter, ValidationError
import pytest

from dimos.manipulation.planning.planners.config import (
    ManipulationPlannerConfig,
    RoboPlanPlannerConfig,
    RRTConnectPlannerConfig,
    planner_config_from_name,
)


def test_planner_config_discriminates_backend() -> None:
    adapter = TypeAdapter(ManipulationPlannerConfig)

    assert adapter.validate_python({"backend": "rrt_connect"}) == RRTConnectPlannerConfig()
    assert adapter.validate_python(
        {
            "backend": "roboplan",
            "linear_cartesian": {"dt": 0.02, "max_linear_speed": 0.2},
        }
    ) == RoboPlanPlannerConfig(
        linear_cartesian={"dt": 0.02, "max_linear_speed": 0.2}  # type: ignore[arg-type]
    )


@pytest.mark.parametrize(
    "linear_config",
    [
        {"dt": 0.0},
        {"velocity_scale": 1.1},
        {"acceleration_scale": 0.0},
        {"limit_ratio_tolerance": 0.99},
        {"max_attempts_per_step": 0},
    ],
)
def test_roboplan_linear_cartesian_config_rejects_invalid_limits(
    linear_config: dict[str, float],
) -> None:
    with pytest.raises(ValidationError):
        RoboPlanPlannerConfig(linear_cartesian=linear_config)  # type: ignore[arg-type]


def test_planner_config_from_legacy_name() -> None:
    assert planner_config_from_name("rrt_connect") == RRTConnectPlannerConfig()
    assert planner_config_from_name("roboplan") == RoboPlanPlannerConfig()
