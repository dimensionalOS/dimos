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

"""Tests for trajectory-parametrization startup configuration."""

from pydantic import TypeAdapter, ValidationError
import pytest

from dimos.manipulation.planning.trajectory_generator.config import (
    RoboPlanTOPPRAParametrizationConfig,
    SimpleTrapezoidParametrizationConfig,
    TrajectoryParametrizationConfig,
)


@pytest.mark.parametrize(
    ("payload", "expected_type"),
    [
        ({"backend": "simple_trapezoid"}, SimpleTrapezoidParametrizationConfig),
        ({"backend": "roboplan_toppra"}, RoboPlanTOPPRAParametrizationConfig),
    ],
)
def test_trajectory_parametrization_config_selects_one_backend(
    payload: dict[str, str],
    expected_type: type[object],
) -> None:
    result = TypeAdapter(TrajectoryParametrizationConfig).validate_python(payload)

    assert isinstance(result, expected_type)


@pytest.mark.parametrize(
    "payload",
    [
        {"backend": "unknown"},
        {"backend": "simple_trapezoid", "velocity_scale": 0.0},
        {"backend": "simple_trapezoid", "acceleration_scale": 1.01},
        {"backend": "simple_trapezoid", "points_per_segment": 0},
        {"backend": "roboplan_toppra", "output_period": 0.0},
        {"backend": "roboplan_toppra", "velocity_scale": 1.01},
        {"backend": "roboplan_toppra", "acceleration_scale": -0.1},
        {"backend": "roboplan_toppra", "max_adaptive_iterations": 0},
        {"backend": "roboplan_toppra", "max_adaptive_step_size": 0.0},
        {"backend": "roboplan_toppra", "max_blend_deviation": -0.1},
    ],
)
def test_trajectory_parametrization_config_rejects_invalid_options(
    payload: dict[str, object],
) -> None:
    with pytest.raises(ValidationError):
        TypeAdapter(TrajectoryParametrizationConfig).validate_python(payload)
