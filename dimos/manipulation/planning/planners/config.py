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

"""Typed configuration models for manipulation planner backends."""

from __future__ import annotations

from typing import Annotated, Literal

from pydantic import Field

from dimos.protocol.service.spec import BaseConfig


class RRTConnectPlannerConfig(BaseConfig):
    """Configuration selecting the backend-agnostic RRT-Connect planner."""

    backend: Literal["rrt_connect"] = "rrt_connect"


class RoboPlanLinearCartesianConfig(BaseConfig):
    """Official RoboPlan bounded Cartesian path planner options."""

    dt: float = Field(default=0.01, gt=0.0)
    max_linear_speed: float = Field(default=0.1, gt=0.0)
    max_angular_speed: float = Field(default=0.5, gt=0.0)
    max_linear_acceleration: float = Field(default=0.5, gt=0.0)
    max_angular_acceleration: float = Field(default=2.5, gt=0.0)
    max_position_error: float = Field(default=0.005, gt=0.0)
    max_orientation_error: float = Field(default=0.01, gt=0.0)
    velocity_scale: float = Field(default=1.0, gt=0.0, le=1.0)
    acceleration_scale: float = Field(default=1.0, gt=0.0, le=1.0)
    limit_ratio_tolerance: float = Field(default=1.05, ge=1.0)
    position_limit_gain: float = Field(default=1.0, gt=0.0)
    max_attempts_per_step: int = Field(default=16, ge=1)


class RoboPlanPlannerConfig(BaseConfig):
    """Configuration for scene-backed RoboPlan planning."""

    backend: Literal["roboplan"] = "roboplan"
    linear_cartesian: RoboPlanLinearCartesianConfig = Field(
        default_factory=RoboPlanLinearCartesianConfig
    )


ManipulationPlannerConfig = Annotated[
    RRTConnectPlannerConfig | RoboPlanPlannerConfig,
    Field(discriminator="backend"),
]
