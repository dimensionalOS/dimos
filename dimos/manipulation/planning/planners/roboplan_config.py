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

"""Typed configuration models for RoboPlan-backed planning."""

from __future__ import annotations

from typing import Literal

from pydantic import Field

from dimos.protocol.service.spec import BaseConfig


class RoboPlanCartesianPathConfig(BaseConfig):
    """Runtime options for the official RoboPlan Cartesian path planner."""

    backend: Literal["roboplan"] = "roboplan"
    speed_mode: Literal["bounded", "time_optimal"] = "time_optimal"
    dt: float = Field(default=0.01, gt=0.0)
    max_linear_speed: float = Field(default=0.1, gt=0.0)
    max_angular_speed: float = Field(default=0.5, gt=0.0)
    max_linear_acceleration: float = Field(default=0.5, gt=0.0)
    max_angular_acceleration: float = Field(default=2.5, gt=0.0)
    max_position_error: float = Field(default=0.005, gt=0.0)
    max_orientation_error: float = Field(default=0.01, gt=0.0)
    position_cost: float = Field(default=1.0, ge=0.0)
    orientation_cost: float = Field(default=1.0, ge=0.0)
    task_gain: float = Field(default=1.0, gt=0.0)
    lm_damping: float = Field(default=0.01, ge=0.0)
    regularization: float = Field(default=1e-6, ge=0.0)
    config_task_weight: float = Field(default=0.05, ge=0.0)
    velocity_scale: float = Field(default=1.0, gt=0.0, le=1.0)
    acceleration_scale: float = Field(default=1.0, gt=0.0, le=1.0)
    toppra_blend_deviation: float = Field(default=0.05, ge=0.0)
    position_limit_gain: float = Field(default=1.0, gt=0.0, le=1.0)


class RoboPlanPathShortcuttingConfig(BaseConfig):
    """Configuration for RoboPlan's native joint-path shortcutter."""

    enabled: bool = True
    max_step_size: float = Field(default=0.05, gt=0.0)
    max_iters: int = Field(default=100, ge=0)
    seed: int = 0
    max_convergence_iters: int = Field(default=20, ge=0)
    redundant_removal_iters: int = Field(default=20, ge=0)


class RoboPlanPlannerConfig(BaseConfig):
    """Configuration for scene-backed RoboPlan planning."""

    backend: Literal["roboplan"] = "roboplan"
    path_shortcutting: RoboPlanPathShortcuttingConfig = Field(
        default_factory=RoboPlanPathShortcuttingConfig
    )
