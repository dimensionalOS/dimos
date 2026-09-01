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

"""Typed configuration for manipulation trajectory parametrization."""

from typing import Annotated, Literal

from pydantic import Field

from dimos.protocol.service.spec import BaseConfig


class SimpleTrapezoidParametrizationConfig(BaseConfig):
    """Configuration for the compatibility segmented-trapezoid backend."""

    backend: Literal["simple_trapezoid"] = "simple_trapezoid"
    velocity_scale: float = Field(default=1.0, gt=0.0, le=1.0)
    acceleration_scale: float = Field(default=1.0, gt=0.0, le=1.0)
    points_per_segment: int = Field(default=50, ge=1)


class RoboPlanTOPPRAParametrizationConfig(BaseConfig):
    """Configuration for RoboPlan TOPP-RA path parametrization."""

    backend: Literal["roboplan_toppra"] = "roboplan_toppra"
    output_period: float = Field(default=0.01, gt=0.0)
    velocity_scale: float = Field(default=1.0, gt=0.0, le=1.0)
    acceleration_scale: float = Field(default=1.0, gt=0.0, le=1.0)


TrajectoryParametrizationConfig = Annotated[
    SimpleTrapezoidParametrizationConfig | RoboPlanTOPPRAParametrizationConfig,
    Field(discriminator="backend"),
]
