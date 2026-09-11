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

from dimos.manipulation.planning.planners.roboplan_config import (
    RoboPlanCartesianPathConfig as _RoboPlanCartesianPathConfig,
    RoboPlanPlannerConfig as _RoboPlanPlannerConfig,
)
from dimos.protocol.service.spec import BaseConfig


class RRTConnectPlannerConfig(BaseConfig):
    """Configuration selecting the backend-agnostic RRT-Connect planner."""

    backend: Literal["rrt_connect"] = "rrt_connect"


CartesianPathConfig = _RoboPlanCartesianPathConfig


ManipulationPlannerConfig = Annotated[
    RRTConnectPlannerConfig | _RoboPlanPlannerConfig,
    Field(discriminator="backend"),
]
