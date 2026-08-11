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

"""Configuration models for manipulation kinematics backends."""

from __future__ import annotations

from typing import Annotated, Literal

from pydantic import Field

from dimos.protocol.service.spec import BaseConfig


class JacobianKinematicsConfig(BaseConfig):
    """Configuration for the backend-agnostic Jacobian IK solver."""

    backend: Literal["jacobian"] = "jacobian"


class DrakeOptimizationKinematicsConfig(BaseConfig):
    """Configuration for the Drake mathematical-program IK solver."""

    backend: Literal["drake_optimization"] = "drake_optimization"


class PinkKinematicsConfig(BaseConfig):
    """Configuration for the Pink task/QP IK solver."""

    backend: Literal["pink"] = "pink"
    solver: str = "proxqp"
    dt: float = 0.05
    max_iterations: int = 200
    # Near a reach-envelope edge the arm Jacobian goes near-singular and
    # proxqp stops converging below ~1e-7; 1e-6 costs nothing on easy poses.
    damping: float = 1e-6
    position_cost: float = 1.0
    orientation_cost: float = 1.0
    posture_cost: float = 1e-3
    # Error-proportional task damping; 1e-6 is effectively off, and without it
    # full QP steps limit-cycle around the tolerance near singular reaches.
    lm_damping: float = 1e-1
    gain: float = 0.5
    safety_break: bool = True
    # Default solve tolerances; callers may still override per call.
    position_tolerance: float = 0.001
    orientation_tolerance: float = 0.01
    # Retries only run after a failure; easy poses solve on attempt 0.
    max_attempts: int = 10
    # Std of the Gaussian perturbation applied per joint on a retry, scaled by
    # the attempt index, so restarts widen from a nudge to a rethink.
    retry_seed_noise: float = 0.02
    # Trailing attempts that redraw uniformly over the joint range, so a seed
    # sitting in a genuinely bad basin still gets a global restart.
    uniform_restart_attempts: int = 1


ManipulationKinematicsConfig = Annotated[
    JacobianKinematicsConfig | DrakeOptimizationKinematicsConfig | PinkKinematicsConfig,
    Field(discriminator="backend"),
]


def kinematics_config_from_name(name: str) -> ManipulationKinematicsConfig:
    """Create a default kinematics config from the legacy backend name."""
    if name == "jacobian":
        return JacobianKinematicsConfig()
    if name == "drake_optimization":
        return DrakeOptimizationKinematicsConfig()
    if name == "pink":
        return PinkKinematicsConfig()
    raise ValueError(
        f"Unknown kinematics solver: {name}. Available: ['jacobian', 'drake_optimization', 'pink']"
    )
