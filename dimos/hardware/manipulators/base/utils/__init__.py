# Copyright 2025 Dimensional Inc.
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

"""Shared utilities for manipulator drivers."""

from .converters import degrees_to_radians, meters_to_mm, mm_to_meters, radians_to_degrees
from .shared_state import SharedState
from .validators import (
    clamp_positions,
    scale_velocities,
    validate_acceleration_limits,
    validate_joint_limits,
    validate_trajectory,
    validate_velocity_limits,
)

__all__ = [
    "SharedState",
    "clamp_positions",
    "degrees_to_radians",
    "meters_to_mm",
    "mm_to_meters",
    "radians_to_degrees",
    "scale_velocities",
    "validate_acceleration_limits",
    "validate_joint_limits",
    "validate_trajectory",
    "validate_velocity_limits",
]
