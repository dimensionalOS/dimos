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

from __future__ import annotations

from typing import TypedDict


class ImuNoise(TypedDict):
    """Splatted into an ImuConfig, which also has non-float fields, so a plain dict would
    widen to dict[str, float] and fail the call's type check."""

    gyro_noise_density: float
    gyro_random_walk: float
    accel_noise_density: float
    accel_random_walk: float


IMU_BMI055 = ImuNoise(
    gyro_noise_density=0.0018,
    gyro_random_walk=2e-5,
    accel_noise_density=0.02,
    accel_random_walk=3e-3,
)
