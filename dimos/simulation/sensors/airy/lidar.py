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

"""Simulated RoboSense Airy: the fitted pattern and returns of two M20 units."""

from __future__ import annotations

from dimos.simulation.sensors.airy.pattern import AiryPattern
from dimos.simulation.sensors.lidar import Raycaster, ReturnModel, SimLidar

HOUSING_HALF_EXTENTS = (0.03, 0.03, 0.0315)
LENS_HEIGHT = 0.04532

RETURNS = ReturnModel(
    blind_range=0.1,
    max_range=30.0,
    noise_floor_m=0.01,
    noise_per_m=0.0005,
    incidence_exponent=0.5,
    incidence_deg=(0.0, 78.0, 80.0, 81.0, 84.0, 87.0, 88.0),
    incidence_p_return=(1.0, 1.0, 0.4, 0.25, 0.2, 0.15, 0.0),
)


def sim_airy(raycaster: Raycaster, seed: int) -> SimLidar:
    return SimLidar(raycaster, AiryPattern(), seed, RETURNS, lens_height=LENS_HEIGHT)
