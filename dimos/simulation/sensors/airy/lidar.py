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

"""Simulated RoboSense Airy: the spec sheet pattern and range, with the Mid-360's grazing dropout."""

from __future__ import annotations

from dataclasses import replace

from dimos.simulation.sensors.airy.pattern import AiryPattern
from dimos.simulation.sensors.lidar import Raycaster, SimLidar
from dimos.simulation.sensors.mid360 import lidar as mid360

HOUSING_HALF_EXTENTS = (0.03, 0.03, 0.0315)

RETURNS = replace(
    mid360.RETURNS, blind_range=0.1, max_range=30.0, noise_floor_m=0.01, noise_per_m=0.0
)


def sim_airy(raycaster: Raycaster, seed: int) -> SimLidar:
    return SimLidar(raycaster, AiryPattern(), seed, RETURNS)
