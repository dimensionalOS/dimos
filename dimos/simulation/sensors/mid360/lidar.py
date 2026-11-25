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

"""Simulated Mid-360: the fitted pattern with the Go2 return model."""

from __future__ import annotations

from dimos.simulation.sensors.lidar import OcclusionMap, Raycaster, ReturnModel, SimLidar
from dimos.simulation.sensors.mid360.pattern import Mid360Pattern
from dimos.utils.data import get_data

HOUSING_HALF_EXTENTS = (0.0325, 0.0325, 0.03)

RETURNS = ReturnModel(
    blind_range=0.16,
    max_range=40.0,
    noise_floor_m=0.0034,
    noise_per_m=0.00073,
    incidence_exponent=0.78,
    incidence_deg=(0.0, 78.0, 79.5, 82.5, 85.0, 88.0, 90.0),
    incidence_p_return=(1.0, 1.0, 0.76, 0.5, 0.22, 0.08, 0.0),
)


def sim_mid360(raycaster: Raycaster, seed: int, occlusion: OcclusionMap | None = None) -> SimLidar:
    """The Mid-360 with the fitted pattern, shadowed by its mount's occlusion if given."""
    pattern = Mid360Pattern.load(get_data("go2_sim") / "mid360_pattern.npz")
    return SimLidar(raycaster, pattern, seed, RETURNS, occlusion)
