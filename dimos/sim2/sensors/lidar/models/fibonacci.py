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

"""PimSim's ideal equal-area scan pattern, with sensor-frame angles in degrees."""

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray


@dataclass(frozen=True)
class Fibonacci:
    ray_count: int = 15_000
    elevation_min: float = -7.2
    elevation_max: float = 52.2
    min_range: float = 0.1
    max_range: float = 30.0

    def directions(self) -> NDArray[np.float64]:
        indices = np.arange(self.ray_count, dtype=np.float64)
        z_min, z_max = np.sin(np.deg2rad([self.elevation_min, self.elevation_max]))
        z = z_min + (z_max - z_min) * (indices + 0.5) / self.ray_count
        azimuth = indices * (np.pi * (3.0 - np.sqrt(5.0)))
        radius = np.sqrt(1.0 - z * z)
        return np.column_stack((radius * np.cos(azimuth), radius * np.sin(azimuth), z))
