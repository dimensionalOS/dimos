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

"""RoboSense Airy scan pattern from its spec sheet, not yet fitted to a recording.

96 beams fan evenly from the horizon to the zenith and spin about the sensor's z axis,
firing every beam at each azimuth step.
"""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

BEAMS = 96
AZIMUTH_STEPS = 900
FRAME_RATE = 10.0
POINT_RATE = BEAMS * AZIMUTH_STEPS * FRAME_RATE

_ELEVATION = np.radians((np.arange(BEAMS) + 0.5) * 90.0 / BEAMS)


class AiryPattern:
    point_rate = POINT_RATE

    def directions(self, k0: int, n: int) -> NDArray[np.float64]:
        """Unit sensor-frame directions of global point indices k0 .. k0 + n."""
        k = np.arange(k0, k0 + n)
        azimuth = 2.0 * np.pi * (k // BEAMS) / AZIMUTH_STEPS
        elevation = _ELEVATION[k % BEAMS]
        out: NDArray[np.float64] = np.column_stack(
            [
                np.cos(elevation) * np.cos(azimuth),
                np.cos(elevation) * np.sin(azimuth),
                np.sin(elevation),
            ]
        )
        return out
