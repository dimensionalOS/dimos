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

"""RoboSense Airy scan pattern, with the beam table of two M20 units.

96 beams fan from the horizon to the zenith and spin clockwise about the sensor's z axis,
firing every beam at each azimuth step.
"""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

BEAMS = 96
AZIMUTH_STEPS = 900
FRAME_RATE = 10.0
POINT_RATE = BEAMS * AZIMUTH_STEPS * FRAME_RATE

_ELEVATION = np.radians(
    [
        -0.11,
        0.83,
        1.82,
        2.72,
        3.795,
        4.795,
        5.76,
        6.6,
        7.68,
        8.64,
        9.57,
        10.455,
        11.465,
        12.39,
        13.345,
        14.21,
        15.225,
        16.155,
        17.08,
        18.005,
        18.99,
        19.9,
        20.825,
        21.735,
        22.675,
        23.6,
        24.53,
        25.34,
        26.365,
        27.28,
        28.195,
        29.065,
        30.05,
        30.945,
        31.85,
        32.755,
        33.7,
        34.6,
        35.505,
        36.385,
        37.36,
        38.245,
        39.16,
        40.045,
        41.035,
        41.88,
        42.8,
        43.705,
        44.61,
        45.5,
        46.4,
        47.29,
        48.285,
        49.135,
        50.05,
        50.96,
        51.98,
        52.785,
        53.685,
        54.6,
        55.615,
        56.455,
        57.39,
        58.285,
        59.3,
        60.13,
        61.045,
        61.965,
        63.045,
        63.81,
        64.735,
        65.65,
        66.615,
        67.48,
        68.415,
        69.345,
        70.375,
        71.22,
        72.15,
        73.135,
        74.225,
        75.025,
        75.96,
        76.91,
        78.005,
        78.83,
        79.765,
        80.74,
        81.775,
        82.65,
        83.59,
        84.6,
        85.725,
        86.525,
        87.57,
        88.475,
    ]
)
_AZIMUTH_OFFSET = np.radians(
    [
        -1.63,
        -1.67,
        -1.69,
        -1.72,
        -1.74,
        -1.77,
        -1.79,
        -1.82,
        -1.84,
        -1.87,
        -1.89,
        -1.91,
        1.3,
        1.3,
        1.31,
        1.32,
        1.33,
        1.34,
        1.34,
        1.36,
        1.37,
        1.38,
        1.39,
        1.41,
        -1.81,
        -1.83,
        -1.86,
        -1.87,
        -1.9,
        -1.92,
        -1.94,
        -1.96,
        -1.99,
        -2.01,
        -2.03,
        -2.05,
        1.59,
        1.6,
        1.62,
        1.63,
        1.64,
        1.66,
        1.68,
        1.7,
        1.72,
        1.74,
        1.77,
        1.8,
        -2.47,
        -2.49,
        -2.52,
        -2.55,
        -2.57,
        -2.6,
        -2.63,
        -2.65,
        -2.68,
        -2.81,
        -2.93,
        -2.97,
        2.14,
        2.17,
        2.22,
        2.26,
        2.3,
        2.34,
        2.39,
        2.45,
        2.51,
        2.58,
        2.64,
        2.72,
        -5.05,
        -5.61,
        -5.79,
        -5.99,
        -6.2,
        -6.45,
        -6.71,
        -7.01,
        -7.35,
        -7.75,
        -8.19,
        -8.7,
        4.73,
        5.09,
        5.52,
        6.05,
        6.71,
        7.58,
        8.71,
        10.38,
        12.93,
        15.49,
        23.52,
        35.04,
    ]
)


class AiryPattern:
    point_rate = POINT_RATE

    def directions(self, k0: int, n: int) -> NDArray[np.float64]:
        """Unit sensor-frame directions of global point indices k0 .. k0 + n."""
        k = np.arange(k0, k0 + n)
        azimuth = 2.0 * np.pi * (k // BEAMS) / AZIMUTH_STEPS + _AZIMUTH_OFFSET[k % BEAMS]
        elevation = _ELEVATION[k % BEAMS]
        out: NDArray[np.float64] = np.column_stack(
            [
                np.cos(elevation) * np.cos(azimuth),
                -np.cos(elevation) * np.sin(azimuth),
                np.sin(elevation),
            ]
        )
        return out
