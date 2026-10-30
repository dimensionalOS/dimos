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

"""Continuous four-channel Mid360 firing and approximate range response.

Uses the shared Fourier implementation on main, with the G1 capture calibration
from experiments/mid360/README.md at commit 8ef292d4f. That calibration has
different rotor frequencies from the Go2 fit; it is not silently substituted.
"""

from dataclasses import dataclass
from functools import lru_cache
import math
from typing import Literal

import numpy as np
from numpy.typing import NDArray

from dimos.sim2.sensors.lidar.models.mid360.pattern import POINT_RATE, Mid360Pattern
from dimos.sim2.sensors.lidar.models.mid360.returns import OcclusionMap, ReturnModel
from dimos.sim2.sensors.lidar.spec import TimedRays
from dimos.utils.data import get_data


@lru_cache(maxsize=2)
def _pattern(calibration: str = "g1") -> Mid360Pattern:
    if calibration == "go2":
        return Mid360Pattern.load(get_data("go2_sim/mid360_pattern.npz"))
    with np.load(get_data("mid360_pattern/fourier.npz"), allow_pickle=False) as archive:
        return Mid360Pattern(
            float(archive["f1"]),
            float(archive["f2"]),
            int(archive["m1"]),
            int(archive["m2"]),
            np.asarray(archive["coefs"], dtype=np.float64),
        )


@lru_cache(maxsize=1)
def _go2_occlusion() -> OcclusionMap:
    return OcclusionMap.load(get_data("go2_sim/go2_mid360_occlusion.npz"))


@dataclass(frozen=True)
class Mid360:
    """One rolling device model; return coefficients from Andrew's PR #4441.

    Noise/dropout are approximations, not a calibrated reflectivity model.
    Disable them for geometry-only diagnostics without changing acquisition.
    """

    point_rate_hz: int = 200_000
    motion_sample_rate_hz: float = 200.0
    downsample: int = 1
    min_range: float = 0.16
    max_range: float = 40.0
    noise: bool = True
    dropout: bool = True
    seed: int = 0
    calibration: Literal["g1", "go2"] = "g1"
    mount_occlusion: bool = False

    def __post_init__(self) -> None:
        if self.calibration not in ("g1", "go2"):
            raise ValueError("Mid360 calibration must be g1 or go2")
        if self.mount_occlusion and self.calibration != "go2":
            raise ValueError("measured mount occlusion is only available for Go2")
        if self.point_rate_hz != POINT_RATE:
            raise ValueError("Mid360 fires at 200000 points/s; use downsample to reduce ray count")
        if not isinstance(self.downsample, int) or self.downsample < 1 or self.seed < 0:
            raise ValueError("Mid360 downsample must be a positive integer and seed nonnegative")
        if not all(
            math.isfinite(v) for v in (self.motion_sample_rate_hz, self.min_range, self.max_range)
        ):
            raise ValueError("Mid360 rates and range limits must be finite")
        if self.motion_sample_rate_hz <= 0 or not 0 <= self.min_range < self.max_range:
            raise ValueError("Mid360 requires a positive motion rate and ordered range limits")

    def scan(self, start: float, duration: float) -> TimedRays:
        if not math.isfinite(start) or start < 0:
            raise ValueError("Mid360 scan start must be finite and nonnegative")
        if not math.isfinite(duration):
            raise ValueError("Mid360 scan duration must contain complete four-laser groups")
        count = round(duration * self.point_rate_hz)
        if duration <= 0 or not math.isclose(count, duration * self.point_rate_hz) or count % 4:
            raise ValueError("Mid360 scan duration must contain complete four-laser groups")
        offsets = (np.arange(0, count, 4 * self.downsample)[:, None] + np.arange(4)).ravel()
        indices = round(start * self.point_rate_hz) + offsets
        return TimedRays(
            directions=_pattern(self.calibration).directions(
                round(start * self.point_rate_hz), count
            )[offsets],
            offsets=offsets.astype(np.float64) / self.point_rate_hz,
            lines=(indices % 4).astype(np.uint8),
        )

    def measure(
        self, ranges: NDArray[np.float64], cos_incidence: NDArray[np.float64], start: float
    ) -> NDArray[np.float64]:
        # Key each scan independently: skipping a late scan cannot shift later noise.
        rng = np.random.default_rng([self.seed, round(start * self.point_rate_hz)])
        cosine = np.clip(np.abs(cos_incidence), 0.0, 1.0)
        keep = (ranges >= self.min_range) & (ranges <= self.max_range)
        if self.mount_occlusion:
            # Same measured mask as the existing Go2 sensor, without another ray engine.
            count = len(ranges)
            offsets = (
                np.arange(0, math.ceil(count / 4) * 4 * self.downsample, 4 * self.downsample)[
                    :, None
                ]
                + np.arange(4)
            ).ravel()[:count]
            directions = _pattern(self.calibration).directions(
                round(start * self.point_rate_hz), int(offsets[-1]) + 1
            )[offsets]
            keep &= rng.random(count) >= _go2_occlusion().lookup(directions)
        response = ReturnModel()
        if self.dropout:
            probability = np.interp(
                np.rad2deg(np.arccos(cosine)),
                response.incidence_deg,
                response.incidence_p_return,
            )
            keep &= rng.random(len(ranges)) < probability
        result = ranges.copy()
        if self.noise:
            sigma = np.hypot(response.noise_floor_m, response.noise_per_m * ranges) / (
                np.maximum(cosine, 0.05) ** response.incidence_exponent
            )
            result += rng.standard_normal(len(ranges)) * sigma
        keep &= (result >= self.min_range) & (result <= self.max_range)
        result[~keep] = -1
        return result
