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

"""Simulated Mid-360 returns: pattern directions cast into a scene with the Go2 return model."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Protocol

import numpy as np
from numpy.typing import NDArray

from dimos.simulation.sensors.mid360.pattern import Mid360Pattern
from dimos.utils.data import get_data


class Raycaster(Protocol):
    def cast(
        self, origin: NDArray[np.float64], directions: NDArray[np.float64], max_range: float
    ) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        """Distance and hit normal per world-frame unit direction. Distance is negative on a miss."""
        ...


@dataclass(frozen=True)
class ReturnModel:
    """Range noise and dropout of a Go2-mounted Mid-360."""

    blind_range: float = 0.16
    max_range: float = 40.0
    noise_floor_m: float = 0.0034
    noise_per_m: float = 0.00073
    incidence_exponent: float = 0.78
    incidence_deg: tuple[float, ...] = (0.0, 78.0, 79.5, 82.5, 85.0, 88.0, 90.0)
    incidence_p_return: tuple[float, ...] = (1.0, 1.0, 0.76, 0.5, 0.22, 0.08, 0.0)


@dataclass(frozen=True)
class OcclusionMap:
    """Per sensor-frame direction probability that the mount blocks the ray, on 1 degree cells."""

    probability: NDArray[np.float32]
    az0: float
    el0: float

    @classmethod
    def load(cls, path: str | Path) -> OcclusionMap:
        archive = np.load(path)
        return cls(
            archive["probability"], float(archive["az_edges"][0]), float(archive["el_edges"][0])
        )

    def lookup(self, directions: NDArray[np.float64]) -> NDArray[np.float32]:
        az = np.degrees(np.arctan2(directions[:, 1], directions[:, 0]))
        el = np.degrees(np.arcsin(np.clip(directions[:, 2], -1.0, 1.0)))
        rows, cols = self.probability.shape
        i = np.clip((el - self.el0).astype(int), 0, rows - 1)
        j = np.clip((az - self.az0).astype(int), 0, cols - 1)
        p: NDArray[np.float32] = self.probability[i, j]
        return p


@dataclass
class SimMid360:
    raycaster: Raycaster
    pattern: Mid360Pattern
    seed: int
    occlusion: OcclusionMap | None = None
    returns: ReturnModel = field(default_factory=ReturnModel)

    def __post_init__(self) -> None:
        self._rng = np.random.default_rng(self.seed)
        self._k = int(self._rng.integers(0, 2**31))
        self._p_knots = np.asarray(self.returns.incidence_deg)
        self._p_return = np.asarray(self.returns.incidence_p_return)

    @classmethod
    def go2(cls, raycaster: Raycaster, seed: int) -> SimMid360:
        """The Mid-360 with the fitted pattern and the Go2 mount occlusion."""
        root = get_data("go2_sim")
        return cls(
            raycaster=raycaster,
            pattern=Mid360Pattern.load(root / "mid360_pattern.npz"),
            seed=seed,
            occlusion=OcclusionMap.load(root / "go2_mid360_occlusion.npz"),
        )

    def cast(
        self,
        origin: NDArray[np.float64],
        world_from_sensor: NDArray[np.float64],
        n: int,
    ) -> NDArray[np.float32]:
        """The next n points of the pattern cast from one sensor pose, as sensor-frame returns."""
        rm = self.returns
        dirs = self.pattern.directions(self._k, n)
        self._k += n
        world_dirs = dirs @ world_from_sensor.T
        dist, normals = self.raycaster.cast(origin, world_dirs, rm.max_range)
        keep = dist > rm.blind_range
        if self.occlusion is not None:
            keep &= self._rng.random(n) >= self.occlusion.lookup(dirs)
        cos_inc = np.abs(np.einsum("ij,ij->i", world_dirs, normals))
        incidence = np.degrees(np.arccos(np.clip(cos_inc, 0.0, 1.0)))
        keep &= self._rng.random(n) < np.interp(incidence, self._p_knots, self._p_return)
        sigma = (
            np.hypot(rm.noise_floor_m, rm.noise_per_m * dist)
            / np.maximum(cos_inc, 0.05) ** rm.incidence_exponent
        )
        ranges = dist[keep] + self._rng.standard_normal(int(keep.sum())) * sigma[keep]
        points: NDArray[np.float32] = (dirs[keep] * ranges[:, None]).astype(np.float32)
        return points
