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

"""Scan timing and return behavior belong to lidar, independently of a ray engine."""

from collections.abc import Callable
from dataclasses import dataclass
from typing import Protocol, runtime_checkable

from dimos_lcm.sim_msgs import WorldState
import numpy as np
from numpy.typing import NDArray

from dimos.sim2.description import SceneConfig


@runtime_checkable
class ScanPattern(Protocol):
    def rays(
        self, start_ns: int, duration_ns: int
    ) -> tuple[NDArray[np.float64], NDArray[np.int64], NDArray[np.uint8]]:
        """Unit sensor-axis directions, offsets in nanoseconds, and laser channel IDs."""
        ...


@runtime_checkable
class RangeResponse(Protocol):
    def measure(
        self,
        ranges: NDArray[np.float64],
        cos_incidence: NDArray[np.float64],
        rng: np.random.Generator,
    ) -> NDArray[np.float64]:
        """Measured ranges in metres; negative values denote missing returns."""
        ...


@dataclass(frozen=True)
class UniformScan:
    rays_per_scan: int = 720
    elevations: tuple[float, ...] = (-0.65, -0.3, 0.0, 0.15)

    def rays(
        self, start_ns: int, duration_ns: int
    ) -> tuple[NDArray[np.float64], NDArray[np.int64], NDArray[np.uint8]]:
        az = np.linspace(-np.pi, np.pi, self.rays_per_scan, endpoint=False)
        el = np.resize(self.elevations, len(az))
        directions = np.column_stack((np.cos(az) * np.cos(el), np.sin(az) * np.cos(el), np.sin(el)))
        return (
            directions,
            np.arange(len(az), dtype=np.int64) * duration_ns // len(az),
            (np.arange(len(az)) % len(self.elevations)).astype(np.uint8),
        )


@dataclass(frozen=True)
class LidarConfig:
    frame_id: str
    pattern: Callable[[], ScanPattern] = UniformScan
    scan_hz: float = 10
    motion_hz: float = 200
    min_range: float = 0.05
    max_range: float = 40
    max_pose_gap_ns: int = 10_000_000
    range_std: float = 0
    seed: int = 0
    response: Callable[[], RangeResponse] | None = None
    # Optical exclusions refer to links in the shared model, never whole robots.
    excluded_bodies: tuple[str, ...] = ()
    publish_truth: bool = True


class Raycaster(Protocol):
    def load(self, scene: SceneConfig, excluded_bodies: tuple[str, ...] = ()) -> None: ...
    def set_poses(self, sample: WorldState) -> None: ...
    def cast(
        self,
        origins: NDArray[np.float64],
        directions: NDArray[np.float64],
        exclude_body: str | None = None,
    ) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        """Distance and world normal for each ray; negative distance on a miss."""
        ...

    def close(self) -> None: ...
