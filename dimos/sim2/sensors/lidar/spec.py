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

"""Lidar acquisition settings and ray-pattern/return-model contracts."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import field
from typing import Any, Literal, Protocol, runtime_checkable

import numpy as np
from numpy.typing import NDArray
from pydantic import ConfigDict
from pydantic.dataclasses import dataclass

from dimos.sim2.sensors.imu.spec import Imu
from dimos.sim2.sensors.spec import AcquisitionRequirements, Mount, validate_timing


@runtime_checkable
class RayPattern(Protocol):
    @property
    def min_range(self) -> float: ...
    @property
    def max_range(self) -> float: ...

    def directions(self) -> NDArray[np.float64]: ...


@dataclass(frozen=True, config=ConfigDict(arbitrary_types_allowed=True))
class TimedRays:
    directions: NDArray[np.float64]
    offsets: NDArray[np.float64]
    lines: NDArray[np.uint8]


@runtime_checkable
class TimedRayPattern(Protocol):
    @property
    def min_range(self) -> float: ...
    @property
    def max_range(self) -> float: ...
    @property
    def motion_sample_rate_hz(self) -> float: ...

    def scan(self, start: float, duration: float) -> TimedRays: ...


@runtime_checkable
class RangeResponse(Protocol):
    def measure(
        self, ranges: NDArray[np.float64], cos_incidence: NDArray[np.float64], start: float
    ) -> NDArray[np.float64]:
        """Apply return noise/dropout, preserving ray indices and negative misses."""
        ...


@dataclass(frozen=True, config=ConfigDict(extra="forbid"))
class Lidar:
    name: str
    site: str | Mount
    model: Callable[..., RayPattern | TimedRayPattern]
    model_kwargs: dict[str, Any] = field(default_factory=dict)
    rate_hz: float = 10.0
    # Optional world-frame cutoff in degrees for ideal mapping scans.
    maximum_world_elevation: float | None = None
    # Sensor-frame clouds retain the ray origin for ray-tracing mappers.
    output_frame: Literal["world", "sensor"] = "world"
    self_occlusion: bool = False
    # Explicit optical housing exclusions in this robot's mesh namespace.
    excluded_meshes: tuple[str, ...] = ()
    # A device IMU is independent of the robot's coherent control IMU.
    imu: Imu | None = None
    # Disable both corrected cloud and truth TF when testing an estimator.
    truth_outputs: bool = True
    latency: float = 0.0

    def __post_init__(self) -> None:
        validate_timing(self.rate_hz, self.latency)
        if self.imu is not None and not isinstance(
            self.model(**self.model_kwargs), TimedRayPattern
        ):
            raise ValueError("a lidar IMU requires timed acquisition")

    @property
    def model_name(self) -> str:
        return self.site if isinstance(self.site, str) else f"sensor/{self.name}"

    @property
    def acquisition(self) -> AcquisitionRequirements:
        model = self.model(**self.model_kwargs)
        rate = self.rate_hz
        if isinstance(model, TimedRayPattern):
            if model.motion_sample_rate_hz < self.rate_hz:
                raise ValueError("motion sampling cannot be slower than scan acquisition")
            rate = max(rate, model.motion_sample_rate_hz)
        if self.imu is not None:
            rate = max(rate, self.imu.rate_hz)
        return AcquisitionRequirements(rate, 2 / self.rate_hz)
