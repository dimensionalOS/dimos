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

"""IMU mount and acquisition settings."""

from __future__ import annotations

from pydantic import ConfigDict
from pydantic.dataclasses import dataclass

from dimos.sim2.sensors.spec import AcquisitionRequirements, Mount, validate_timing


@dataclass(frozen=True, config=ConfigDict(extra="forbid"))
class Imu:
    name: str
    site: str | Mount
    rate_hz: float = 200.0
    latency: float = 0.0

    def __post_init__(self) -> None:
        validate_timing(self.rate_hz, self.latency)

    @property
    def model_name(self) -> str:
        return self.site if isinstance(self.site, str) else f"sensor/{self.name}"

    @property
    def acquisition(self) -> AcquisitionRequirements:
        return AcquisitionRequirements(self.rate_hz, 2 / self.rate_hz)
