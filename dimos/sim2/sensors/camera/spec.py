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

"""Camera geometry, output selection and acquisition settings."""

from __future__ import annotations

from pydantic import ConfigDict
from pydantic.dataclasses import dataclass

from dimos.sim2.sensors.spec import AcquisitionRequirements, Mount, validate_timing


@dataclass(frozen=True, config=ConfigDict(extra="forbid"))
class Camera:
    name: str
    camera: str | Mount
    width: int = 640
    height: int = 480
    # Named cameras retain their asset calibration. Only added cameras set fovy.
    fovy: float | None = None
    rate_hz: float = 10.0
    depth: bool = True
    pointcloud: bool = False
    shadows: bool = True
    latency: float = 0.0

    def __post_init__(self) -> None:
        validate_timing(self.rate_hz, self.latency)
        if self.pointcloud and not self.depth:
            raise ValueError("camera point clouds require depth")
        if isinstance(self.camera, str) and self.fovy is not None:
            raise ValueError("set named camera calibration in the MJCF asset, not RobotConfig")

    @property
    def model_name(self) -> str:
        return self.camera if isinstance(self.camera, str) else f"sensor/{self.name}"

    @property
    def acquisition(self) -> AcquisitionRequirements:
        return AcquisitionRequirements(self.rate_hz, 2 / self.rate_hz)
