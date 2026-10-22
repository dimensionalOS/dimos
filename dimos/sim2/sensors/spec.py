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

"""Mounts and acquisition timing shared by all sensor families."""

from __future__ import annotations

import math

from pydantic import ConfigDict
from pydantic.dataclasses import dataclass


@dataclass(frozen=True, config=ConfigDict(extra="forbid"))
class Mount:
    link: str
    xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)


@dataclass(frozen=True, config=ConfigDict(extra="forbid"))
class AcquisitionRequirements:
    """World samples needed by a device, independent of its output message rate."""

    sample_hz: float
    history_seconds: float

    def __post_init__(self) -> None:
        if not math.isfinite(self.sample_hz) or self.sample_hz <= 0:
            raise ValueError("acquisition sample rate must be finite and positive")
        if not math.isfinite(self.history_seconds) or self.history_seconds < 0:
            raise ValueError("acquisition history must be finite and nonnegative")


def validate_timing(rate_hz: float, latency: float) -> None:
    if not math.isfinite(rate_hz) or rate_hz <= 0:
        raise ValueError("sensor rate must be finite and positive")
    if not math.isfinite(latency) or latency < 0:
        raise ValueError("sensor latency must be finite and nonnegative")
