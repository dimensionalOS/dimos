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

"""Acquisition contract: read scene observations without advancing physics."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from typing import TYPE_CHECKING, Protocol

import numpy as np
from numpy.typing import NDArray

from dimos.sim2.description import World
from dimos.sim2.state import Array, Layout

if TYPE_CHECKING:
    from dimos_lcm.sim_msgs import Observations

    from dimos.sim2.messages.observations import SceneSample


@dataclass(frozen=True)
class Scan:
    points: Array  # world-frame hits, preserving acquisition time per point
    times: Array
    epoch: int


class SensorReader(Protocol):
    """Read-only acquisition provider; a separate seam from dynamics."""

    gaps: int

    def observe(self, message: Observations) -> SceneSample | None: ...
    def camera(
        self, frame: str, state: SceneSample
    ) -> tuple[NDArray[np.uint8], NDArray[np.float32]]: ...
    def lidar(self, frame: str, duration: float = 0.1) -> Scan | None: ...
    def close(self) -> None: ...


SensorFactory = Callable[[World, Layout], SensorReader]
