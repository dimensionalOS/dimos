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

from __future__ import annotations

from pathlib import Path
from typing import Protocol

import numpy as np
from numpy.typing import NDArray

from dimos.simulation.scenes.procedural import Family, Params
from dimos.simulation.sim_world.body import BodyShape
from dimos.spec.utils import Spec


class SimWorldSpec(Spec, Protocol):
    def load_scene(self, family: Family, seed: int, params: Params) -> None: ...

    def shape(self) -> BodyShape: ...

    def reset_pose(self, x: float, y: float, z: float, yaw: float) -> None: ...

    def publish_premap(self, route: NDArray[np.float64], objects: bool) -> int: ...

    def export_model(self, path: Path) -> None: ...
