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

"""Simulation engine registry."""

from __future__ import annotations

from typing import Literal

from dimos.simulation.engines.base import SimulationEngine
from dimos.simulation.engines.mujoco_engine import MujocoEngine

EngineType = Literal["mujoco"]

_ENGINES: dict[EngineType, type[SimulationEngine]] = {
    "mujoco": MujocoEngine,
}


def get_engine(engine_name: EngineType) -> type[SimulationEngine]:
    return _ENGINES[engine_name]
