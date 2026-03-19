# Copyright 2025-2026 Dimensional Inc.
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

"""MuJoCo simulation adapter for ControlCoordinator integration.

Thin wrapper around SimManipInterface that plugs into the adapter registry.
All 23 ManipulatorAdapter methods are inherited from SimManipInterface.
"""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING

from dimos.simulation.engines.mujoco_engine import MujocoEngine
from dimos.simulation.manipulators.sim_manip_interface import SimManipInterface

if TYPE_CHECKING:
    from dimos.hardware.manipulators.registry import AdapterRegistry


class SimMujocoAdapter(SimManipInterface):
    """ManipulatorAdapter backed by MuJoCo simulation.

    Uses ``address`` as the MJCF XML path (same field real adapters use for IP/port).
    """

    def __init__(
        self,
        dof: int = 7,
        address: str | None = None,
        headless: bool = True,
        **_: object,
    ) -> None:
        if address is None:
            raise ValueError("address (MJCF XML path) is required for sim_mujoco adapter")
        engine = MujocoEngine(config_path=Path(address), headless=headless)
        super().__init__(engine=engine)


def register(registry: AdapterRegistry) -> None:
    """Register this adapter with the registry."""
    registry.register("sim_mujoco", SimMujocoAdapter)


__all__ = ["SimMujocoAdapter"]
