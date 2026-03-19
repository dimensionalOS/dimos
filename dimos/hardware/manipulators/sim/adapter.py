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
Arm joint methods are inherited from SimManipInterface.
Gripper is handled separately — engine joint at index ``dof`` (if present).
"""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING

from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.simulation.engines.mujoco_engine import MujocoEngine
from dimos.simulation.manipulators.sim_manip_interface import SimManipInterface

if TYPE_CHECKING:
    from dimos.hardware.manipulators.registry import AdapterRegistry


class SimMujocoAdapter(SimManipInterface):
    """ManipulatorAdapter backed by MuJoCo simulation.

    Uses ``address`` as the MJCF XML path (same field real adapters use for IP/port).
    If the engine has more joints than ``dof``, the extra joint at index ``dof``
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

        # Engine may report more joints than arm DOF (e.g. 8 = 7 arm + 1 gripper).
        self._gripper_idx: int | None = None
        if len(self._joint_names) > dof:
            self._gripper_idx = dof
        self._dof = dof

    def read_gripper_position(self) -> float | None:
        if self._gripper_idx is None:
            return None
        positions = self._engine.read_joint_positions()
        if self._gripper_idx < len(positions):
            return positions[self._gripper_idx]
        return None

    def write_gripper_position(self, position: float) -> bool:
        if self._gripper_idx is None:
            return False
        # Read current full state and update only the gripper index
        positions = list(self._engine.read_joint_positions())
        if self._gripper_idx < len(positions):
            positions[self._gripper_idx] = position
            self._engine.write_joint_command(JointState(position=positions))
            return True
        return False


def register(registry: AdapterRegistry) -> None:
    """Register this adapter with the registry."""
    registry.register("sim_mujoco", SimMujocoAdapter)


__all__ = ["SimMujocoAdapter"]
