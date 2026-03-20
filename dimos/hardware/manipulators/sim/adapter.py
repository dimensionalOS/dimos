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

from dimos.simulation.engines.mujoco_engine import MujocoEngine
from dimos.simulation.manipulators.sim_manip_interface import SimManipInterface

if TYPE_CHECKING:
    from dimos.hardware.manipulators.registry import AdapterRegistry


class SimMujocoAdapter(SimManipInterface):
    """ManipulatorAdapter backed by MuJoCo simulation.

    Uses ``address`` as the MJCF XML path (same field real adapters use for IP/port).
    If the engine has more joints than ``dof``, the extra joint at index ``dof``
    is treated as the gripper, with ctrl range scaled automatically.
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
        self._gripper_ctrl_range: tuple[float, float] = (0.0, 255.0)
        self._gripper_joint_range: tuple[float, float] = (0.0, 0.85)
        if len(self._joint_names) > dof:
            self._gripper_idx = dof
            # Read actuator ctrl range from MuJoCo model
            mapping = engine._joint_mappings[dof]
            if mapping.actuator_id is not None:
                lo = float(engine._model.actuator_ctrlrange[mapping.actuator_id, 0])
                hi = float(engine._model.actuator_ctrlrange[mapping.actuator_id, 1])
                self._gripper_ctrl_range = (lo, hi)
            # Read joint range for position feedback
            if mapping.tendon_qpos_adrs:
                first_joint_adr = mapping.tendon_qpos_adrs[0]
                for jid in range(engine._model.njnt):
                    if engine._model.jnt_qposadr[jid] == first_joint_adr:
                        lo = float(engine._model.jnt_range[jid, 0])
                        hi = float(engine._model.jnt_range[jid, 1])
                        self._gripper_joint_range = (lo, hi)
                        break
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
        # Scale from joint range (0-0.85) to actuator ctrl range (0-255), inverted.
        # MuJoCo driver joints close when ctrl increases, so invert.
        jlo, jhi = self._gripper_joint_range
        clo, chi = self._gripper_ctrl_range
        if jhi != jlo:
            t = (position - jlo) / (jhi - jlo)
            ctrl_value = chi - t * (chi - clo)
        else:
            ctrl_value = clo
        with self._engine._lock:
            self._engine._joint_position_targets[self._gripper_idx] = ctrl_value
        return True


def register(registry: AdapterRegistry) -> None:
    """Register this adapter with the registry."""
    registry.register("sim_mujoco", SimMujocoAdapter)


__all__ = ["SimMujocoAdapter"]
