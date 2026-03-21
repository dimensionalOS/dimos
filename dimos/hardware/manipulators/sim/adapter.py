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
"""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING, Any

from dimos.simulation.engines.mujoco_engine import MujocoEngine
from dimos.simulation.manipulators.sim_manip_interface import SimManipInterface

if TYPE_CHECKING:
    from dimos.hardware.manipulators.registry import AdapterRegistry


class SimMujocoAdapter(SimManipInterface):
    """Uses ``address`` as the MJCF XML path (same field real adapters use for IP/port).
    If the engine has more joints than ``dof``, the extra joint at index ``dof``
    is treated as the gripper, with ctrl range scaled automatically.
    """

    def __init__(
        self,
        dof: int = 7,
        address: str | None = None,
        headless: bool = True,
        **_: Any,
    ) -> None:
        if address is None:
            raise ValueError("address (MJCF XML path) is required for sim_mujoco adapter")
        engine = MujocoEngine(config_path=Path(address), headless=headless)

        # Detect gripper from engine joints
        gripper_idx = None
        gripper_kwargs = {}
        joint_names = list(engine.joint_names)
        if len(joint_names) > dof:
            gripper_idx = dof
            ctrl_range = engine.get_actuator_ctrl_range(dof)
            joint_range = engine.get_joint_range(dof)
            if ctrl_range is None or joint_range is None:
                raise ValueError(f"Gripper joint at index {dof} missing ctrl/joint range in MJCF")
            gripper_kwargs = {"gripper_ctrl_range": ctrl_range, "gripper_joint_range": joint_range}

        super().__init__(engine=engine, dof=dof, gripper_idx=gripper_idx, **gripper_kwargs)


def register(registry: AdapterRegistry) -> None:
    """Register this adapter with the registry."""
    registry.register("sim_mujoco", SimMujocoAdapter)


__all__ = ["SimMujocoAdapter"]
