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

"""Simulation helpers for manipulator blueprints."""

from __future__ import annotations

from pathlib import Path
from typing import cast

from dimos.core.coordination.blueprints import Blueprint
from dimos.core.global_config import global_config
from dimos.core.module import Module
from dimos.deps.requires import Requires
from dimos.hardware.adapter_registry import LazyAdapterRegistry

REQUIRES = Requires(selectors={"g.simulation": "simulation"})


class SimModuleRegistry(LazyAdapterRegistry[Module]):
    """Simulation module blueprints by ``--simulation`` value, from ``engines/_registry.py``."""

    kind = "simulation module"
    manifest_table = "SIM_MODULE_FACTORIES"
    manifest_roots = (("dimos.simulation.engines", 0),)


sim_modules = SimModuleRegistry()


def mujoco_if_sim(sim_path: str | Path, dof: int) -> tuple[Blueprint, ...]:
    """The simulation module for the selected simulator, or nothing on real hardware."""
    if not global_config.simulation:
        return ()
    module = cast("type[Module]", sim_modules.resolve(global_config.simulation))
    return (module.blueprint(address=str(sim_path), headless=False, dof=dof),)
