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

from dimos.core.coordination.blueprints import Blueprint
from dimos.core.global_config import global_config


def mujoco_if_sim(sim_path: str | Path, arm_dof: int) -> tuple[Blueprint, ...]:
    """The blueprint's MuJoCo sim, under ``--simulation`` only.

    ``arm_dof`` counts the arm joints alone. The sim takes a scene joint past
    them as the gripper and drives it through its own range.
    """
    if not global_config.simulation:
        return ()

    from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule

    return (MujocoSimModule.blueprint(address=str(sim_path), headless=False, dof=arm_dof),)
