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

from functools import cache

from dimos.core.coordination.blueprints import Blueprint
from dimos.core.global_config import global_config
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.simulation.providers import (
    SimulationBinding,
    SimulationRequest,
    load_simulation_provider,
)


def resolve_go2_platform() -> Blueprint:
    if global_config.simulation in ("", "dimsim"):
        return GO2Connection.blueprint()
    return _resolve_simulation_binding().backend


def resolve_go2_rerun_config() -> dict[str, object]:
    if global_config.simulation in ("", "dimsim"):
        return {}
    return _resolve_simulation_binding().rerun_config


@cache
def _resolve_simulation_binding() -> SimulationBinding:
    if global_config.simulation != "mujoco":
        raise ValueError("unitree-go2 only supports --simulation mujoco")
    if not global_config.simulation_provider:
        raise ValueError("unitree-go2 simulation requires --simulation-provider pimsim")

    provider = load_simulation_provider(global_config.simulation_provider)
    return provider.build(
        SimulationRequest(
            robot_model="unitree_go2",
            scene_package=global_config.scene_package,
        )
    )


__all__ = ["resolve_go2_platform", "resolve_go2_rerun_config"]
