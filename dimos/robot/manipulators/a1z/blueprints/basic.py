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

"""Basic Galaxea A1Z coordinator and planner blueprints."""

from __future__ import annotations

from dimos.control.coordinator import ControlCoordinator
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.global_config import global_config
from dimos.robot.manipulators.a1z.config import (
    A1Z_G1Z_MODEL_PATH,
    A1Z_G1Z_SIM_MODEL_PATH,
    a1z_hardware,
    make_a1z_model_config,
    make_a1z_sim_robot_config,
)
from dimos.robot.manipulators.common.blueprints import coordinator, planner, trajectory_task
from dimos.simulation.providers import (
    SimulationBinding,
    SimulationFeature,
    SimulationRequest,
    resolve_robot,
)
from dimos.visualization.vis_module import vis_module


def _resolve_a1z_robot() -> SimulationBinding:
    return resolve_robot(
        real_hardware=(a1z_hardware("arm"),),
        simulation=SimulationRequest(
            robot_model="galaxea_a1z",
            model_path=A1Z_G1Z_SIM_MODEL_PATH,
            scene_package=global_config.scene_package,
            features=frozenset(
                (
                    SimulationFeature.EPISODE_CONTROL,
                    SimulationFeature.MANIPULATION_SCENE,
                )
            ),
        ),
    )


_simulation = _resolve_a1z_robot()
_a1z_simulation_modules: tuple[Blueprint, ...]
if not global_config.simulation_provider:
    _a1z_planner_hw = _simulation.hardware[0]
    _a1z_planner_model = make_a1z_model_config(name="arm")
    _a1z_simulation_modules = (_simulation.backend,)
else:
    if len(_simulation.hardware) != 1:
        raise ValueError("a1z-planner-coordinator requires one arm hardware component")
    _a1z_planner_hw = _simulation.hardware[0]
    _a1z_planner_model = make_a1z_sim_robot_config(_simulation.robot_base_pose)
    _a1z_simulation_modules = (
        _simulation.backend,
        vis_module(
            viewer_backend=global_config.viewer,
            rerun_config=_simulation.rerun_config,
        ),
    )

a1z_planner_coordinator = autoconnect(
    planner(robots=[_a1z_planner_model]),
    coordinator(
        hardware=[_a1z_planner_hw],
        tasks=[trajectory_task(_a1z_planner_hw)],
    ),
    *_a1z_simulation_modules,
)

_coordinator_a1z_hw = a1z_hardware(
    "arm",
    dynamics_urdf_path=A1Z_G1Z_MODEL_PATH,
)

coordinator_a1z = ControlCoordinator.blueprint(
    hardware=[_coordinator_a1z_hw],
    tasks=[trajectory_task(_coordinator_a1z_hw)],
)
