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

"""Basic Piper coordinator blueprints."""

from __future__ import annotations

from dimos.control.coordinator import ControlCoordinator
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.global_config import global_config
from dimos.robot.manipulators.common.sim import mujoco_if_sim
from dimos.robot.manipulators.piper.config import (
    PIPER_ROBOT_MODEL_PATH,
    PIPER_SIM_PATH,
    piper_hardware,
)
from dimos.simulation.providers import (
    SimulationBinding,
    SimulationFeature,
    SimulationRequest,
    resolve_robot,
)
from dimos.visualization.vis_module import vis_module


def _resolve_piper_robot() -> SimulationBinding:
    real_hardware = piper_hardware("arm")
    default_backend = autoconnect(*mujoco_if_sim(PIPER_SIM_PATH, len(real_hardware.joints)))
    return resolve_robot(
        real_hardware=(real_hardware,),
        default_backend=default_backend,
        simulation=SimulationRequest(
            robot_model="agilex_piper",
            model_path=PIPER_ROBOT_MODEL_PATH,
            scene_package=global_config.scene_package,
            features=frozenset((SimulationFeature.EPISODE_CONTROL,)),
        ),
    )


_piper_simulation = _resolve_piper_robot()
if len(_piper_simulation.hardware) != 1:
    raise ValueError("coordinator-piper requires one arm hardware component")
_piper_hw = _piper_simulation.hardware[0]
if not global_config.simulation_provider:
    _piper_simulation_modules: tuple[Blueprint, ...] = (_piper_simulation.backend,)
else:
    _piper_simulation_modules = (
        _piper_simulation.backend,
        vis_module(
            viewer_backend=global_config.viewer,
            rerun_config=_piper_simulation.rerun_config,
        ),
    )

coordinator_piper = autoconnect(
    ControlCoordinator.blueprint(
        hardware=[_piper_hw],
        tasks=[joint_trajectory_task(_piper_hw.joints)],
    ),
    *_piper_simulation_modules,
)
