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

"""Simulation xArm perception manipulation blueprints."""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.robot.manipulators.common.blueprints import coordinator, trajectory_task
from dimos.robot.manipulators.xarm.config import (
    XARM7_MODEL_PATH,
    XARM7_SIM_HOME,
    XARM7_TABLETOP_SCENE,
    make_xarm7_sim_robot_config,
    make_xarm_hardware,
)
from dimos.simulation.providers import (
    SimulationBinding,
    SimulationFeature,
    SimulationRequest,
    resolve_robot,
)
from dimos.visualization.vis_module import vis_module


def _resolve_xarm7_simulation() -> SimulationBinding:
    return resolve_robot(
        real_hardware=(
            make_xarm_hardware(
                "arm",
                7,
                gripper=True,
                gripper_open_position=0.85,
                gripper_closed_position=0.0,
                home_joints=XARM7_SIM_HOME,
            ),
        ),
        simulation=SimulationRequest(
            robot_model="xarm7",
            model_path=XARM7_MODEL_PATH,
            scene_package=global_config.scene_package or XARM7_TABLETOP_SCENE,
            features=frozenset(
                (
                    SimulationFeature.EPISODE_CONTROL,
                    SimulationFeature.MANIPULATION_SCENE,
                    SimulationFeature.SENSORS,
                )
            ),
        ),
    )


def _require_pimsim() -> str | None:
    if global_config.simulation != "mujoco":
        return "xarm-perception-sim requires --simulation mujoco"
    if global_config.simulation_provider != "pimsim":
        return "xarm-perception-sim requires --simulation-provider pimsim"
    return None


_simulation = _resolve_xarm7_simulation()
if len(_simulation.hardware) != 1:
    raise ValueError("xarm-perception-sim requires one arm hardware component")
_xarm7_sim_hw = _simulation.hardware[0]

xarm_perception_sim = autoconnect(
    PickAndPlaceModule.blueprint(
        robots=[make_xarm7_sim_robot_config(_simulation.robot_base_pose)],
        planning_timeout=10.0,
        pick_verification="observed_lift",
        visualization={"backend": "viser"},
    ),
    _simulation.backend,
    coordinator(
        hardware=[_xarm7_sim_hw],
        tasks=[trajectory_task(_xarm7_sim_hw)],
    ),
    vis_module(
        viewer_backend=global_config.viewer,
        rerun_config=_simulation.rerun_config,
    ),
).requirements(_require_pimsim)
