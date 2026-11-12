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

"""Simulated single-arm OpenYAM manipulation in ABC Sim scenes.

    dimos --simulation run openyam-manipulation-sim

Pick the scene with ``--mujocosimmodule.address`` and, for a left-arm scene, the base with
``MANIPULATIONMODULE__MODEL__BASE_POSE``.
"""

from __future__ import annotations

from dimos.control.coordinator import TaskConfig
from dimos.core.coordination.blueprints import autoconnect
from dimos.manipulation.grasping.heuristic_grasp import HeuristicGraspModule
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.manipulation_skills import ManipulationSkills
from dimos.robot.manipulators.common.blueprints import coordinator, trajectory_task
from dimos.robot.manipulators.openyam.config import (
    OPENYAM_GRIPPER_JOINT,
    OPENYAM_HOME_JOINTS,
    OPENYAM_SIM_GRIPPER_TASK,
    OPENYAM_SIM_SCENE,
    make_openyam_sim_model_config,
    openyam_sim_hardware,
)
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule
from dimos.visualization.rerun.bridge import RerunBridgeModule

_openyam_sim_hw = openyam_sim_hardware()

openyam_manipulation_sim = autoconnect(
    ManipulationModule.blueprint(
        gripper_tasks={"manipulator": OPENYAM_SIM_GRIPPER_TASK},
        model=make_openyam_sim_model_config(),
        planning_timeout=10.0,
        visualization={"backend": "viser"},
    ),
    ManipulationSkills.blueprint(),
    # Grasps from an object's point cloud, e.g. localize's object_pointcloud.
    HeuristicGraspModule.blueprint(),
    MujocoSimModule.blueprint(
        address=OPENYAM_SIM_SCENE,
        headless=False,
        dof=len(OPENYAM_HOME_JOINTS),
        # The fixed overhead camera sees the whole table.
        camera_name="overhead",
        base_frame_id="world",
        reset_joint_positions=OPENYAM_HOME_JOINTS,
    ),
    coordinator(
        hardware=[_openyam_sim_hw],
        tasks=[
            trajectory_task(_openyam_sim_hw),
            TaskConfig(
                name=OPENYAM_SIM_GRIPPER_TASK,
                type="gripper",
                joint_names=[OPENYAM_GRIPPER_JOINT],
                priority=20,
            ),
        ],
    ),
    RerunBridgeModule.blueprint(),
)
