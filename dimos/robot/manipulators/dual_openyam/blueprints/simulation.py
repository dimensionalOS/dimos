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

"""Both OpenYAM arms in an ABC Sim scene, planned and driven by dimOS.

    dimos --simulation run dual-openyam-abc-sim

Pick the scene with ``MUJOCOSIMMODULE__ADDRESS=data/abc_<task>/scene.xml``. In a scene
whose table is at 0.76 m, place the planning model to match with
``MANIPULATIONMODULE__MODEL__BASE_POSE`` (see ``dual_openyam_sim_base_pose``).
"""

from __future__ import annotations

from dimos.control.coordinator import TaskConfig
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.core.coordination.blueprints import autoconnect
from dimos.manipulation.grasping.heuristic_grasp import HeuristicGraspModule
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.manipulation_skills import ManipulationSkills
from dimos.robot.manipulators.common.blueprints import coordinator
from dimos.robot.manipulators.dual_openyam.abc_sim import (
    DUAL_OPENYAM_SIM_ARM_JOINTS,
    DUAL_OPENYAM_SIM_GROUPS,
    DUAL_OPENYAM_SIM_JOINTS,
    DUAL_OPENYAM_SIM_RESET,
    DUAL_OPENYAM_SIM_SCENE,
    DUAL_OPENYAM_SIM_SIDES,
    dual_openyam_sim_gripper_joint,
    dual_openyam_sim_gripper_task,
    dual_openyam_sim_hardware,
    make_dual_openyam_sim_model_config,
)
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule
from dimos.visualization.rerun.bridge import RerunBridgeModule

_hardware = dual_openyam_sim_hardware()

dual_openyam_abc_sim = autoconnect(
    ManipulationModule.blueprint(
        model=make_dual_openyam_sim_model_config(),
        gripper_tasks={
            DUAL_OPENYAM_SIM_GROUPS[side]: dual_openyam_sim_gripper_task(side)
            for side in DUAL_OPENYAM_SIM_SIDES
        },
        planning_timeout=10.0,
        visualization={"backend": "viser"},
    ),
    ManipulationSkills.blueprint(),
    # Grasps from an object's point cloud, e.g. ManipulationLocalize.get_pointcloud.
    HeuristicGraspModule.blueprint(),
    MujocoSimModule.blueprint(
        address=DUAL_OPENYAM_SIM_SCENE,
        headless=False,
        # Every actuator, both grippers included, is a plain position target.
        dof=len(DUAL_OPENYAM_SIM_JOINTS),
        # ABC's fixed camera above the table, the one its policies see.
        camera_name="top",
        base_frame_id="world",
        reset_joint_positions=DUAL_OPENYAM_SIM_RESET,
    ),
    coordinator(
        hardware=[_hardware],
        tasks=[
            joint_trajectory_task(list(DUAL_OPENYAM_SIM_ARM_JOINTS), start_position_tolerance=0.05),
            *(
                TaskConfig(
                    name=dual_openyam_sim_gripper_task(side),
                    type="gripper",
                    joint_names=[dual_openyam_sim_gripper_joint(side)],
                    priority=20,
                )
                for side in DUAL_OPENYAM_SIM_SIDES
            ),
        ],
    ),
    RerunBridgeModule.blueprint(),
)
