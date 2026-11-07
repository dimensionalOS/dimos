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

"""One Metal on a bench in MuJoCo, with its wrist camera, the objects' true poses
and a voxel map of the bench that the planner avoids and a viewer draws."""

from dimos.control.coordinator import TaskConfig
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.core.coordination.blueprints import autoconnect
from dimos.manipulation.grasping.heuristic_grasp import HeuristicGraspModule
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.manipulation_skills import ManipulationSkills
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.perception.experimental.object_scene_registration import ObjectSceneRegistrationModule
from dimos.robot.manipulators.common.blueprints import coordinator, manipulation_mapping
from dimos.robot.manipulators.makermods_metal.config import (
    METAL_HEURISTIC_AIM,
    METAL_SIM_HOME,
    METAL_SIM_MAP_GRACE,
    METAL_SIM_VOXEL_SIZE,
    METAL_TF_LINKS,
    make_metal_model_config,
    make_metal_sim_hardware,
)
from dimos.robot.manipulators.makermods_metal.mjcf import make_metal_sim_module_kwargs
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule

# Every link on tf, so a viewer can draw the arm at its pose.
_MODEL = make_metal_model_config(
    robot="arm", home_joints=METAL_SIM_HOME, tf_extra_links=METAL_TF_LINKS
)
_HARDWARE = make_metal_sim_hardware()

# Trajectories, the jaw (joint7 is open at its lower end), the planner and the
# bench map, on the simulated arm. The map's self filter is the UrdfManager
# that serves the URDF to viewers.
metal_sim = autoconnect(
    MujocoSimModule.blueprint(**make_metal_sim_module_kwargs()),
    coordinator(
        hardware=[_HARDWARE],
        tasks=[
            joint_trajectory_task(list(_MODEL.joint_names)),
            TaskConfig(
                name="metal_gripper",
                type="gripper",
                joint_names=["arm/joint7"],
                priority=20,
                params={"open_at": "lower"},
            ),
        ],
    ),
    ManipulationModule.blueprint(
        model=_MODEL,
        gripper_tasks={"arm": "metal_gripper"},
        voxel_map_resolution=METAL_SIM_VOXEL_SIZE,
    ),
    manipulation_mapping(
        self_filter_model=_MODEL.model,
        voxel_size=METAL_SIM_VOXEL_SIZE,
        grace_depth=METAL_SIM_MAP_GRACE,
    ),
)

# The same arm with object detection from the wrist camera and pick-and-place.
metal_perception_sim = autoconnect(
    metal_sim,
    ManipulationSkills.blueprint(),
    PickAndPlaceModule.blueprint(planning_frame="world", robot=_MODEL),
    HeuristicGraspModule.blueprint(**METAL_HEURISTIC_AIM),
    ObjectSceneRegistrationModule.blueprint(
        target_frame="world",
        detector_backend="moondream",
        segmentation_backend="edgetam",
        detect_on_request=True,
    ),
)
