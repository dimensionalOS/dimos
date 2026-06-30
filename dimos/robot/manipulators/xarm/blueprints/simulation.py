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

import math

from dimos_gpd_grasp_demo.blueprint import gpd_grasp_gen_blueprint
from dimos_gpd_grasp_demo.gpd_grasp_gen_module import GPDGraspGenModule

from dimos.core.coordination.blueprints import autoconnect
from dimos.manipulation.grasping.grasping import GraspingModule
from dimos.manipulation.grasping.pointcloud_grasp_demo_controller import (
    PointcloudGraspDemoController,
)
from dimos.manipulation.grasping.target_grasp_demo_controller import TargetGraspDemoController
from dimos.manipulation.grasping.vgn_grasp_gen_module import VGNGraspGenModule
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.perception.object_scene_registration import ObjectSceneRegistrationModule
from dimos.perception.reconstruction import SceneReconstructionModule
from dimos.robot.manipulators.common.blueprints import coordinator, trajectory_task
from dimos.robot.manipulators.xarm.config import (
    XARM7_SIM_PATH,
    make_xarm7_model_config,
    make_xarm_hardware,
)
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule
from dimos.visualization.rerun.bridge import RerunBridgeModule

XARM7_SIM_HOME = [0.0, 0.0, 0.0, 0.0, 0.0, -0.7, 0.0]
XARM7_VGN_OBSERVATION_HOME = [0.0, -0.247, 0.0, 0.909, 0.0, 1.15644, 0.0]

_xarm7_sim_hw = make_xarm_hardware(
    "arm",
    7,
    adapter_type="sim_mujoco",
    address=str(XARM7_SIM_PATH),
    gripper=True,
    home_joints=XARM7_SIM_HOME,
)

xarm_perception_sim = autoconnect(
    PickAndPlaceModule.blueprint(
        robots=[
            make_xarm7_model_config(
                name="arm",
                add_gripper=True,
                pitch=math.radians(45),
                tf_extra_links=["link7"],
                home_joints=XARM7_SIM_HOME,
                pre_grasp_offset=0.05,
            )
        ],
        planning_timeout=10.0,
        visualization={"backend": "meshcat"},
    ),
    MujocoSimModule.blueprint(
        address=str(XARM7_SIM_PATH),
        headless=False,
        dof=7,
        camera_name="wrist_camera",
        base_frame_id="link7",
    ),
    ObjectSceneRegistrationModule.blueprint(target_frame="world"),
    coordinator(
        hardware=[_xarm7_sim_hw],
        tasks=[trajectory_task(_xarm7_sim_hw)],
    ),
    RerunBridgeModule.blueprint(),
)

vgn_mujoco_grasp_demo = autoconnect(
    MujocoSimModule.blueprint(
        address=str(XARM7_SIM_PATH),
        headless=False,
        dof=7,
        camera_name="wrist_camera",
        base_frame_id="link7",
        enable_depth=True,
        enable_color=True,
        enable_pointcloud=False,
        camera_info_fps=5.0,
        initial_joint_positions=XARM7_VGN_OBSERVATION_HOME,
    ),
    SceneReconstructionModule.blueprint(
        target_frame="world",
        workspace_center=(0.45, 0.0, 0.18),
        workspace_size=0.3,
        resolution=40,
        reconstruction_fps=2.0,
        depth_trunc=2.0,
    ),
    ObjectSceneRegistrationModule.blueprint(
        target_frame="world",
        min_detections_for_permanent=1,
        use_aabb=True,
    ),
    VGNGraspGenModule.blueprint(
        output_frame="world",
        quality_threshold=0.05,
        width_filter_min_voxels=0.0,
        width_filter_max_voxels=1000.0,
        filter_candidates_to_target_bounds=True,
        auto_generate_on_tsdf=False,
        debug_export_dir="/tmp/opencode/dimos-vgn-tsdf-debug",
    ),
    GraspingModule.blueprint(),
    TargetGraspDemoController.blueprint(target_name="sphere", cushion_m=0.2),
    RerunBridgeModule.blueprint(),
).remappings(
    [
        (SceneReconstructionModule, "tsdf", "tsdf_surface"),
        (VGNGraspGenModule, "tsdf", "tsdf_surface"),
    ]
)

gpd_mujoco_grasp_demo = autoconnect(
    MujocoSimModule.blueprint(
        address=str(XARM7_SIM_PATH),
        headless=False,
        dof=7,
        camera_name="wrist_camera",
        base_frame_id="link7",
        enable_depth=True,
        enable_color=True,
        enable_pointcloud=True,
        camera_info_fps=5.0,
        initial_joint_positions=XARM7_VGN_OBSERVATION_HOME,
    ),
    ObjectSceneRegistrationModule.blueprint(
        target_frame="world",
        min_detections_for_permanent=1,
        use_aabb=True,
    ),
    gpd_grasp_gen_blueprint(),
    GraspingModule.blueprint(),
    PointcloudGraspDemoController.blueprint(target_name="sphere", filter_collisions=False),
    RerunBridgeModule.blueprint(),
).remappings(
    [
        (GPDGraspGenModule, "grasp_candidates", "gpd_grasp_candidates"),
    ]
)
