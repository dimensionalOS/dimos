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

from types import ModuleType

from dimos_gpd_grasp_demo.blueprint import gpd_grasp_gen_blueprint
from dimos_gpd_grasp_demo.gpd_grasp_gen_module import GPDGraspGenModule

from dimos.core.coordination.blueprints import autoconnect
from dimos.manipulation.agentic_manipulation_module import AgenticGraspManipulationModule
from dimos.manipulation.grasping.grasping import GraspingModule
from dimos.manipulation.grasping.pointcloud_grasp_demo_controller import (
    PointcloudGraspDemoController,
)
from dimos.manipulation.grasping.target_grasp_demo_controller import TargetGraspDemoController
from dimos.manipulation.grasping.vgn_grasp_gen_module import VGNGraspGenModule
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.utils.mesh_utils import prepare_urdf_for_drake
from dimos.perception.detection.detectors.yoloe import YoloePromptMode
from dimos.perception.experimental.object_scene_registration import ObjectSceneRegistrationModule
from dimos.perception.reconstruction.scene_reconstruction import SceneReconstructionModule
from dimos.robot.manipulators.common.blueprints import coordinator, trajectory_task
from dimos.robot.manipulators.xarm.config import (
    XARM7_SIM_PATH,
    make_xarm7_model_config,
    make_xarm7_sim_hardware,
    make_xarm7_sim_module_kwargs,
    make_xarm7_sim_robot_config,
)
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule
from dimos.visualization.rerun.bridge import RerunBridgeModule
from dimos.visualization.rerun.urdf import unique_link_names, urdf_visuals_to_rerun

XARM7_SIM_HOME = [0.0, 0.0, 0.0, 0.0, 0.0, -0.7, 0.0]
XARM7_VGN_OBSERVATION_HOME = [0.0, -0.247, 0.0, 0.909, 0.0, 1.15644, 0.0]
XARM7_RERUN_LINK_FRAMES = unique_link_names(
    [
        "link_base",
        "link1",
        "link2",
        "link3",
        "link4",
        "link5",
        "link6",
        "link7",
        "link_eef",
        "xarm_gripper_base_link",
        "left_outer_knuckle",
        "left_finger",
        "left_inner_knuckle",
        "right_outer_knuckle",
        "right_finger",
        "right_inner_knuckle",
        "link_tcp",
    ]
)

XARM7_RERUN_HIGHLIGHT_LINKS = ["link7", "xarm_gripper_base_link", "link_tcp"]


def _manual_agentic_xarm7_model_config() -> RobotModelConfig:
    return make_xarm7_model_config(
        name="arm",
        add_gripper=True,
        tf_extra_links=XARM7_RERUN_LINK_FRAMES,
        home_joints=XARM7_VGN_OBSERVATION_HOME,
        pre_grasp_offset=0.05,
    )


def _manual_agentic_xarm7_rerun_static(rr: ModuleType) -> list[tuple[str, object]]:
    robot_config = _manual_agentic_xarm7_model_config()
    urdf_path = prepare_urdf_for_drake(
        robot_config.model_path,
        robot_config.package_paths,
        robot_config.xacro_args,
        robot_config.auto_convert_meshes,
        robot_config.base_link if robot_config.strip_model_world_joint else None,
    )
    return urdf_visuals_to_rerun(
        rr,
        urdf_path,
        entity_prefix="world/robot",
        highlight_links=XARM7_RERUN_HIGHLIGHT_LINKS,
    )


_xarm7_sim_hw = make_xarm7_sim_hardware(XARM7_SIM_PATH)

xarm_perception_sim = autoconnect(
    PickAndPlaceModule.blueprint(
        robots=[make_xarm7_sim_robot_config()],
        planning_timeout=10.0,
        visualization={"backend": "meshcat"},
    ),
    MujocoSimModule.blueprint(**make_xarm7_sim_module_kwargs(XARM7_SIM_PATH)),
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


manual_agentic_gpd_mujoco_grasp_demo = autoconnect(
    PickAndPlaceModule.blueprint(
        robots=[_manual_agentic_xarm7_model_config()],
        planning_timeout=10.0,
        visualization={"backend": "meshcat"},
    ),
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
        prompt_mode=YoloePromptMode.PROMPT,
        min_detections_for_permanent=1,
        use_aabb=True,
    ),
    gpd_grasp_gen_blueprint(),
    AgenticGraspManipulationModule.blueprint(),
    coordinator(
        hardware=[_xarm7_sim_hw],
        tasks=[trajectory_task(_xarm7_sim_hw)],
    ),
    RerunBridgeModule.blueprint(
        static={"manual_agentic_xarm7_urdf": _manual_agentic_xarm7_rerun_static}
    ),
).remappings(
    [
        (GPDGraspGenModule, "grasp_candidates", "gpd_grasp_candidates"),
        (AgenticGraspManipulationModule, "_grasp_gen", GPDGraspGenModule),
    ]
)
