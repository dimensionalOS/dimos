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
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.perception.object_scene_registration import ObjectSceneRegistrationModule
from dimos.perception.point_cloud_self_filter import PointCloudSelfFilter, SelfFilterRegion
from dimos.protocol.tf.tf_pose_source import TfPoseSource
from dimos.robot.manipulators.common.blueprints import coordinator, trajectory_task
from dimos.robot.manipulators.xarm.config import (
    XARM7_SIM_PATH,
    make_xarm7_sim_hardware,
    make_xarm7_sim_module_kwargs,
    make_xarm7_sim_robot_config,
)
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule
from dimos.visualization.rerun.bridge import RerunBridgeModule

_xarm7_sim_hw = make_xarm7_sim_hardware(XARM7_SIM_PATH)
XARM_VOXEL_PLANNING_RESOLUTION = 0.05

_XARM_SELF_FILTER_REGIONS = [
    SelfFilterRegion(
        shape="box",
        frame_id="link7",
        size=(0.22, 0.22, 0.30),
        center=(0.0, 0.0, 0.02),
    ),
    SelfFilterRegion(shape="sphere", frame_id="link_tcp", radius=0.16),
]

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


xarm_voxel_planning_viser_demo = (
    autoconnect(
        ManipulationModule.blueprint(
            robots=[make_xarm7_sim_robot_config()],
            planning_timeout=10.0,
            planning_world_frame="world",
            planning_voxel_resolution=XARM_VOXEL_PLANNING_RESOLUTION,
            visualization={"backend": "viser"},
            world_backend="roboplan",
            planner_name="roboplan",
            kinematics={"backend": "pink"},
        ),
        MujocoSimModule.blueprint(
            # The simulator already has the camera's world pose. Publishing
            # that edge directly keeps mapping independent from the
            # manipulation module's derived world -> link7 TF loop.
            **(make_xarm7_sim_module_kwargs(XARM7_SIM_PATH) | {"base_frame_id": ""}),
            enable_depth=True,
            enable_color=True,
            enable_pointcloud=True,
            camera_info_fps=5.0,
        ),
        PointCloudSelfFilter.blueprint(
            regions=_XARM_SELF_FILTER_REGIONS,
            tf_tolerance_s=0.25,
            drop_cloud_on_missing_tf=True,
        ),
        TfPoseSource.blueprint(
            target_frame="world",
            source_frame="wrist_camera_color_optical_frame",
            tf_tolerance_s=0.25,
            publish_rate_hz=10.0,
        ),
        RayTracingVoxelMap.blueprint(
            voxel_size=XARM_VOXEL_PLANNING_RESOLUTION,
            pose_match_tolerance_s=0.1,
        ),
        coordinator(
            hardware=[_xarm7_sim_hw],
            tasks=[trajectory_task(_xarm7_sim_hw)],
        ),
    )
    .remappings(
        [
            (RayTracingVoxelMap, "lidar", "filtered_pointcloud"),
            (ManipulationModule, "planning_voxel_map", "global_map"),
        ]
    )
    .global_config(simulation="mujoco")
)
