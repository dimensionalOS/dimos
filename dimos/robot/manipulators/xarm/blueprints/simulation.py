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
from dimos.manipulation.planning.global_map_obstacle_bridge import GlobalMapObstacleBridge
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.perception.experimental.object_scene_registration import ObjectSceneRegistrationModule
from dimos.perception.point_cloud_self_filter import PointCloudSelfFilter
from dimos.protocol.tf.point_cloud_tf_pose_source import PointCloudTfPoseSource
from dimos.robot.manipulators.common.blueprints import coordinator, trajectory_task
from dimos.robot.manipulators.xarm.config import (
    XARM7_SIM_PATH,
    make_xarm7_sim_hardware,
    make_xarm7_sim_module_kwargs,
    make_xarm7_sim_robot_config,
)
from dimos.robot.robot_tf_publisher import RobotTfPublisher
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule
from dimos.visualization.rerun.bridge import RerunBridgeModule

_xarm7_sim_hw = make_xarm7_sim_hardware(XARM7_SIM_PATH)
XARM_VOXEL_PLANNING_RESOLUTION = 0.05

xarm_perception_sim = autoconnect(
    PickAndPlaceModule.blueprint(
        robots=[make_xarm7_sim_robot_config()],
        planning_timeout=10.0,
        visualization={"backend": "viser"},
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
            visualization={"backend": "viser"},
            world_backend="roboplan",
            planner={"backend": "roboplan"},
            # Pink/Pinocchio and RoboPlan currently load incompatible native
            # dependencies in one process; the first interactive IK call can
            # abort the worker. Jacobian IK uses RoboPlan's synchronized world
            # interface and keeps gizmo evaluation in one native backend.
            kinematics={"backend": "jacobian"},
        ),
        MujocoSimModule.blueprint(
            **make_xarm7_sim_module_kwargs(XARM7_SIM_PATH),
            enable_depth=True,
            enable_color=True,
            enable_pointcloud=True,
            camera_info_fps=5.0,
        ),
        PointCloudSelfFilter.blueprint(
            robot_model=make_xarm7_sim_robot_config(),
            padding_m=XARM_VOXEL_PLANNING_RESOLUTION / 2.0,
            voxel_size=XARM_VOXEL_PLANNING_RESOLUTION,
            planning_frame="world",
            tf_tolerance_s=0.02,
            tf_forward_tolerance_s=0.05,
        ),
        PointCloudTfPoseSource.blueprint(
            fixed_frame="world",
            tf_tolerance_s=0.02,
            tf_forward_tolerance_s=0.05,
        ),
        RobotTfPublisher.blueprint(
            robot_model=make_xarm7_sim_robot_config(),
            fixed_frame="world",
        ),
        RayTracingVoxelMap.blueprint(
            voxel_size=XARM_VOXEL_PLANNING_RESOLUTION,
            map_frame="world",
            pose_match_tolerance_s=0.02,
        ),
        GlobalMapObstacleBridge.blueprint(
            resolution=XARM_VOXEL_PLANNING_RESOLUTION,
            planning_frame="world",
            world_backend="roboplan",
        ),
        coordinator(
            hardware=[_xarm7_sim_hw],
            tasks=[trajectory_task(_xarm7_sim_hw)],
        ),
    )
    .remappings(
        [
            (RayTracingVoxelMap, "lidar", "filtered_pointcloud"),
        ]
    )
    .global_config(simulation="mujoco")
)
