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

"""The FR3 grasping stack: wrist RealSense, detection, grasps and pick and place.

``dimos run fr3-grasp --frankaconnection.ip 172.16.0.2``

The same stack as ``xarm-grasp`` on hardware: the arm through
``FrankaConnection``, a RealSense D435i on the Franka Hand (its hand-eye
calibrated mount in ``fr3_wrist_camera_transform``), heuristic top-down grasps
aimed with the Hand's geometry, and pick and place.
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.manipulation.grasping.heuristic_grasp import HeuristicGraspModule
from dimos.manipulation.manipulation_localize import ManipulationLocalize
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.manipulation_skills import ManipulationSkills
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.perception.localize.types import LocalizeParams
from dimos.robot.manipulators.common.blueprints import manipulation_mapping
from dimos.robot.manipulators.franka.blueprints.basic import FRANKA_REMAPS, franka_coordinator
from dimos.robot.manipulators.franka.config import (
    FR3_HEURISTIC_AIM,
    FR3_PREFIX,
    FR3_ROOT_LINK,
    fr3_model,
    fr3_wrist_camera_transform,
    make_fr3_model_config,
)
from dimos.visualization.rerun.bridge import RerunBridgeModule

# The mapper and planner octree use the same resolution.
FR3_GRASP_VOXEL_SIZE = 0.025

_model = make_fr3_model_config(
    robot="arm",
    base_pose=PoseStamped(frame_id="world"),
    # The wrist camera's transform hangs off fr3_hand, and the self filter places
    # the cloud in the model's root link, so both go out on tf.
    tf_extra_links=[FR3_ROOT_LINK, f"{FR3_PREFIX}hand"],
)

fr3_grasp = autoconnect(
    ManipulationModule.blueprint(
        gripper_tasks={"manipulator": "arm_gripper"},
        model=_model,
        static_transforms=[fr3_wrist_camera_transform()],
        planning_timeout=10.0,
        visualization={"backend": "viser"},
        world_frame="world",
        voxel_map_resolution=FR3_GRASP_VOXEL_SIZE,
    ),
    ManipulationSkills.blueprint(),
    PickAndPlaceModule.blueprint(planning_frame="world", robot=_model),
    # enable_pointcloud is off by default, and the voxel chain has nothing to map without it.
    RealSenseCamera.blueprint(enable_pointcloud=True),
    # Finds the objects pick_object takes by id, and the dense scene its contact check needs.
    # The wrist camera often looks from one pose (the arm still), so one view confirms an object.
    ManipulationLocalize.blueprint(world_frame="world", params=LocalizeParams(min_views=1)),
    manipulation_mapping(
        # Filter the arm and the Hand out of the camera's cloud.
        self_filter_model=fr3_model(robot="arm"),
        voxel_size=FR3_GRASP_VOXEL_SIZE,
    ),
    RerunBridgeModule.blueprint(),
    franka_coordinator(),
    HeuristicGraspModule.blueprint(**FR3_HEURISTIC_AIM),
).remappings(FRANKA_REMAPS)
