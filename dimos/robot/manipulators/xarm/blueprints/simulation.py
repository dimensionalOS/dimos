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

from dimos.control.coordinator import TaskConfig
from dimos.core.coordination.blueprints import autoconnect
from dimos.manipulation.grasping.grasp_gen_x import GraspGenXModule
from dimos.manipulation.grasping.heuristic_grasp import HeuristicGraspModule
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.manipulation_skills import ManipulationSkills
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.manipulation.planning.utils.point_cloud_self_filter import PointCloudSelfFilter
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.perception.experimental.object_scene_registration import ObjectSceneRegistrationModule
from dimos.robot.manipulators.common.blueprints import coordinator, trajectory_task
from dimos.robot.manipulators.xarm.config import (
    XARM7_COLLISION_LINKS,
    XARM7_SIM_PATH,
    make_xarm7_sim_hardware,
    make_xarm7_sim_module_kwargs,
    make_xarm7_sim_robot_config,
)
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule
from dimos.utils.data import LfsPath
from dimos.visualization.rerun.bridge import RerunBridgeModule

_xarm7_sim_model = make_xarm7_sim_robot_config()
_xarm7_sim_hw = make_xarm7_sim_hardware(XARM7_SIM_PATH)
XARM_GRASP_SCENE_PATH = LfsPath("xarm_grasp_sim/scene.xml")
# The stock xArm home points the narrow wrist-camera frustum between the six
# widely spaced targets. This collision-free top-down pose raises the camera
# enough to put every mesh in one frame, without changing the configured base
# pose or introducing coordinate offsets.
XARM_GRASP_SCAN_JOINTS = [0.0, -0.04609, 0.0, 1.83940, 0.0, 1.87106, 0.0]
_xarm_grasp_sim_hw = make_xarm7_sim_hardware(
    XARM_GRASP_SCENE_PATH, home_joints=XARM_GRASP_SCAN_JOINTS
)
# One resolution for the whole mapping chain. The self filter's clear mask, the
# mapper's cells and the planner's octree must all agree: a mismatched mask names
# cells the map does not hold and clears nothing, and a mismatched octree does not
# line up with what was mapped.
XARM_GRASP_VOXEL_SIZE = 0.025
# data/xarm_grasp_sim/xarm7.xml bolts link_base to the world origin instead of the
# 12 cm pedestal data/xarm7 uses, so this scene must not inherit the pedestal offset.
# With it, the planning model sits 12 cm above the arm MuJoCo simulates and every
# pose executes 12 cm low -- the exact failure XARM7_SIM_BASE_POSE warns about.
_xarm_grasp_sim_model = make_xarm7_sim_robot_config(
    base_pose=PoseStamped(frame_id="world"),
    # The self filter needs a capture-time transform for every collision link
    # and drops the whole cloud when one is missing.
    tf_extra_links=XARM7_COLLISION_LINKS,
)
XARM_GRASP_PROMPTS = [
    "black bottle",
    "gray can",
    "red cup",
    "green tape roll",
    "blue marker",
    "brown box",
    # The wrist camera sees the tape almost directly from above, where it reads
    # as a ring instead of a roll. Keep a shape-word fallback for that view.
    "green ring",
]

xarm_perception_sim = autoconnect(
    ManipulationModule.blueprint(
        model=_xarm7_sim_model,
        planning_timeout=10.0,
        visualization={"backend": "viser"},
    ),
    ManipulationSkills.blueprint(),
    PickAndPlaceModule.blueprint(planning_frame="world"),
    HeuristicGraspModule.blueprint(),
    MujocoSimModule.blueprint(**make_xarm7_sim_module_kwargs(XARM7_SIM_PATH)),
    ObjectSceneRegistrationModule.blueprint(
        target_frame="world",
        detector_backend="moondream",
        segmentation_backend="edgetam",
        detect_on_request=True,
    ),
    coordinator(
        hardware=[_xarm7_sim_hw],
        tasks=[
            trajectory_task(_xarm7_sim_hw),
            TaskConfig(
                name="arm_gripper",
                type="gripper",
                joint_names=["arm/gripper"],
                priority=20,
            ),
        ],
    ),
    RerunBridgeModule.blueprint(),
)

# Measured off data/xarm_grasp_sim (mj_forward on the driver joint limits), expressed
# in GraspGenX's convention -- approach along +Z, jaws closing along X -- with the
# origin on xarm_gripper_base_link, which is the frame GraspGenX predicts into.
XARM_GRIPPER_SWEEP_VOLUME = {
    "extents_open": (0.0889, 0.030, 0.0370),
    "offset_open": (0.0, 0.0, 0.1421),
    "extents_half_open": (0.0479, 0.030, 0.0370),
    "offset_half_open": (0.0, 0.0, 0.1530),
    "fingertip_depth": 0.1606,
}
# xarm_gripper_base_link -> link_tcp, the planning tip frame: +0.172 m along the
# approach axis (xarm_gripper.urdf.xacro joint_tcp) plus the quarter turn that takes
# GraspGenX's X closing axis onto the xArm gripper's Y.
XARM_GRASP_FRAME_TO_TCP = (
    (0.0, 1.0, 0.0, 0.0),
    (-1.0, 0.0, 0.0, 0.0),
    (0.0, 0.0, 1.0, 0.172),
    (0.0, 0.0, 0.0, 1.0),
)


# Everything the room scene needs except the GraspGenSpec provider, which the two
# blueprints below choose between. Exactly one provider may be composed in: the
# PickAndPlaceModule resolves its generator by spec, so two would be ambiguous.
_XARM_GRASP_SIM_MODULES = (
    ManipulationModule.blueprint(
        model=_xarm_grasp_sim_model,
        planning_timeout=10.0,
        visualization={"backend": "viser"},
        world_frame="world",
        voxel_map_resolution=XARM_GRASP_VOXEL_SIZE,
    ),
    ManipulationSkills.blueprint(),
    PickAndPlaceModule.blueprint(
        planning_frame="world",
        # These jaws asymptote to 0.995-0.996 and never reach 1.0, so opening a
        # gripper that is already open moves nothing and has to settle on arrival
        # alone. The stock 0.005 tolerance sits right on that gap and passes or
        # fails by luck; 0.01 clears it while still tracking real jaw travel.
        grasp_verification={"settle_tolerance": 0.01},
    ),
    MujocoSimModule.blueprint(
        **{
            **make_xarm7_sim_module_kwargs(XARM_GRASP_SCENE_PATH),
            "headless": True,
            # Publish the simulated camera pose directly in the planning frame.
            # A wrist-relative TF would require a second asynchronously stamped
            # world->link7 edge and can make an otherwise valid scan unregistrable.
            "base_frame_id": "world",
            "reset_joint_positions": XARM_GRASP_SCAN_JOINTS,
            # Off by default, and the voxel mapping chain has nothing to map
            # without it. Scene registration reads colour and depth directly.
            "enable_pointcloud": True,
        }
    ),
    # The wrist camera sees the arm itself, so the arm's returns must be dropped
    # before mapping and the volume it occupies erased from the map: ray tracing
    # cannot clear what the arm permanently occludes.
    PointCloudSelfFilter.blueprint(
        model=_xarm_grasp_sim_model.model,
        voxel_size=XARM_GRASP_VOXEL_SIZE,
        world_frame="world",
        # ManipulationModule publishes robot TF at 10Hz, so the stock 20ms
        # tolerance cannot bracket a ~92ms publish period: 61% of clouds found no
        # transform and were dropped, each with a warning. One full period admits
        # them all. The arm holds still while scanning, so a transform up to a
        # period old describes the same pose.
        tf_tolerance_s=0.1,
        tf_forward_tolerance_s=0.1,
    ),
    # Tabletop reach, not a room-scale lidar sweep: a short max_range keeps the
    # octree to the workspace the arm can actually plan into.
    RayTracingVoxelMap.blueprint(
        voxel_size=XARM_GRASP_VOXEL_SIZE,
        world_frame="world",
        max_range=2.0,
    ),
    ObjectSceneRegistrationModule.blueprint(
        target_frame="world",
        detector_backend="owlv2",
        # OWLv2 is box-only; YOLO-E visual prompts refine its boxes into masks.
        segmentation_backend="yolo",
        # Synthetic MuJoCo renders score far below natural images.
        detector_confidence=0.07,
        segmentation_confidence=0.05,
        # Keep adjacent tabletop targets distinct instead of merging by label.
        distance_threshold=0.05,
        detect_on_request=True,
        # The obstacle stream contains permanent objects only; one explicit
        # room scan must therefore promote its first sightings immediately.
        min_detections_for_permanent=1,
    ),
    coordinator(
        hardware=[_xarm_grasp_sim_hw],
        tasks=[
            trajectory_task(_xarm_grasp_sim_hw),
            TaskConfig(
                name="arm_gripper",
                type="gripper",
                joint_names=["arm/gripper"],
                priority=20,
            ),
        ],
    ),
)


# The mapping chain is wired by stream name, so the two edges whose names differ
# are bridged here: self filter -> mapper, and mapper -> the planner's octree.
_XARM_GRASP_SIM_REMAPPINGS = [
    (RayTracingVoxelMap, "lidar", "filtered_pointcloud"),
    (ManipulationModule, "voxel_map", "global_map"),
]

xarm_grasp_sim = autoconnect(*_XARM_GRASP_SIM_MODULES, HeuristicGraspModule.blueprint()).remappings(
    _XARM_GRASP_SIM_REMAPPINGS
)

xarm_grasp_sim_graspgenx = autoconnect(
    *_XARM_GRASP_SIM_MODULES,
    GraspGenXModule.blueprint(
        gripper=XARM_GRIPPER_SWEEP_VOLUME,
        grasp_frame_to_tcp=XARM_GRASP_FRAME_TO_TCP,
    ),
).remappings(_XARM_GRASP_SIM_REMAPPINGS)
