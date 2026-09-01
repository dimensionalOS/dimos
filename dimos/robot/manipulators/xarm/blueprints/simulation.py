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
from dimos.manipulation.grasping.heuristic_grasp import HeuristicGraspModule
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.manipulation_skills import ManipulationSkills
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.perception.experimental.object_scene_registration import ObjectSceneRegistrationModule
from dimos.robot.manipulators.common.blueprints import coordinator, trajectory_task
from dimos.robot.manipulators.xarm.config import (
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
XARM_ROOM_SCENE_PATH = LfsPath("xarm_grasp_sim/scene.xml")
# The stock xArm home points the narrow wrist-camera frustum between the six
# widely spaced targets. This collision-free top-down pose raises the camera
# enough to put every mesh in one frame, without changing the configured base
# pose or introducing coordinate offsets.
XARM_ROOM_SCAN_JOINTS = [0.0, -0.04609, 0.0, 1.83940, 0.0, 1.87106, 0.0]
_xarm_room_sim_hw = make_xarm7_sim_hardware(XARM_ROOM_SCENE_PATH, home_joints=XARM_ROOM_SCAN_JOINTS)
XARM_ROOM_PROMPTS = [
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
        optical_frame="wrist_camera_color_optical_frame",
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

xarm_room_sim = autoconnect(
    ManipulationModule.blueprint(
        model=_xarm7_sim_model,
        planning_timeout=10.0,
        visualization={"backend": "none"},
    ),
    ManipulationSkills.blueprint(),
    PickAndPlaceModule.blueprint(planning_frame="world"),
    HeuristicGraspModule.blueprint(),
    MujocoSimModule.blueprint(
        **{
            **make_xarm7_sim_module_kwargs(XARM_ROOM_SCENE_PATH),
            "headless": True,
            # Publish the simulated camera pose directly in the planning frame.
            # A wrist-relative TF would require a second asynchronously stamped
            # world->link7 edge and can make an otherwise valid scan unregistrable.
            "base_frame_id": "world",
            "reset_joint_positions": XARM_ROOM_SCAN_JOINTS,
        }
    ),
    ObjectSceneRegistrationModule.blueprint(
        target_frame="world",
        # Synthetic MuJoCo renders score far below natural images.
        candidate_floor=0.07,
        accept_score=0.07,
        # This deterministic room scan intentionally uses one fixed overview.
        min_views=1,
        optical_frame="wrist_camera_color_optical_frame",
    ),
    coordinator(
        hardware=[_xarm_room_sim_hw],
        tasks=[
            trajectory_task(_xarm_room_sim_hw),
            TaskConfig(
                name="arm_gripper",
                type="gripper",
                joint_names=["arm/gripper"],
                priority=20,
            ),
        ],
    ),
)
