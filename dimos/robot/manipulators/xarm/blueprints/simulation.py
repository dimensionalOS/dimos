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
from dimos.manipulation.grasping.heuristic_grasp import HeuristicGraspModule
from dimos.manipulation.manipulation_localize import ManipulationLocalize
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.manipulation_skills import ManipulationSkills
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.robot.manipulators.xarm.blueprints.basic import xarm_coordinator, xarm_gripper_task
from dimos.robot.manipulators.xarm.blueprints.teleop import XArmJogCoordinator, xarm_jog_task
from dimos.robot.manipulators.xarm.config import (
    XARM7_SIM_PATH,
    XARM_HEURISTIC_AIM,
    make_xarm7_sim_module_kwargs,
    make_xarm7_sim_robot_config,
)
from dimos.robot.raw_robot_bridge import RawRobotBridge
from dimos.visualization.rerun.bridge import RerunBridgeModule

_xarm7_sim_model = make_xarm7_sim_robot_config(robot="arm")
_xarm7_sim_kwargs = make_xarm7_sim_module_kwargs(XARM7_SIM_PATH)

xarm_perception_sim = autoconnect(
    ManipulationModule.blueprint(
        gripper_tasks={"manipulator": "arm_gripper"},
        model=_xarm7_sim_model,
        planning_timeout=10.0,
        visualization={"backend": "viser"},
    ),
    ManipulationSkills.blueprint(),
    PickAndPlaceModule.blueprint(planning_frame="world", robot=_xarm7_sim_model),
    HeuristicGraspModule.blueprint(**XARM_HEURISTIC_AIM),
    ManipulationLocalize.blueprint(world_frame="world"),
    xarm_coordinator("xarm7", simulated=True, sim_module_kwargs=_xarm7_sim_kwargs),
    RerunBridgeModule.blueprint(),
)

# Robot-only stack: control and sensors, exposed as plain Zenoh topics for agents without dimOS.
xarm_sim = autoconnect(
    xarm_coordinator(
        "xarm7",
        cls=XArmJogCoordinator,
        tasks=[
            xarm_jog_task(_xarm7_sim_model, "link_tcp", max_joint_velocity_rad_s=0.5),
            xarm_gripper_task(),
        ],
        simulated=True,
        sim_module_kwargs={**_xarm7_sim_kwargs, "base_frame_id": "world"},
        publish_frame_poses=True,
    ),
    RawRobotBridge.blueprint(
        camera_frame="wrist_camera_color_optical_frame",
        ee_frame="link_tcp",
        gripper_joint="arm/drive_joint",
        # drive_joint is closed at 0.85 rad and open at 0.
        gripper_range=(0.85, 0.0),
    ),
)
