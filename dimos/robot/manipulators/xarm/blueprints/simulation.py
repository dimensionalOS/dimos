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
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.global_config import global_config
from dimos.manipulation.grasping.heuristic_grasp import HeuristicGraspModule
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.manipulation_skills import ManipulationSkills
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.perception.experimental.object_scene_registration import ObjectSceneRegistrationModule
from dimos.robot.manipulators.common.blueprints import (
    coordinator,
    eef_twist_task,
    trajectory_task,
)
from dimos.robot.manipulators.common.coordinators import ArmTwistCoordinator
from dimos.robot.manipulators.xarm.config import (
    XARM7_SIM_PATH,
    make_xarm7_sim_hardware,
    make_xarm7_sim_module_kwargs,
    make_xarm7_sim_robot_config,
)
from dimos.robot.raw_robot_bridge import RawRobotBridge
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule
from dimos.visualization.rerun.bridge import RerunBridgeModule

_xarm7_sim_scene = global_config.mujoco_scene or XARM7_SIM_PATH
_xarm7_sim_model = make_xarm7_sim_robot_config()
_xarm7_sim_hw = make_xarm7_sim_hardware(_xarm7_sim_scene)
_xarm7_sim_kwargs = make_xarm7_sim_module_kwargs(_xarm7_sim_scene)

xarm_perception_sim = autoconnect(
    ManipulationModule.blueprint(
        model=_xarm7_sim_model,
        planning_timeout=10.0,
        visualization={"backend": "viser"},
    ),
    ManipulationSkills.blueprint(),
    PickAndPlaceModule.blueprint(planning_frame="world"),
    HeuristicGraspModule.blueprint(),
    MujocoSimModule.blueprint(**_xarm7_sim_kwargs),
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


def _xarm_sim(base_pose: PoseStamped | None = None) -> Blueprint:
    """Robot-only stack: control and sensors, exposed as plain Zenoh topics for agents without
    dimOS. ``base_pose`` is where the scene mounts ``link_base``; the eef_twist IK and the
    measured TCP pose on tf use it."""
    return autoconnect(
        MujocoSimModule.blueprint(
            **{
                **_xarm7_sim_kwargs,
                "base_frame_id": "world",
            }
        ),
        coordinator(
            hardware=[_xarm7_sim_hw],
            tasks=[
                eef_twist_task(
                    _xarm7_sim_hw,
                    robot_model=make_xarm7_sim_robot_config(base_pose=base_pose),
                    target_frame="link_tcp",
                    max_joint_velocity_rad_s=0.5,
                ),
                TaskConfig(
                    name="arm_gripper",
                    type="gripper",
                    joint_names=["arm/gripper"],
                    priority=20,
                ),
            ],
            cls=ArmTwistCoordinator,
            instance_name="ControlCoordinator",
            publish_frame_poses=True,
        ),
        RawRobotBridge.blueprint(
            camera_frame="wrist_camera_color_optical_frame",
            ee_frame="link_tcp",
            gripper_joint="arm/gripper",
            gripper_range=(0.0, 0.85),
        ),
    )


xarm_sim = autoconnect(_xarm_sim())
# The robosuite exports keep their full-height tables and mount link_base 0.912 m up.
xarm_sim_robosuite = autoconnect(
    _xarm_sim(PoseStamped(frame_id="world", position=Vector3(z=0.912)))
)
