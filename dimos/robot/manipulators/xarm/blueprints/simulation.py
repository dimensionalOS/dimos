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

from typing import Any

from dimos.control.components import HardwareComponent
from dimos.control.coordinator import TaskConfig
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.manipulation.grasping.heuristic_grasp import HeuristicGraspModule
from dimos.manipulation.manipulation_localize import ManipulationLocalize
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.manipulation_skills import ManipulationSkills
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.perception.localize.types import LocalizeParams
from dimos.robot.manipulators.common.blueprints import (
    coordinator,
    eef_twist_task,
    trajectory_task,
)
from dimos.robot.manipulators.common.coordinators import ArmTwistCoordinator
from dimos.robot.manipulators.xarm.blueprints.teleop import xarm6_teleop_control
from dimos.robot.manipulators.xarm.config import (
    XARM6_SIM_PATH,
    XARM7_SIM_PATH,
    XARM_HEURISTIC_AIM,
    make_xarm7_sim_hardware,
    make_xarm7_sim_module_kwargs,
    make_xarm7_sim_robot_config,
    make_xarm_sim_hardware,
    make_xarm_sim_module_kwargs,
    make_xarm_sim_robot_config,
    xarm_link_frames,
)
from dimos.robot.raw_robot_bridge import RawRobotBridge
from dimos.robot.urdf_manager import UrdfManager
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule
from dimos.teleop.webxr.extensions import ArmTeleopModule
from dimos.visualization.rerun.bridge import RerunBridgeModule

_xarm7_sim_model = make_xarm7_sim_robot_config(tf_extra_links=xarm_link_frames(7))
_xarm7_sim_hw = make_xarm7_sim_hardware()
_xarm7_sim_kwargs = make_xarm7_sim_module_kwargs(XARM7_SIM_PATH)
_xarm6_sim_model = make_xarm_sim_robot_config(6, tf_extra_links=xarm_link_frames(6))
_xarm6_sim_hw = make_xarm_sim_hardware(6)


def _perception_sim(
    model: RobotModelConfig, hardware: HardwareComponent, sim_kwargs: dict[str, Any]
) -> tuple[Blueprint, ...]:
    """Scan, pick and place with an xArm in MuJoCo, finding objects with its wrist camera.

    UrdfManager serves the URDF to web 3D views. The planner already puts every
    link on tf, so the manager leaves joint tf off (on, links would get two parents).

    Args:
        model: The arm's planning model, with every link on tf.
        hardware: The simulated arm and its gripper, named "arm".
        sim_kwargs: MujocoSimModule settings: the scene and the wrist camera's link.
    """
    return (
        ManipulationModule.blueprint(
            gripper_tasks={"manipulator": "arm_gripper"},
            model=model,
            planning_timeout=10.0,
            visualization={"backend": "viser"},
        ),
        ManipulationSkills.blueprint(),
        PickAndPlaceModule.blueprint(planning_frame="world", robot=model),
        HeuristicGraspModule.blueprint(**XARM_HEURISTIC_AIM),
        MujocoSimModule.blueprint(**sim_kwargs),
        # The wrist camera holds still while the arm waits, so one view confirms an object.
        ManipulationLocalize.blueprint(world_frame="world", params=LocalizeParams(min_views=1)),
        coordinator(
            hardware=[hardware],
            tasks=[
                trajectory_task(hardware),
                TaskConfig(
                    name="arm_gripper",
                    type="gripper",
                    joint_names=["arm/gripper"],
                    priority=20,
                ),
            ],
        ),
        UrdfManager.blueprint(model=model.model, publish_joint_tf=False),
        RerunBridgeModule.blueprint(),
    )


xarm_perception_sim = autoconnect(
    *_perception_sim(_xarm7_sim_model, _xarm7_sim_hw, _xarm7_sim_kwargs)
)

# The same stack on the xArm6, its wrist camera on link6.
xarm6_sim = autoconnect(
    *_perception_sim(
        _xarm6_sim_model, _xarm6_sim_hw, make_xarm_sim_module_kwargs(6, XARM6_SIM_PATH)
    )
)

# Robot-only stack: control and sensors, exposed as plain Zenoh topics for agents without dimOS.
xarm_sim = autoconnect(
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
                robot_model=_xarm7_sim_model,
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

# teleop-webxr-xarm6 over the arm in MuJoCo: the Quest's right controller drives
# the arm and its trigger the gripper. Drawn in web 3D views like xarm6_sim.
teleop_webxr_xarm6_sim = autoconnect(
    ArmTeleopModule.blueprint(),
    xarm6_teleop_control(_xarm6_sim_hw, _xarm6_sim_model),
    MujocoSimModule.blueprint(**make_xarm_sim_module_kwargs(6, XARM6_SIM_PATH)),
    UrdfManager.blueprint(model=_xarm6_sim_model.model, publish_joint_tf=False),
).remappings(
    [
        (ArmTeleopModule, "right_controller_output", "right_cartesian_command"),
        (ArmTeleopModule, "right_gripper_command", "right_gripper_command"),
    ]
)
