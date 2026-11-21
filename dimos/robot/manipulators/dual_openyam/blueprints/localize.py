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

"""Dual OpenYAM with the overhead D405, localize and grasp poses (heuristic, GraspGenX).

``dimos run dual-openyam-localize --left-can-port follower_l --right-can-port follower_r``

The overhead camera sits at its calibrated pose in the world, so localize answers in
the planner's frame. Each wrist D405 publishes its own color and depth streams,
``left_wrist_color_image`` and so on; their mounts are not calibrated, so localize
does not use them. Each arm's tip is its fingertip centre, so a localized position
is a pose an arm can be sent to. The table is a planner obstacle; objects on it are
not (no voxel map).
"""

from __future__ import annotations

from dimos.agents.skills.observe_skill import ObserveSkill
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.manipulation.grasping.grasp_gen_x.module import GraspGenXModule
from dimos.manipulation.grasping.heuristic_grasp import HeuristicGraspModule
from dimos.manipulation.manipulation_localize import ManipulationLocalize
from dimos.manipulation.manipulation_skills import ManipulationSkills
from dimos.perception.localize.types import LocalizeParams
from dimos.robot.manipulators.common.blueprints import planner
from dimos.robot.manipulators.dual_openyam.blueprints.basic import (
    dual_openyam_control,
)
from dimos.robot.manipulators.dual_openyam.config import (
    DUAL_OPENYAM_GRASP_FRAME_TO_TCP,
    DUAL_OPENYAM_GRIPPER_SWEEP_VOLUME,
    DUAL_OPENYAM_HEURISTIC_AIM,
    DUAL_OPENYAM_OVERHEAD_CAMERA_TRANSFORM,
    DUAL_OPENYAM_TABLE_BOXES,
    dual_openyam_tcp_model_config,
)

_REALSENSE_OUTPUTS = (
    "color_image",
    "depth_image",
    "infrared_left",
    "infrared_right",
    "imu",
    "imu_info",
    "pointcloud",
    "camera_info",
    "depth_camera_info",
    "infrared_left_camera_info",
    "infrared_right_camera_info",
    "tf",
)


def dual_openyam_wrist_camera(side: str) -> Blueprint:
    """One wrist D405 under its own stream names, e.g. ``left_wrist_color_image``.

    Its serial comes from the run config, e.g. ``LEFT_WRIST_CAMERA__SERIAL_NUMBER``.
    """
    name = f"{side}_wrist"
    return RealSenseCamera.blueprint(
        instance_name=f"{name}_camera",
        frame_id_prefix=name,
        width=640,
        # 15 fps keeps three D405s on USB 2 within bandwidth.
        fps=15,
    ).remappings([(f"{name}_camera", stream, f"{name}_{stream}") for stream in _REALSENSE_OUTPUTS])


dual_openyam_localize = autoconnect(
    planner(
        # Poses place the fingertip centre; fingers-down is roll pi.
        model=dual_openyam_tcp_model_config(robot="arm"),
        gripper_tasks={
            "left_manipulator": "left_arm_gripper",
            "right_manipulator": "right_arm_gripper",
        },
        static_transforms=[DUAL_OPENYAM_OVERHEAD_CAMERA_TRANSFORM],
        world_frame="world",
        # The table as a box between the heights the two arms touched it at.
        table_boxes=DUAL_OPENYAM_TABLE_BOXES,
    ),
    dual_openyam_control(),
    ManipulationSkills.blueprint(),
    RealSenseCamera.blueprint(width=640, fps=30),
    # The overhead camera never moves, so every sighting is one view.
    ManipulationLocalize.blueprint(world_frame="world", params=LocalizeParams(min_views=1)),
    # Top-down grasp poses (position and jaw angle) from an object's cloud.
    HeuristicGraspModule.blueprint(**DUAL_OPENYAM_HEURISTIC_AIM),
    # Learned 6-DoF grasp poses from an object's cloud, in the tcp frame.
    GraspGenXModule.blueprint(
        gripper=DUAL_OPENYAM_GRIPPER_SWEEP_VOLUME,
        grasp_frame_to_tcp=DUAL_OPENYAM_GRASP_FRAME_TO_TCP,
    ),
    ObserveSkill.blueprint(),
    dual_openyam_wrist_camera("left"),
    dual_openyam_wrist_camera("right"),
)
