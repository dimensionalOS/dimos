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

"""Real-hardware xArm perception manipulation blueprints."""

from __future__ import annotations

import math

from dimos.core.coordination.blueprints import autoconnect
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.manipulation.grasping.heuristic_grasp import HeuristicGraspModule
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.manipulation_skills import ManipulationSkills
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.perception.experimental.object_scene_registration import ObjectSceneRegistrationModule
from dimos.protocol.tf.static_tf_publisher import StaticTfPublisher
from dimos.robot.manipulators.xarm.config import make_xarm7_model_config

XARM_PERCEPTION_CAMERA_TRANSFORM = Transform(
    translation=Vector3(x=0.06693724, y=-0.0309563, z=0.00691482),
    rotation=Quaternion(0.70513398, 0.00535696, 0.70897578, -0.01052180),  # xyzw
    frame_id="link7",
    child_frame_id="camera_link",
)


class _XArmCameraMountTfPublisher(StaticTfPublisher):
    def transforms(self) -> list[Transform]:
        return [XARM_PERCEPTION_CAMERA_TRANSFORM]


xarm_perception = autoconnect(
    ManipulationModule.blueprint(
        model=make_xarm7_model_config(
            add_gripper=True,
            gripper_hardware_id="arm",
            base_pose=PoseStamped(
                frame_id="world",
                orientation=Quaternion.from_euler(Vector3(0.0, math.radians(45), 0.0)),
            ),
            tf_extra_links=["link7"],
        ),
        planning_timeout=10.0,
        visualization={"backend": "viser"},
        floor_z=-0.02,
    ),
    ManipulationSkills.blueprint(),
    PickAndPlaceModule.blueprint(planning_frame="world"),
    HeuristicGraspModule.blueprint(),
    _XArmCameraMountTfPublisher.blueprint(),
    RealSenseCamera.blueprint(),
    ObjectSceneRegistrationModule.blueprint(
        target_frame="world",
        optical_frame="camera_color_optical_frame",
    ),
).global_config(n_workers=4)
