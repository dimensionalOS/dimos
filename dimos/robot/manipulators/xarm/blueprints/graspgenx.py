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

"""GraspGenX-enabled real-hardware xArm perception blueprint."""

from __future__ import annotations

import math

from dimos.core.coordination.blueprints import autoconnect
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.manipulation.grasping.grasp_gen_x import GraspGenXModule
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.manipulation_skills import ManipulationSkills
from dimos.manipulation.pick_and_place import PickAndPlaceModule
from dimos.perception.experimental.object_scene_registration import ObjectSceneRegistrationModule
from dimos.robot.manipulators.xarm.blueprints.perception import XARM_PERCEPTION_CAMERA_TRANSFORM
from dimos.robot.manipulators.xarm.config import make_xarm7_model_config
from dimos.robot.manipulators.xarm.grasp_config import make_xarm_graspgenx_config

_graspgenx_config = make_xarm_graspgenx_config()

xarm_graspgenx = autoconnect(
    ManipulationModule.blueprint(
        robots=[
            make_xarm7_model_config(
                name="arm",
                add_gripper=True,
                pitch=math.radians(45),
                tf_extra_links=["link7"],
            )
        ],
        planning_timeout=10.0,
        floor_z=-0.02,
    ),
    ManipulationSkills.blueprint(instance_name="manipulation_skills"),
    PickAndPlaceModule.blueprint(
        instance_name="pick_and_place",
        grasp="graspgenx",
    ),
    RealSenseCamera.blueprint(
        base_frame_id="link7",
        base_transform=XARM_PERCEPTION_CAMERA_TRANSFORM,
    ),
    ObjectSceneRegistrationModule.blueprint(target_frame="world"),
    GraspGenXModule.blueprint(
        **_graspgenx_config.model_dump(exclude={"rpc_transport", "tf_transport", "g"})
    ),
).global_config(n_workers=5)
